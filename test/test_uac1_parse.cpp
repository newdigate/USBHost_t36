// Copyright (c) 2026 Nicholas Newdigate
// SPDX-License-Identifier: MIT
#include "usb_audio_parse.h"
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>

static int failures = 0;
static int checks = 0;

#define CHECK(cond) do { checks++; if (!(cond)) { \
	printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); failures++; } } while (0)

// Both operands are cast to long, so this is only suitable for values that
// fit in a long -- not pointers or 64-bit-wide fields.
#define CHECK_EQ(a, b) do { checks++; long _ck_a = (long)(a), _ck_b = (long)(b); \
	if (_ck_a != _ck_b) { printf("FAIL %s:%d: %s == %s (got %ld, want %ld)\n", \
	__FILE__, __LINE__, #a, #b, _ck_a, _ck_b); failures++; } } while (0)

static uint8_t fixture[4096];
static size_t fixture_len;

static void load_fixture(void)
{
	const char *path = "fixtures/headset_uac1_config.bin";
	FILE *f = fopen(path, "rb");
	if (!f) {
		printf("FAIL cannot open fixture %s: %s\n", path, strerror(errno));
		exit(1);
	}
	fixture_len = fread(fixture, 1, sizeof(fixture), f);
	fclose(f);
	if (fixture_len == sizeof(fixture)) {
		printf("FAIL fixture %s may have been truncated (hit %zu byte buffer limit)\n",
			path, sizeof(fixture));
		exit(1);
	}
}

static void test_fixture_is_a_config_descriptor(void)
{
	CHECK_EQ(fixture_len, 799);
	CHECK_EQ(fixture[0], 9);      // bLength
	CHECK_EQ(fixture[1], 0x02);   // CONFIGURATION
	CHECK_EQ(fixture[2] | (fixture[3] << 8), 799);  // wTotalLength
	CHECK_EQ(fixture[4], 4);      // bNumInterfaces
}

static void test_rejects_garbage(void)
{
	UAC1Topology t;
	uint8_t junk[16] = {0};
	CHECK(!uac1_parse_config(junk, sizeof(junk), &t));
	CHECK(!uac1_parse_config(0, 0, &t));
}

static void test_identifies_interfaces(void)
{
	UAC1Topology t;
	CHECK(uac1_parse_config(fixture, fixture_len, &t));
	CHECK_EQ(t.bcd_adc, 0x0100);           // UAC 1.00
	CHECK_EQ(t.control_interface, 0);
	CHECK_EQ(t.streaming_interface, 2);    // not 1, which is the microphone
}

static void test_collects_alt_settings(void)
{
	UAC1Topology t;
	CHECK(uac1_parse_config(fixture, fixture_len, &t));
	CHECK_EQ(t.alt_count, 8);              // alts 0..7 on interface 2

	// alt 0 is the zero-bandwidth setting: no endpoint
	CHECK_EQ(t.alts[0].alternate_setting, 0);
	CHECK_EQ(t.alts[0].endpoint_address, 0);

	// alt 7 is 48 kHz stereo 16-bit
	CHECK_EQ(t.alts[7].alternate_setting, 7);
	CHECK_EQ(t.alts[7].rate_count, 1);
	CHECK_EQ(t.alts[7].rates[0], 48000);
	CHECK_EQ(t.alts[7].channels, 2);
	CHECK_EQ(t.alts[7].bit_resolution, 16);
	CHECK_EQ(t.alts[7].subframe_size, 2);
	CHECK_EQ(t.alts[7].endpoint_address, 0x04);
	CHECK_EQ(t.alts[7].max_packet_size, 248);
	// isochronous (bits 1:0 == 01) and adaptive (bits 3:2 == 10) => 0x09
	CHECK_EQ(t.alts[7].endpoint_attributes, 0x09);

	// alt 6 is 44.1 kHz stereo 16-bit
	CHECK_EQ(t.alts[6].rate_count, 1);
	CHECK_EQ(t.alts[6].rates[0], 44100);
	CHECK_EQ(t.alts[6].max_packet_size, 228);
}

static void test_resolves_speaker_feature_unit(void)
{
	UAC1Topology t;
	CHECK(uac1_parse_config(fixture, fixture_len, &t));
	// Output terminal 16 is a Speaker (0x0301) sourced from unit 22.
	// Units 19 and 35 are microphone feature units and must not be chosen.
	CHECK_EQ(t.feature_unit_id, 22);
}

static void test_finds_alt_by_format(void)
{
	UAC1Topology t;
	CHECK(uac1_parse_config(fixture, fixture_len, &t));
	CHECK_EQ(uac1_find_alt(&t, 48000, 2, 16), 7);   // bring-up target
	CHECK_EQ(uac1_find_alt(&t, 44100, 2, 16), 6);   // shipping target
	CHECK_EQ(uac1_find_alt(&t,  8000, 2, 16), 1);
	CHECK_EQ(uac1_find_alt(&t, 96000, 2, 16), -1);  // unsupported rate
	CHECK_EQ(uac1_find_alt(&t, 48000, 1, 16), -1);  // unsupported channel count
	CHECK_EQ(uac1_find_alt(0,   48000, 2, 16), -1);
}

// USBDriver::claim() at type 0 receives `enumbuf + 9` -- the descriptor set
// with the 9-byte configuration header already skipped. Parsing either form
// must give the same answer, or the driver will disagree with these tests.
static void test_parses_without_config_header(void)
{
	UAC1Topology full, offset;
	CHECK(uac1_parse_config(fixture, fixture_len, &full));
	CHECK(uac1_parse_config(fixture + 9, fixture_len - 9, &offset));
	CHECK_EQ(offset.streaming_interface, full.streaming_interface);
	CHECK_EQ(offset.control_interface,   full.control_interface);
	CHECK_EQ(offset.feature_unit_id,     full.feature_unit_id);
	CHECK_EQ(offset.alt_count,           full.alt_count);
	CHECK_EQ(uac1_find_alt(&offset, 48000, 2, 16), 7);
}

// This parser sits in front of descriptor bytes from an arbitrary plugged-in
// device, so it must survive malformed and adversarial input without
// crashing or reading out of bounds. test_rejects_garbage above only tries a
// 16-byte zero buffer; this pins the specific safety properties the review
// had to reconstruct by hand. Build once with -fsanitize=address,undefined
// when changing this function.
static void test_survives_hostile_input(void)
{
	// (a) bLength == 0 embedded mid-stream must stop the parse rather than
	// loop forever. This is the single most safety-critical invariant in
	// the file: both parse loops require desc[i] >= 2 to continue.
	{
		uint8_t buf[11] = {
			9, 0x04, 0, 0, 1, 0x01, 0x02, 0, 0,  // AudioStreaming interface, no endpoint yet
			0, 0,                                 // bLength == 0 -- must stop here
		};
		UAC1Topology t;
		CHECK(!uac1_parse_config(buf, sizeof(buf), &t));
	}

	// (b) An endpoint descriptor claiming a bLength that overruns the real
	// buffer (200, with only 7 bytes actually present) must not be read
	// past the buffer end.
	{
		uint8_t buf[16] = {
			9, 0x04, 0, 0, 1, 0x01, 0x02, 0, 0,    // AudioStreaming interface
			200, 0x05, 0x04, 0x01, 0, 0, 0,        // endpoint claims bLength 200
		};
		UAC1Topology t;
		CHECK(!uac1_parse_config(buf, sizeof(buf), &t));
	}

	// (c) A streaming interface with only an IN endpoint (0x81) must not be
	// selected as the playback interface.
	{
		uint8_t buf[16] = {
			9, 0x04, 0, 0, 1, 0x01, 0x02, 0, 0,   // AudioStreaming interface
			7, 0x05, 0x81, 0x01, 0x40, 0, 1,      // IN, isochronous endpoint
		};
		UAC1Topology t;
		CHECK(!uac1_parse_config(buf, sizeof(buf), &t));
	}

	// (d) Truncating the real fixture at every possible length must never
	// crash or read out of bounds. The return value is unconstrained here
	// (some prefixes may parse to true, most to false); what matters is
	// that the loop below completes -- verified for real by running under
	// ASan/UBSan.
	{
		UAC1Topology t;
		size_t survived = 0;
		for (size_t n = 0; n <= fixture_len; n++) {
			uac1_parse_config(fixture, n, &t);
			survived++;
		}
		CHECK_EQ(survived, fixture_len + 1);
	}

	// (e) A device advertising more than UAC1_MAX_ALTS (16) alternate
	// settings on one iso-OUT streaming interface must be capped, not
	// overflow out->alts[]. Built with a loop rather than a giant literal.
	{
		const uint8_t num_alts = 20;  // > UAC1_MAX_ALTS
		uint8_t buf[20 * 16];
		size_t pos = 0;
		for (uint8_t k = 0; k < num_alts; k++) {
			buf[pos + 0] = 9;     // bLength
			buf[pos + 1] = 0x04;  // INTERFACE
			buf[pos + 2] = 5;     // bInterfaceNumber (same iface, every alt)
			buf[pos + 3] = k;     // bAlternateSetting
			buf[pos + 4] = 1;     // bNumEndpoints
			buf[pos + 5] = 0x01;  // bInterfaceClass = AUDIO
			buf[pos + 6] = 0x02;  // bInterfaceSubClass = STREAMING
			buf[pos + 7] = 0;
			buf[pos + 8] = 0;
			pos += 9;

			buf[pos + 0] = 7;     // bLength
			buf[pos + 1] = 0x05;  // ENDPOINT
			buf[pos + 2] = 0x04;  // OUT
			buf[pos + 3] = 0x01;  // isochronous
			buf[pos + 4] = 0x40;
			buf[pos + 5] = 0;
			buf[pos + 6] = 1;
			pos += 7;
		}

		UAC1Topology t;
		CHECK(uac1_parse_config(buf, pos, &t));
		CHECK_EQ(t.alt_count, UAC1_MAX_ALTS);
	}
}

static void test_frame_bytes(void)
{
	// 48 kHz divides evenly: 48 samples every frame, forever.
	uint32_t acc = 0;
	for (int i = 0; i < 100; i++) {
		CHECK_EQ(uac1_frame_bytes(&acc, 48000, 2, 2), 192);
	}
	CHECK_EQ(acc, 0);   // no fraction ever accumulates

	// 44.1 kHz does not: 44 samples nine times then 45, repeating. Over any
	// 10 frames exactly 441 samples must go out, or the stream drifts.
	acc = 0;
	int small = 0, large = 0;
	uint32_t total_samples = 0;
	for (int i = 0; i < 10; i++) {
		uint16_t b = uac1_frame_bytes(&acc, 44100, 2, 2);
		if (b == 176) small++;
		else if (b == 180) large++;
		else CHECK_EQ(b, 0);      // any other size is a failure
		total_samples += b / 4;
	}
	CHECK_EQ(small, 9);
	CHECK_EQ(large, 1);
	CHECK_EQ(total_samples, 441);

	// And it keeps averaging over the long run rather than drifting.
	acc = 0;
	total_samples = 0;
	for (int i = 0; i < 1000; i++) total_samples += uac1_frame_bytes(&acc, 44100, 2, 2) / 4;
	CHECK_EQ(total_samples, 44100);   // exactly one second of audio

	// Mono and 8-bit scale as expected.
	acc = 0;
	CHECK_EQ(uac1_frame_bytes(&acc, 48000, 1, 2), 96);

	// Rejections.
	acc = 0;
	CHECK_EQ(uac1_frame_bytes(0, 48000, 2, 2), 0);
	CHECK_EQ(uac1_frame_bytes(&acc, 0, 2, 2), 0);
	CHECK_EQ(uac1_frame_bytes(&acc, 48000, 0, 2), 0);
	CHECK_EQ(uac1_frame_bytes(&acc, 48000, 2, 0), 0);
}

static uint8_t jabra[4096];
static size_t jabra_len;

// Jabra 0B0E:2301, captured from the device with the descriptor survey. It
// uses the other UAC1 rate idiom: five discrete rates in ONE alternate
// setting, where the Logitech uses one rate per alt across eight of them.
// A parser reading only tSamFreq[0] sees this as an 8 kHz-only device.
static void test_multirate_device(void)
{
	FILE *f = fopen("fixtures/jabra_uac1_multirate.bin", "rb");
	if (!f) { printf("FAIL cannot open jabra fixture\n"); failures++; return; }
	jabra_len = fread(jabra, 1, sizeof(jabra), f);
	fclose(f);
	CHECK_EQ(jabra_len, 272);

	UAC1Topology t;
	CHECK(uac1_parse_config(jabra, jabra_len, &t));
	CHECK_EQ(t.bcd_adc, 0x0100);
	CHECK_EQ(t.streaming_interface, 2);      // interface 1 is the microphone

	// Two alts: 0 is zero-bandwidth, 1 carries the stream.
	CHECK_EQ(t.alt_count, 2);
	CHECK_EQ(t.alts[1].endpoint_address, 0x04);
	CHECK_EQ(t.alts[1].channels, 1);         // mono
	CHECK_EQ(t.alts[1].bit_resolution, 16);

	// All five rates must be recorded, not just the first.
	CHECK_EQ(t.alts[1].rate_count, 5);
	CHECK_EQ(t.alts[1].rates[0], 8000);
	CHECK_EQ(t.alts[1].rates[1], 16000);
	CHECK_EQ(t.alts[1].rates[2], 32000);
	CHECK_EQ(t.alts[1].rates[3], 44100);
	CHECK_EQ(t.alts[1].rates[4], 48000);

	// The bug this fixture exists for: 44.1 kHz is in the list, so a mono
	// 16-bit request at 44100 must match. Before the fix this returned -1
	// because only tSamFreq[0] (8000) was stored.
	CHECK_EQ(uac1_find_alt(&t, 44100, 1, 16), 1);
	CHECK_EQ(uac1_find_alt(&t, 48000, 1, 16), 1);
	CHECK_EQ(uac1_find_alt(&t,  8000, 1, 16), 1);
	CHECK_EQ(uac1_find_alt(&t, 96000, 1, 16), -1);   // not offered
	CHECK_EQ(uac1_find_alt(&t, 44100, 2, 16), -1);   // mono device, not stereo

	// The device advertises the sampling frequency control (CS_ENDPOINT
	// bmAttributes bit 0), and offers five rates, so SET_INTERFACE alone
	// leaves the rate undetermined -- the host must also issue SET_CUR.
	CHECK_EQ(t.alts[1].ep_controls, 0x01);
	CHECK_EQ(uac1_alt_needs_rate_request(&t.alts[1]), true);
}

static void test_needs_rate_request(void)
{
	UAC1AltSetting a = {};

	// One rate, control present: the alt setting already fixes the rate.
	a.ep_controls = 0x01; a.rate_count = 1; a.rates[0] = 48000;
	CHECK_EQ(uac1_alt_needs_rate_request(&a), false);

	// Several rates, control present: must be told which one.
	a.rate_count = 2; a.rates[1] = 44100;
	CHECK_EQ(uac1_alt_needs_rate_request(&a), true);

	// Continuous range, control present.
	UAC1AltSetting c = {};
	c.ep_controls = 0x01; c.rate_count = 0; c.rate_min = 8000; c.rate_max = 48000;
	CHECK_EQ(uac1_alt_needs_rate_request(&c), true);

	// No sampling frequency control: nothing to send, whatever the rate list
	// says. Sending SET_CUR to a device that does not implement the control
	// risks a stall, so this must stay false.
	UAC1AltSetting n = {};
	n.ep_controls = 0x00; n.rate_count = 3;
	CHECK_EQ(uac1_alt_needs_rate_request(&n), false);

	CHECK_EQ(uac1_alt_needs_rate_request(0), false);
}

static void test_supports_rate(void)
{
	UAC1AltSetting a = {};

	// Discrete list.
	a.rate_count = 3;
	a.rates[0] = 44100; a.rates[1] = 48000; a.rates[2] = 96000;
	CHECK_EQ(uac1_alt_supports_rate(&a, 44100), true);
	CHECK_EQ(uac1_alt_supports_rate(&a, 96000), true);
	CHECK_EQ(uac1_alt_supports_rate(&a, 22050), false);

	// Continuous range: rate_count 0 with bounds.
	UAC1AltSetting c = {};
	c.rate_count = 0; c.rate_min = 8000; c.rate_max = 48000;
	CHECK_EQ(uac1_alt_supports_rate(&c, 8000), true);
	CHECK_EQ(uac1_alt_supports_rate(&c, 44100), true);
	CHECK_EQ(uac1_alt_supports_rate(&c, 48000), true);
	CHECK_EQ(uac1_alt_supports_rate(&c, 7999), false);
	CHECK_EQ(uac1_alt_supports_rate(&c, 48001), false);

	// Both bounds zero means the format descriptor was missing, which must
	// not be read as "accepts anything".
	UAC1AltSetting z = {};
	CHECK_EQ(uac1_alt_supports_rate(&z, 44100), false);

	CHECK_EQ(uac1_alt_supports_rate(0, 44100), false);
	CHECK_EQ(uac1_alt_supports_rate(&a, 0), false);
}

// Generalplus 1B3F:2008, captured with the descriptor survey. A second,
// independent witness for the multi-rate idiom -- and unlike the Jabra it is
// stereo 16-bit, so USBAudioOut actually claims it. Under the old parser only
// tSamFreq[0] (44100) was stored, so the driver's default 48 kHz found no
// match and the device was silently not claimed.
static void test_multirate_stereo_device(void)
{
	static uint8_t buf[4096];
	FILE *f = fopen("fixtures/generalplus_uac1_multirate.bin", "rb");
	if (!f) { printf("FAIL cannot open generalplus fixture\n"); failures++; return; }
	size_t len = fread(buf, 1, sizeof(buf), f);
	fclose(f);
	CHECK_EQ(len, 244);

	UAC1Topology t;
	CHECK(uac1_parse_config(buf, len, &t));
	CHECK_EQ(t.bcd_adc, 0x0100);
	CHECK_EQ(t.streaming_interface, 1);
	CHECK_EQ(t.alt_count, 2);

	const UAC1AltSetting *a = &t.alts[1];
	CHECK_EQ(a->endpoint_address, 0x05);
	CHECK_EQ(a->channels, 2);
	CHECK_EQ(a->bit_resolution, 16);
	CHECK_EQ(a->max_packet_size, 192);
	CHECK_EQ(a->rate_count, 2);
	CHECK_EQ(a->rates[0], 44100);
	CHECK_EQ(a->rates[1], 48000);

	// Both offered rates resolve to the one alt setting.
	CHECK_EQ(uac1_find_alt(&t, 48000, 2, 16), 1);
	CHECK_EQ(uac1_find_alt(&t, 44100, 2, 16), 1);
	CHECK_EQ(uac1_find_alt(&t, 96000, 2, 16), -1);

	// Two rates behind one alt, with the sampling frequency control present:
	// SET_INTERFACE cannot determine the rate, so SET_CUR is required.
	CHECK_EQ(a->ep_controls, 0x01);
	CHECK_EQ(uac1_alt_needs_rate_request(a), true);
}

// XMOS xcore-200 MC Audio built as 1AMi2o2xxxxxx with 16-bit output --
// descriptors extracted from the .xe before flashing. First device with two
// endpoints in one alternate setting: an asynchronous data endpoint plus a
// feedback endpoint. The parser used to overwrite the data endpoint with the
// feedback one, so the driver would have streamed 196-byte packets at a 3-byte
// endpoint.
static void test_async_feedback_device(void)
{
	static uint8_t buf[4096];
	FILE *f = fopen("fixtures/xmos_uac1_async_feedback.bin", "rb");
	if (!f) { printf("FAIL cannot open xmos fixture\n"); failures++; return; }
	size_t len = fread(buf, 1, sizeof(buf), f);
	fclose(f);
	CHECK_EQ(len, 233);

	UAC1Topology t;
	CHECK(uac1_parse_config(buf, len, &t));
	CHECK_EQ(t.bcd_adc, 0x0100);
	CHECK_EQ(t.streaming_interface, 1);

	const UAC1AltSetting *a = &t.alts[1];
	// The data endpoint, not the feedback endpoint.
	CHECK_EQ(a->endpoint_address, 0x01);
	CHECK_EQ(a->max_packet_size, 196);
	CHECK_EQ(a->channels, 2);
	CHECK_EQ(a->bit_resolution, 16);

	// bmAttributes 0x05: isochronous, asynchronous.
	CHECK_EQ(a->endpoint_attributes, 0x05);
	CHECK_EQ((a->endpoint_attributes >> 2) & 0x03, 1);

	// Feedback endpoint comes from bSynchAddress on the data endpoint. Its own
	// usage bits say "data", so they cannot be used to find it.
	CHECK_EQ(a->feedback_endpoint, 0x82);
	CHECK_EQ(a->feedback_refresh, 4);      // 2^4 = 16 ms

	CHECK_EQ(uac1_find_alt(&t, 48000, 2, 16), 1);
	CHECK_EQ(uac1_find_alt(&t, 44100, 2, 16), 1);
	CHECK_EQ(uac1_alt_needs_rate_request(a), true);
}

// Devices with a single endpoint per alt must keep no feedback endpoint.
static void test_no_feedback_on_sync_devices(void)
{
	static uint8_t buf[4096];
	const char *files[] = { "fixtures/headset_uac1_config.bin",
	                        "fixtures/generalplus_uac1_multirate.bin" };
	for (int k = 0; k < 2; k++) {
		FILE *f = fopen(files[k], "rb");
		if (!f) { printf("FAIL cannot open %s\n", files[k]); failures++; continue; }
		size_t len = fread(buf, 1, sizeof(buf), f);
		fclose(f);
		UAC1Topology t;
		CHECK(uac1_parse_config(buf, len, &t));
		for (uint8_t i = 1; i < t.alt_count; i++) {
			CHECK_EQ(t.alts[i].feedback_endpoint, 0);
			CHECK_EQ((t.alts[i].endpoint_address & 0x80), 0);   // OUT
		}
	}
}

// The millihertz variant must behave identically to the hertz one at whole
// rates, and resolve trims far finer than one hertz -- at 44.1 kHz a single
// hertz is already 22.7 ppm, which is coarser than the crystal error being
// corrected for.
static void test_frame_bytes_mhz(void)
{
	uint32_t acc = 0;
	for (int i = 0; i < 100; i++) {
		CHECK_EQ(uac1_frame_bytes_mhz(&acc, 48000000u, 2, 2), 192);
	}
	CHECK_EQ(acc, 0);

	// Same 9x176 + 1x180 pattern as the hertz version.
	acc = 0;
	int small = 0, large = 0;
	for (int i = 0; i < 10; i++) {
		uint16_t b = uac1_frame_bytes_mhz(&acc, 44100000u, 2, 2);
		if (b == 176) small++;
		else if (b == 180) large++;
		else CHECK_EQ(b, 0);
	}
	CHECK_EQ(small, 9);
	CHECK_EQ(large, 1);

	// Exactly one second of audio, no drift.
	acc = 0;
	uint32_t total = 0;
	for (int i = 0; i < 1000; i++) total += uac1_frame_bytes_mhz(&acc, 44100000u, 2, 2) / 4;
	CHECK_EQ(total, 44100);

	// +200 ppm of 44100 is 8.82 Hz, which one hertz of resolution could not
	// express. Over a second it must show up as ~9 extra samples.
	acc = 0;
	total = 0;
	for (int i = 0; i < 1000; i++) total += uac1_frame_bytes_mhz(&acc, 44108820u, 2, 2) / 4;
	CHECK_EQ(total, 44108);

	// And a negative trim removes samples.
	acc = 0;
	total = 0;
	for (int i = 0; i < 1000; i++) total += uac1_frame_bytes_mhz(&acc, 44091180u, 2, 2) / 4;
	CHECK_EQ(total, 44091);

	// Guards match the hertz version.
	acc = 0;
	CHECK_EQ(uac1_frame_bytes_mhz(&acc, 0, 2, 2), 0);
	CHECK_EQ(uac1_frame_bytes_mhz(&acc, 44100000u, 0, 2), 0);
	CHECK_EQ(uac1_frame_bytes_mhz(0, 44100000u, 2, 2), 0);
}

static void test_uframe_bytes(void)
{
	// 44.1 kHz across 8 kHz microframes: 5.5125 samples/uframe. Over 80
	// microframes exactly 441 samples must emerge, sizes only 5 or 6.
	uint32_t accum = 0;
	uint32_t total = 0;
	for (int i = 0; i < 80; i++) {
		uint16_t b = uac2_uframe_bytes_mhz(&accum, 44100000u, 8, 4);
		CHECK_EQ(b % 32, 0);               // whole 8ch x 4B frames only
		uint16_t samples = b / 32;
		CHECK_EQ(samples == 5 || samples == 6, true);
		total += samples;
	}
	CHECK_EQ(total, 441);
	CHECK_EQ(accum, 0);                    // exact over the repeat period
}


// --- endpoint classification: usage type, not direction ------------------
//
// USB 2.0 section 9.6.6 (page 298 of the 2024-09-27 revision), bmAttributes
// bits 5..4 Usage Type: 00 = Data endpoint, 01 = Feedback endpoint,
// 10 = Implicit feedback Data endpoint, 11 = Reserved.
//
// The parser used to call ANY non-OUT isochronous endpoint the feedback
// endpoint, on direction alone. That is correct only for devices with at
// most one IN endpoint per alt -- which was every device this bench had
// until a full-duplex one turned up. On a device carrying audio IN and audio
// OUT in one alternate setting it arms the feedback reader against the
// microphone stream, and the rate decoder then reads audio samples.

static uint8_t *put_iface(uint8_t *p, uint8_t num, uint8_t alt, uint8_t neps,
                          uint8_t cls, uint8_t sub)
{
	*p++ = 9; *p++ = 0x04; *p++ = num; *p++ = alt; *p++ = neps;
	*p++ = cls; *p++ = sub; *p++ = 0x00; *p++ = 0x00;
	return p;
}

static uint8_t *put_fmt(uint8_t *p, uint8_t ch, uint8_t sub, uint8_t bits,
                        uint32_t rate)
{
	*p++ = 11; *p++ = 0x24; *p++ = 0x02; *p++ = 0x01;
	*p++ = ch; *p++ = sub; *p++ = bits; *p++ = 1;
	*p++ = rate & 0xFF; *p++ = (rate >> 8) & 0xFF; *p++ = (rate >> 16) & 0xFF;
	return p;
}

/* bLength 9 audio endpoint descriptor: adds bRefresh and bSynchAddress. */
static uint8_t *put_ep(uint8_t *p, uint8_t addr, uint8_t attr, uint16_t mps,
                       uint8_t refresh, uint8_t synch)
{
	*p++ = 9; *p++ = 0x05; *p++ = addr; *p++ = attr;
	*p++ = mps & 0xFF; *p++ = (mps >> 8) & 0xFF; *p++ = 1;
	*p++ = refresh; *p++ = synch;
	return p;
}

static void test_in_data_endpoint_is_not_feedback(void)
{
	/* A full-duplex alt: OUT audio (usage 00) and IN audio (usage 00).
	 * Neither declares a feedback endpoint, so there must not be one. */
	uint8_t d[128], *p = d;
	p = put_iface(p, 0, 0, 0, 0x01, 0x01);
	p = put_iface(p, 1, 1, 2, 0x01, 0x02);
	p = put_fmt(p, 2, 2, 16, 44100);
	p = put_ep(p, 0x01, 0x01, 192, 0, 0);
	p = put_ep(p, 0x81, 0x01, 192, 0, 0);      /* IN data, usage 00 */
	UAC1Topology t;
	CHECK_EQ(uac1_parse_config(d, (size_t)(p - d), &t), true);
	const UAC1AltSetting *a = &t.alts[t.alt_count - 1];
	CHECK_EQ(a->endpoint_address, 0x01);
	CHECK_EQ(a->feedback_endpoint, 0);         /* the defect: was 0x81 */
}

static void test_in_feedback_endpoint_is_recognised(void)
{
	uint8_t d[128], *p = d;
	p = put_iface(p, 0, 0, 0, 0x01, 0x01);
	p = put_iface(p, 1, 1, 2, 0x01, 0x02);
	p = put_fmt(p, 2, 2, 16, 44100);
	p = put_ep(p, 0x01, 0x05, 192, 0, 0);
	p = put_ep(p, 0x82, 0x11, 3, 4, 0);        /* IN, usage 01 = feedback */
	UAC1Topology t;
	CHECK_EQ(uac1_parse_config(d, (size_t)(p - d), &t), true);
	const UAC1AltSetting *a = &t.alts[t.alt_count - 1];
	CHECK_EQ(a->feedback_endpoint, 0x82);
	CHECK_EQ(a->feedback_refresh, 4);
}

static void test_implicit_feedback_data_is_not_a_feedback_endpoint(void)
{
	/* Usage type 10 is an implicit feedback DATA endpoint -- it carries
	 * audio. Decoding it as a rate would point the decoder at samples. */
	uint8_t d[128], *p = d;
	p = put_iface(p, 0, 0, 0, 0x01, 0x01);
	p = put_iface(p, 1, 1, 2, 0x01, 0x02);
	p = put_fmt(p, 2, 2, 16, 44100);
	p = put_ep(p, 0x01, 0x05, 192, 0, 0);
	p = put_ep(p, 0x83, 0x21, 192, 0, 0);      /* IN, usage 10 */
	UAC1Topology t;
	CHECK_EQ(uac1_parse_config(d, (size_t)(p - d), &t), true);
	CHECK_EQ(t.alts[t.alt_count - 1].feedback_endpoint, 0);
}

static void test_bsynchaddress_still_wins(void)
{
	/* The OUT endpoint naming its feedback partner via bSynchAddress is the
	 * common UAC1 shape and must keep working. */
	uint8_t d[128], *p = d;
	p = put_iface(p, 0, 0, 0, 0x01, 0x01);
	p = put_iface(p, 1, 1, 2, 0x01, 0x02);
	p = put_fmt(p, 2, 2, 16, 44100);
	p = put_ep(p, 0x01, 0x05, 192, 0, 0x82);
	p = put_ep(p, 0x82, 0x11, 3, 4, 0);
	UAC1Topology t;
	CHECK_EQ(uac1_parse_config(d, (size_t)(p - d), &t), true);
	CHECK_EQ(t.alts[t.alt_count - 1].feedback_endpoint, 0x82);
}

int main(void)
{
	load_fixture();
	test_fixture_is_a_config_descriptor();
	test_rejects_garbage();
	test_identifies_interfaces();
	test_collects_alt_settings();
	test_resolves_speaker_feature_unit();
	test_finds_alt_by_format();
	test_frame_bytes();
	test_frame_bytes_mhz();
	test_uframe_bytes();
	test_multirate_device();
	test_multirate_stereo_device();
	test_async_feedback_device();
	test_no_feedback_on_sync_devices();
	test_supports_rate();
	test_needs_rate_request();
	test_parses_without_config_header();
	test_survives_hostile_input();
	test_in_data_endpoint_is_not_feedback();
	test_in_feedback_endpoint_is_recognised();
	test_implicit_feedback_data_is_not_a_feedback_endpoint();
	test_bsynchaddress_still_wins();
	printf("%d checks, %d failures\n", checks, failures);
	return failures ? 1 : 0;
}
