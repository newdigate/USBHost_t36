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
	CHECK_EQ(t.alts[7].sample_rate, 48000);
	CHECK_EQ(t.alts[7].channels, 2);
	CHECK_EQ(t.alts[7].bit_resolution, 16);
	CHECK_EQ(t.alts[7].subframe_size, 2);
	CHECK_EQ(t.alts[7].endpoint_address, 0x04);
	CHECK_EQ(t.alts[7].max_packet_size, 248);
	// isochronous (bits 1:0 == 01) and adaptive (bits 3:2 == 10) => 0x09
	CHECK_EQ(t.alts[7].endpoint_attributes, 0x09);

	// alt 6 is 44.1 kHz stereo 16-bit
	CHECK_EQ(t.alts[6].sample_rate, 44100);
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
	test_parses_without_config_header();
	test_survives_hostile_input();
	printf("%d checks, %d failures\n", checks, failures);
	return failures ? 1 : 0;
}
