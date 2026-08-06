// Copyright (c) 2026 Nicholas Newdigate
// SPDX-License-Identifier: MIT
#include "usb_audio2_parse.h"
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>

static int failures = 0, checks = 0;
#define CHECK(cond) do { checks++; if (!(cond)) { \
	printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); failures++; } } while (0)
#define CHECK_EQ(a, b) do { checks++; long _ck_a = (long)(a), _ck_b = (long)(b); \
	if (_ck_a != _ck_b) { printf("FAIL %s:%d: %s == %s (got %ld, want %ld)\n", \
	__FILE__, __LINE__, #a, #b, _ck_a, _ck_b); failures++; } } while (0)

static void test_clock_cur_setup(void)
{
	// UAC2 5.2.5.1.1: SET CUR of CS_SAM_FREQ_CONTROL on clock entity 5,
	// AC interface 0; rate payload travels separately (4-byte LE).
	uint8_t s[8];
	uac2_clock_cur_setup(s, 0, 5);
	CHECK_EQ(s[0], 0x21);                  // class request, interface, H->D
	CHECK_EQ(s[1], 0x01);                  // CUR
	CHECK_EQ(s[2], 0x00); CHECK_EQ(s[3], 0x01);   // wValue: CS_SAM_FREQ<<8
	CHECK_EQ(s[4], 0x00); CHECK_EQ(s[5], 0x05);   // wIndex: (clockID<<8)|itf
	CHECK_EQ(s[6], 0x04); CHECK_EQ(s[7], 0x00);   // wLength 4

	// Non-zero AC interface lands in wIndex's low byte.
	uac2_clock_cur_setup(s, 3, 9);
	CHECK_EQ(s[4], 0x03);
	CHECK_EQ(s[5], 0x09);
}

// XMOS MC200 built as 2AMi8o8xxxxxx (8 channels each way), captured via
// lastConfig() from the claim path on hardware (enumbuf+9 form -- this
// fixture starts at the IAD, with the 9-byte CONFIGURATION header already
// stripped off, same as claim() receives it). Ground truth below was derived
// by hand-decoding fixtures/xmos_uac2_2ami8o8.bin with xxd and cross-checked
// with an independent descriptor-walk script. The landmark list below is the
// authoritative record. Landmarks:
//   IAD (first_if=0, count=3, UAC2 protocol 0x20)
//   AC interface 0, alt 0, protocol 0x20
//   CS AC HEADER: bcdADC 0x0200, wTotalLength 167 (offset 17, ends exactly
//     at offset 184 -- confirms the AC class descriptors are exactly bounded)
//   CLOCK_SOURCE id 0x29
//   CLOCK_SELECTOR id 0x28, 1 input pin, sourced from 0x29
//   INPUT_TERMINAL id 2 (USB streaming, i.e. the OUT/speaker path),
//     bCSourceID 0x28, 8 channels
//   FEATURE_UNIT id 0x0A, source terminal 2
//   OUTPUT_TERMINAL id 0x14 (Speaker), source 0x0A, clock 0x28
//   INPUT_TERMINAL id 1 (Microphone), bCSourceID 0x28, 8 channels
//   FEATURE_UNIT id 0x0B, source terminal 1
//   OUTPUT_TERMINAL id 0x16 (USB streaming, capture path), source 0x0B
//   AS interface 1 alt 0: zero-bandwidth (0 endpoints)
//   AS interface 1 alt 1: AS_GENERAL bTerminalLink=2, 8ch; FORMAT_TYPE I
//     subslot 4 / resolution 24; ENDPOINT OUT 0x01 iso-async wMaxPacketSize
//     800; ENDPOINT IN 0x82 iso feedback wMaxPacketSize 4
//   AS interface 1 alt 2: same terminal link/channels; FORMAT_TYPE I
//     subslot 2 / resolution 16; ENDPOINT OUT 0x01 wMaxPacketSize 400;
//     ENDPOINT IN 0x82 feedback
//   AS interface 2 (alt 0 zero-bandwidth, alt 1 one IN endpoint 0x81 iso,
//     wMaxPacketSize 800, AS_GENERAL bTerminalLink=0x16 / 8ch, FORMAT_TYPE I
//     subslot 4 / resolution 24) is the capture interface: no iso OUT
//     endpoint on any alt, so it is not `streaming_interface`, and it is now
//     collected separately as `input_streaming_interface` with its alts in
//     in_alts[]. Until 2026-08-05 those alts were dropped entirely, which is
//     why the host had no way to record from this device.
//   Trailing interface 3 (class 0xFE, DFU) is outside the audio IAD and is
//     naturally skipped (bInterfaceClass != AUDIO).
static uint8_t fixture[4096];
static size_t fixture_len;

static void load_fixture(void)
{
	const char *path = "fixtures/xmos_uac2_2ami8o8.bin";
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

static void test_fixture_is_the_captured_descriptor_set(void)
{
	CHECK_EQ(fixture_len, 372);
	CHECK_EQ(fixture[0], 8);      // bLength
	CHECK_EQ(fixture[1], 0x0B);   // IAD -- claim-time form starts here, not
	                              // at a CONFIGURATION descriptor
	CHECK_EQ(fixture[4], 0x01);   // bFunctionClass AUDIO
	CHECK_EQ(fixture[6], 0x20);   // bFunctionProtocol: UAC2 (IP_VERSION_02_00)
}

static void test_rejects_garbage(void)
{
	UAC1Topology t;
	uint8_t junk[16] = {0};
	CHECK(!uac2_parse_config(junk, sizeof(junk), &t));
	CHECK(!uac2_parse_config(0, 0, &t));
	CHECK(!uac2_parse_config(fixture, fixture_len, 0));
	CHECK(!uac2_parse_config(0, fixture_len, &t));
}

static void test_identifies_interfaces(void)
{
	UAC1Topology t;
	CHECK(uac2_parse_config(fixture, fixture_len, &t));
	CHECK_EQ(t.bcd_adc, 0x0200);           // UAC 2.00
	CHECK_EQ(t.control_interface, 0);
	// Interface 1, not 2 -- interface 2 is the capture-only (8ch IN) AS
	// interface and carries no iso OUT endpoint on either of its alts.
	CHECK_EQ(t.streaming_interface, 1);
	// ...which is exactly what makes it the INPUT interface. Interface 1
	// also has an IN endpoint (feedback 0x82), so direction alone would
	// have picked the wrong one; the absence of an iso OUT is what decides.
	CHECK_EQ(t.input_streaming_interface, 2);
}

// The load-bearing check: the clock chain is terminal(2) -CSourceID-> 0x28
// (a single-input CLOCK_SELECTOR) -baCSourceID[0]-> 0x29 (the CLOCK_SOURCE).
// A parser that rejects selectors outright, or that stops at the selector id
// instead of following it through, rejects this real device.
static void test_resolves_clock_source_through_selector(void)
{
	UAC1Topology t;
	CHECK(uac2_parse_config(fixture, fixture_len, &t));
	CHECK_EQ(t.clock_source_id, 0x29);
}

static void test_collects_alt_settings(void)
{
	UAC1Topology t;
	CHECK(uac2_parse_config(fixture, fixture_len, &t));
	CHECK_EQ(t.alt_count, 3);              // alts 0, 1, 2 on interface 1

	// alt 0 is the zero-bandwidth setting: no endpoint.
	CHECK_EQ(t.alts[0].alternate_setting, 0);
	CHECK_EQ(t.alts[0].endpoint_address, 0);

	// alt 1: 8 channels, 24-in-4 (subslot 4, resolution 24).
	CHECK_EQ(t.alts[1].alternate_setting, 1);
	CHECK_EQ(t.alts[1].channels, 8);
	CHECK_EQ(t.alts[1].subframe_size, 4);
	CHECK_EQ(t.alts[1].bit_resolution, 24);
	CHECK_EQ(t.alts[1].endpoint_address, 0x01);
	CHECK_EQ(t.alts[1].max_packet_size, 800);
	CHECK_EQ(t.alts[1].feedback_endpoint, 0x82);
	CHECK_EQ(t.alts[1].feedback_max_packet, 4);
	CHECK_EQ(t.alts[1].feedback_endpoint & 0x80, 0x80);   // IN bit set
	// UAC2 rates are runtime RANGE negotiation with the clock, not
	// descriptor data -- the descriptor walk must never invent a count.
	CHECK_EQ(t.alts[1].rate_count, 0);
	CHECK_EQ(t.alts[1].rate_min, 0);
	CHECK_EQ(t.alts[1].rate_max, 0);

	// alt 2: same 8 channels, 16-in-2 (subslot 2, resolution 16).
	CHECK_EQ(t.alts[2].alternate_setting, 2);
	CHECK_EQ(t.alts[2].channels, 8);
	CHECK_EQ(t.alts[2].subframe_size, 2);
	CHECK_EQ(t.alts[2].bit_resolution, 16);
	CHECK_EQ(t.alts[2].endpoint_address, 0x01);
	CHECK_EQ(t.alts[2].max_packet_size, 400);
	CHECK_EQ(t.alts[2].feedback_endpoint, 0x82);
	CHECK_EQ(t.alts[2].feedback_max_packet, 4);
	CHECK_EQ(t.alts[2].rate_count, 0);
}

// The capture interface, which this parser dropped on the floor until
// 2026-08-05. Ground truth from the descriptor walk in the landmark list.
static void test_collects_input_alt_settings(void)
{
	UAC1Topology t;
	CHECK(uac2_parse_config(fixture, fixture_len, &t));
	CHECK_EQ(t.in_alt_count, 2);           // alts 0, 1 on interface 2

	CHECK_EQ(t.in_alts[0].alternate_setting, 0);
	CHECK_EQ(t.in_alts[0].endpoint_address, 0);   // zero-bandwidth

	CHECK_EQ(t.in_alts[1].alternate_setting, 1);
	CHECK_EQ(t.in_alts[1].channels, 8);
	CHECK_EQ(t.in_alts[1].subframe_size, 4);
	CHECK_EQ(t.in_alts[1].bit_resolution, 24);
	CHECK_EQ(t.in_alts[1].endpoint_address, 0x81);
	CHECK_EQ(t.in_alts[1].endpoint_address & 0x80, 0x80);   // IN bit set
	CHECK_EQ(t.in_alts[1].max_packet_size, 800);
	// bmAttributes 0x05: isochronous (bits 1:0 = 01), ASYNCHRONOUS
	// (bits 3:2 = 01), data (bits 5:4 = 00). The device free-runs its own
	// converter and sends what it produces -- the host cannot pace it, which
	// is the whole clock-ownership problem the input direction introduces.
	CHECK_EQ(t.in_alts[1].endpoint_attributes, 0x05);
	// An input stream has no feedback endpoint: the device is the source
	// and has nothing to report about a rate it sets itself. If this ever
	// reads non-zero, an audio endpoint has been mistaken for feedback.
	CHECK_EQ(t.in_alts[1].feedback_endpoint, 0);
	CHECK_EQ(t.in_alts[1].rate_count, 0);   // UAC2 rates are runtime, as above
}

// The input path's terminal link is 0x16 -- an OUTPUT_TERMINAL (USB
// streaming), because for a capture stream the USB side is the sink. Its
// bCSourceID sits one byte further along than an INPUT_TERMINAL's, so a
// parser that recorded only input terminals resolves nothing here.
//
// Both directions landing on 0x29 is a fact about THIS device, not about the
// class: the MC200 runs capture and playback off one clock selector. That is
// what will let Stage C size both streams from one rate estimate -- and it is
// recorded as a measurement rather than assumed, because a device with two
// clocks would need the input rate set on its own entity.
static void test_resolves_input_clock_through_output_terminal(void)
{
	UAC1Topology t;
	CHECK(uac2_parse_config(fixture, fixture_len, &t));
	CHECK_EQ(t.in_clock_source_id, 0x29);
	CHECK_EQ(t.in_clock_source_id, t.clock_source_id);
}

static void test_finds_input_alt_by_format(void)
{
	UAC1Topology t;
	CHECK(uac2_parse_config(fixture, fixture_len, &t));
	CHECK_EQ(uac2_find_in_alt(&t, 8, 24), 1);
	CHECK_EQ(uac2_find_in_alt(&t, 8, 16), -1);   // capture offers 24-bit only
	CHECK_EQ(uac2_find_in_alt(&t, 2, 24), -1);   // and 8 channels only
	CHECK_EQ(uac2_find_in_alt(0, 8, 24), -1);

	// Not the same as the output search: the witness's playback interface
	// offers 16-bit and its capture interface does not, so a direction flag
	// defaulted the wrong way would silently select a format the device
	// never advertised on that interface.
	CHECK_EQ(uac2_find_alt(&t, 8, 16), 2);
	CHECK_EQ(uac2_find_alt(&t, 8, 24), 1);
}

// An output-only UAC2 device must report no input rather than mis-assigning
// its feedback endpoint's interface. Built by truncating the fixture just
// before AS interface 2's descriptor, which the landmark list places at the
// interface descriptor whose bInterfaceNumber is 2.
static void test_output_only_device_reports_no_input(void)
{
	size_t cut = 0;
	for (size_t i = 0; i + 1 < fixture_len && fixture[i] >= 2; i += fixture[i]) {
		if (fixture[i + 1] == 0x04 && fixture[i] >= 9 && fixture[i + 2] == 2) {
			cut = i;
			break;
		}
	}
	CHECK(cut > 0);

	UAC1Topology t;
	CHECK(uac2_parse_config(fixture, cut, &t));
	CHECK_EQ(t.streaming_interface, 1);
	CHECK_EQ(t.input_streaming_interface, 0xFF);
	CHECK_EQ(t.in_alt_count, 0);
	CHECK_EQ(t.in_clock_source_id, 0);
	CHECK_EQ(uac2_find_in_alt(&t, 8, 24), -1);
	// The output path is untouched by the input work -- including the
	// feedback endpoint on the very interface that also has an IN endpoint.
	CHECK_EQ(t.alt_count, 3);
	CHECK_EQ(t.alts[1].feedback_endpoint, 0x82);
	CHECK_EQ(t.clock_source_id, 0x29);
}

static void test_finds_alt_by_format(void)
{
	UAC1Topology t;
	CHECK(uac2_parse_config(fixture, fixture_len, &t));
	CHECK_EQ(uac2_find_alt(&t, 8, 24), 1);
	CHECK_EQ(uac2_find_alt(&t, 8, 16), 2);
	CHECK_EQ(uac2_find_alt(&t, 2, 24), -1);    // unsupported channel count
	CHECK_EQ(uac2_find_alt(&t, 8, 32), -1);    // unsupported resolution
	CHECK_EQ(uac2_find_alt(&t, 1, 16), -1);    // unsupported channel count
	CHECK_EQ(uac2_find_alt(0,  8, 24), -1);
}

// claim() at type 0 hands the parser enumbuf+9 (config header already
// skipped) -- which is exactly the form this fixture is already in. The
// header doc comment also promises the OTHER form works (starting at an
// actual CONFIGURATION descriptor): a real config descriptor set always has
// one prepended, so build that shape once and confirm the answer does not
// change. The 9-byte CONFIGURATION descriptor does not match DT_INTERFACE /
// DT_CS_INTERFACE / DT_ENDPOINT, so it is skipped by the same generic
// fallthrough that skips the IAD.
static void test_parses_with_config_header_prefix(void)
{
	static uint8_t buf[16 + sizeof(fixture)];
	buf[0] = 9;                                   // bLength
	buf[1] = 0x02;                                // CONFIGURATION
	uint16_t total = (uint16_t)(9 + fixture_len);
	buf[2] = (uint8_t)total; buf[3] = (uint8_t)(total >> 8);  // wTotalLength
	buf[4] = 3;                                   // bNumInterfaces
	buf[5] = 1; buf[6] = 0;                       // bConfigurationValue, iConfiguration
	buf[7] = 0x80;                                // bmAttributes
	buf[8] = 50;                                  // bMaxPower
	memcpy(buf + 9, fixture, fixture_len);

	UAC1Topology with_header, without_header;
	CHECK(uac2_parse_config(buf, 9 + fixture_len, &with_header));
	CHECK(uac2_parse_config(fixture, fixture_len, &without_header));
	CHECK_EQ(with_header.bcd_adc, without_header.bcd_adc);
	CHECK_EQ(with_header.control_interface, without_header.control_interface);
	CHECK_EQ(with_header.streaming_interface, without_header.streaming_interface);
	CHECK_EQ(with_header.clock_source_id, without_header.clock_source_id);
	CHECK_EQ(with_header.alt_count, without_header.alt_count);
	CHECK_EQ(uac2_find_alt(&with_header, 8, 24), 1);
}

// Truncating the real fixture at every possible length must never crash or
// read out of bounds -- verified for real by running this suite under
// ASan/UBSan during review, which is what the survived-count assertion
// stands in for here. Measured (not guessed) with a sweep of this exact
// fixture: every length in [0, 230] returns false and every length in
// [231, 372] returns true, with no other flips in between. 231 is exactly
// where the byte-224 ENDPOINT descriptor (the alt 1 iso OUT data endpoint)
// finishes -- the find_output_streaming_interface pre-pass will not name a
// streaming interface until it has seen a *complete* iso OUT endpoint, so
// everything before that point fails closed regardless of how much of the
// AC header and clock chain (which fully resolve by byte 59) has already
// been read. This is a property of this fixture's specific layout, not a
// general guarantee -- another descriptor ordering could satisfy the gates
// in a different sequence -- so the two boundary checks below pin the
// measured transition itself rather than the reason, which is what would
// catch a regression.
static void test_survives_truncation(void)
{
	UAC1Topology t;
	size_t survived = 0;
	for (size_t n = 0; n <= fixture_len; n++) {
		uac2_parse_config(fixture, n, &t);
		survived++;
	}
	CHECK_EQ(survived, fixture_len + 1);

	// A handful of specific cut points that must deterministically fail:
	// before the AC HEADER has been read at all, no bcdADC is known.
	CHECK(!uac2_parse_config(fixture, 17, &t));
	// Right after the AC interface descriptor alone (no CS_INTERFACE yet).
	CHECK(!uac2_parse_config(fixture, 9, &t));
	// Cut before any AS interface exists at all: alt_count stays 0.
	CHECK(!uac2_parse_config(fixture, 183, &t));
	// The measured transition: one byte short of the alt 1 OUT endpoint's
	// last byte must still fail closed; including it must succeed.
	CHECK(!uac2_parse_config(fixture, 230, &t));
	CHECK(uac2_parse_config(fixture, 231, &t));
}

static void test_feedback_mps_high_byte(void)
{
	// The real fixture's feedback endpoints both advertise wMaxPacketSize 4
	// (0x0004), which has a zero high byte. This lets a mutant that drops
	// the `| ((uint16_t)b[5] << 8)` term still pass the baseline tests,
	// because (4 | 0) == 4. Patch the wMaxPacketSize to a nonzero high-byte
	// value and re-parse to catch byte-drop mutants.
	static uint8_t buf[sizeof(fixture)];
	memcpy(buf, fixture, fixture_len);

	// Find every 7-byte feedback-EP descriptor matching the pattern
	// {0x07, 0x05, 0x82, 0x11, 0x04, 0x00, 0x04} and patch the wMaxPacketSize
	// bytes (offsets +4, +5) to 0x04, 0x03 -- i.e. 0x0304 = 772.
	const uint8_t needle[] = {0x07, 0x05, 0x82, 0x11, 0x04, 0x00, 0x04};
	size_t needle_len = sizeof(needle);
	size_t occurrence_count = 0;
	for (size_t i = 0; i + needle_len <= fixture_len; i++) {
		if (memcmp(buf + i, needle, needle_len) == 0) {
			occurrence_count++;
			buf[i + 4] = 0x04;
			buf[i + 5] = 0x03;
		}
	}
	CHECK_EQ(occurrence_count, 2);  // both alts on the streaming interface

	// Re-parse the patched buffer and verify both alts now report the
	// nonzero high-byte value (772 = 0x0304), not the mutant's dropped value.
	UAC1Topology t;
	CHECK(uac2_parse_config(buf, fixture_len, &t));
	CHECK_EQ(t.alts[1].feedback_endpoint, 0x82);
	CHECK_EQ(t.alts[1].feedback_max_packet, 772);
	CHECK_EQ(t.alts[2].feedback_endpoint, 0x82);
	CHECK_EQ(t.alts[2].feedback_max_packet, 772);
}

// --- P2: rejection taxonomy ------------------------------------------------
//
// Built by MUTATING the real captured descriptor set rather than hand-rolling
// byte arrays. A hand-built set proves the parser rejects something the parser
// author invented; a one-field mutation of a device that genuinely works
// proves the rejection fires on the field under test and nothing else, because
// every other byte is known-good and known-accepted.
static uint8_t mut[4096];
static size_t  mut_len;

static void mutate_reset(void)
{
	memcpy(mut, fixture, fixture_len);
	mut_len = fixture_len;
}

// Offset of the first CS_INTERFACE (0x24) descriptor carrying `subtype`, or
// -1. Located by walking, not by a baked-in constant, so re-capturing the
// fixture cannot silently point these tests at the wrong byte.
static long find_cs_iface(uint8_t subtype)
{
	size_t i = 0;
	while (i + 1 < mut_len && mut[i] >= 2 && i + mut[i] <= mut_len) {
		if (mut[i + 1] == 0x24 && mut[i] >= 3 && mut[i + 2] == subtype)
			return (long)i;
		i += mut[i];
	}
	return -1;
}

static void test_rejection_taxonomy(void)
{
	UAC1Topology t;

	// The unmutated fixture is the control: if this ever stops being OK,
	// every rejection below is measuring the wrong thing.
	mutate_reset();
	CHECK_EQ(uac2_parse_config_ex(mut, mut_len, &t), UAC2_PARSE_OK);

	// Bad arguments, each of the three ways.
	CHECK_EQ(uac2_parse_config_ex(0, fixture_len, &t), UAC2_REJECT_BAD_ARGS);
	CHECK_EQ(uac2_parse_config_ex(fixture, fixture_len, 0), UAC2_REJECT_BAD_ARGS);
	CHECK_EQ(uac2_parse_config_ex(fixture, 4, &t), UAC2_REJECT_BAD_ARGS);

	// No AS interface owning an iso OUT endpoint: junk that is long enough
	// to pass the length gate but contains no audio function at all.
	uint8_t junk[64];
	memset(junk, 0x11, sizeof(junk));
	junk[0] = 9; junk[1] = 0x04;   // an interface descriptor of some other class
	CHECK_EQ(uac2_parse_config_ex(junk, sizeof(junk), &t),
	         UAC2_REJECT_NO_OUT_STREAM);

	// AC HEADER present but not UAC2. bcdADC sits at b[3..4] of the header.
	mutate_reset();
	long h = find_cs_iface(0x01);        // AC_HEADER
	CHECK(h >= 0);
	mut[h + 3] = 0x00; mut[h + 4] = 0x01;             // bcdADC 0x0100 = UAC1
	CHECK_EQ(uac2_parse_config_ex(mut, mut_len, &t), UAC2_REJECT_NOT_UAC2);

	// No AC HEADER at all -- a different verdict from "declares UAC1", which
	// is the split this taxonomy exists to make. Renumber the subtype so the
	// descriptor is still well formed and still walked over.
	mutate_reset();
	h = find_cs_iface(0x01);
	CHECK(h >= 0);
	mut[h + 2] = 0x7F;                                // not a subtype we know
	CHECK_EQ(uac2_parse_config_ex(mut, mut_len, &t), UAC2_REJECT_NO_AC_HEADER);

	// No AS_GENERAL naming a terminal to clock.
	mutate_reset();
	long g = find_cs_iface(0x01);        // AS_GENERAL shares subtype 0x01...
	// ...so find it on the STREAMING side: the last 0x24/0x01 in the set is
	// an AS_GENERAL, the first is the AC HEADER. Walk to the last.
	{
		size_t i = 0; long last = -1;
		while (i + 1 < mut_len && mut[i] >= 2 && i + mut[i] <= mut_len) {
			if (mut[i + 1] == 0x24 && mut[i] >= 3 && mut[i + 2] == 0x01)
				last = (long)i;
			i += mut[i];
		}
		g = last;
	}
	CHECK(g > h);                        // sanity: streaming side is later
	{
		// Blank every AS_GENERAL, not just the last: any one of them would
		// otherwise supply the terminal link.
		size_t i = 0;
		while (i + 1 < mut_len && mut[i] >= 2 && i + mut[i] <= mut_len) {
			if (mut[i + 1] == 0x24 && mut[i] >= 3 && mut[i + 2] == 0x01
			    && (long)i != h)
				mut[i + 2] = 0x7E;
			i += mut[i];
		}
	}
	CHECK_EQ(uac2_parse_config_ex(mut, mut_len, &t),
	         UAC2_REJECT_NO_TERMINAL_LINK);

	// Clock chain that leads nowhere: keep the terminal link, break the
	// CLOCK_SOURCE's own entity id so nothing the chain reaches is a source.
	mutate_reset();
	long cs = find_cs_iface(0x0A);       // AC_CLOCK_SOURCE
	CHECK(cs >= 0);
	mut[cs + 3] = 0x77;                  // an id nothing points at
	CHECK_EQ(uac2_parse_config_ex(mut, mut_len, &t),
	         UAC2_REJECT_CLOCK_UNRESOLVED);

	// Every reason has a name, and none of them is "unknown".
	for (int r = UAC2_PARSE_OK; r <= UAC2_REJECT_CLOCK_UNRESOLVED; r++) {
		const char *s = uac2_parse_result_str((uac2_parse_result)r);
		CHECK(s != 0);
		CHECK(strcmp(s, "unknown") != 0);
	}

	// The bool wrapper still agrees with the detailed result, which is what
	// lets every existing caller stay as it is.
	mutate_reset();
	CHECK(uac2_parse_config(mut, mut_len, &t));
	mut[cs = find_cs_iface(0x0A) + 3] = 0x77;
	CHECK(!uac2_parse_config(mut, mut_len, &t));
}

// --- P2: clock multiplier in the chain -------------------------------------
//
// Rather than invent a device, this rewrites the fixture's existing clock
// SELECTOR as a clock MULTIPLIER and requires the SAME clock to come out. The
// two descriptors differ by exactly one thing that matters here: a selector
// carries bNrInPins at b[4] and its first source at b[5], a multiplier carries
// bCSourceID at b[4] with no pin count. So moving one byte converts the
// topology's expression without changing the topology, and the resolved clock
// id is the control -- if the multiplier hop were silently dropped the chain
// would fail closed and the parse would be rejected outright.
static void test_resolves_clock_through_multiplier(void)
{
	UAC1Topology before, after;

	mutate_reset();
	CHECK_EQ(uac2_parse_config_ex(mut, mut_len, &before), UAC2_PARSE_OK);

	long sel = find_cs_iface(0x0B);          // AC_CLOCK_SELECTOR
	CHECK(sel >= 0);
	CHECK(mut[sel] >= 6);
	CHECK_EQ(mut[sel + 4], 1);               // single input, as the parser needs
	uint8_t upstream = mut[sel + 5];

	mut[sel + 2] = 0x0C;                     // subtype -> CLOCK_MULTIPLIER
	mut[sel + 4] = upstream;                 // bCSourceID moves to b[4]

	CHECK_EQ(uac2_parse_config_ex(mut, mut_len, &after), UAC2_PARSE_OK);
	CHECK_EQ(after.clock_source_id, before.clock_source_id);
	CHECK_EQ(after.clock_source_id, 0x29);   // the MC200's, from P1

	// And the negative: a multiplier whose upstream is nobody must still fail
	// closed rather than resolve to something arbitrary.
	mut[sel + 4] = 0x77;
	UAC1Topology broken;
	CHECK_EQ(uac2_parse_config_ex(mut, mut_len, &broken),
	         UAC2_REJECT_CLOCK_UNRESOLVED);
}

// --- P2: composite device, two audio functions -----------------------------
//
// PREPENDS a capture-only audio function to the real descriptor set. That
// ordering is the point: the decoy is declared first and owns no isochronous
// OUT endpoint, so a parser that identifies interfaces by class alone would
// assemble the decoy's AudioControl interface together with the real
// function's AudioStreaming interface -- a topology describing no device that
// exists, with the clock chain resolved across the seam. And a parser that
// scoped to the FIRST audio function would reject a device it can drive
// perfectly well.
static const uint8_t decoy_function[] = {
	// IAD: audio function, interfaces 0x10..0x11.
	// bFunctionSubClass is 0x00 (FUNCTION_SUBCLASS_UNDEFINED, UAC2 4.6) --
	// what the real MC200 IAD carries. This decoy originally said 0x01, which
	// is the intuitive-but-wrong value, and the mismatch against the captured
	// fixture is what exposed the same wrong assumption in the parser.
	8, 0x0B, 0x10, 2, 0x01, 0x00, 0x20, 0,
	// AudioControl interface 0x10
	9, 0x04, 0x10, 0, 0, 0x01, 0x01, 0x20, 0,
	//   AC HEADER, bcdADC 0x0200 -- a well-formed UAC2 function
	9, 0x24, 0x01, 0x00, 0x02, 0x00, 0, 0, 0,
	// AudioStreaming interface 0x11 alt 1
	9, 0x04, 0x11, 1, 1, 0x01, 0x02, 0x20, 0,
	//   AS_GENERAL, 2 channels
	16, 0x24, 0x01, 0x99, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0,
	//   isochronous IN endpoint ONLY: this function captures, it cannot play
	7, 0x05, 0x83, 0x05, 0x00, 0x01, 1,
};

static void test_composite_two_audio_functions(void)
{
	UAC1Topology alone, composite, decoy_only;

	mutate_reset();
	CHECK_EQ(uac2_parse_config_ex(mut, mut_len, &alone), UAC2_PARSE_OK);

	// The decoy on its own is exactly the rejection the loop must step over.
	CHECK_EQ(uac2_parse_config_ex(decoy_function, sizeof(decoy_function),
	                              &decoy_only), UAC2_REJECT_NO_OUT_STREAM);

	// decoy first, real function second
	memcpy(mut, decoy_function, sizeof(decoy_function));
	memcpy(mut + sizeof(decoy_function), fixture, fixture_len);
	mut_len = sizeof(decoy_function) + fixture_len;

	CHECK_EQ(uac2_parse_config_ex(mut, mut_len, &composite), UAC2_PARSE_OK);

	// Everything must come from the REAL function, not a blend of the two.
	CHECK_EQ(composite.clock_source_id,     alone.clock_source_id);
	CHECK_EQ(composite.clock_source_id,     0x29);
	CHECK_EQ(composite.alt_count,           alone.alt_count);
	CHECK_EQ(composite.streaming_interface, alone.streaming_interface);
	CHECK_EQ(composite.control_interface,   alone.control_interface);
	CHECK_EQ(composite.in_alt_count,        alone.in_alt_count);
	CHECK_EQ(composite.input_streaming_interface,
	         alone.input_streaming_interface);
	// The decoy's interface numbers must appear nowhere.
	CHECK(composite.control_interface   != 0x10);
	CHECK(composite.streaming_interface != 0x11);
	// ...and its 2-channel AS_GENERAL must not have become an alt of ours.
	for (uint8_t i = 0; i < composite.alt_count; i++)
		CHECK_EQ(composite.alts[i].channels, alone.alts[i].channels);
}

// --- P2: CS_SAM_FREQ_CONTROL RANGE ----------------------------------------
static void put16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void put32(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8);
	p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}

static void test_range_setup(void)
{
	uint8_t s[8];
	uac2_clock_range_setup(s, 0x00, 0x29, 26);
	CHECK_EQ(s[0], 0xA1);                  // class, interface, DEVICE-to-host
	CHECK_EQ(s[1], 0x02);                  // RANGE, not CUR
	CHECK_EQ(s[2], 0x00); CHECK_EQ(s[3], 0x01);   // CS_SAM_FREQ << 8
	CHECK_EQ(s[4], 0x00); CHECK_EQ(s[5], 0x29);   // (clockID<<8) | interface
	CHECK_EQ(s[6], 26);   CHECK_EQ(s[7], 0);      // wLength
	// The direction bit is the one that separates this from the CUR write --
	// get it wrong and the device is asked to RECEIVE its own capabilities.
	uint8_t cur[8];
	uac2_clock_cur_setup(cur, 0x00, 0x29);
	CHECK(cur[0] != s[0]);
	// A two-byte wLength must survive the split.
	uac2_clock_range_setup(s, 0x01, 0x05, 0x0102);
	CHECK_EQ(s[6], 0x02); CHECK_EQ(s[7], 0x01);
}

static void test_range_parse(void)
{
	// Two discrete rates, the ordinary shape: MIN == MAX, RES 0.
	uint8_t r[2 + 12 * 2];
	put16(r, 2);
	put32(r + 2,  44100); put32(r + 6,  44100); put32(r + 10, 0);
	put32(r + 14, 48000); put32(r + 18, 48000); put32(r + 22, 0);

	CHECK_EQ(uac2_range_count(r, sizeof(r)), 2);
	uint32_t lo = 0, hi = 0, st = 9;
	CHECK(uac2_range_get(r, sizeof(r), 0, &lo, &hi, &st));
	CHECK_EQ(lo, 44100); CHECK_EQ(hi, 44100); CHECK_EQ(st, 0);
	CHECK(uac2_range_get(r, sizeof(r), 1, &lo, &hi, &st));
	CHECK_EQ(lo, 48000); CHECK_EQ(hi, 48000);
	CHECK(!uac2_range_get(r, sizeof(r), 2, &lo, &hi, &st));   // past the end
	CHECK(uac2_range_get(r, sizeof(r), 0, 0, 0, 0));          // all-null is fine

	CHECK(uac2_range_supports(r, sizeof(r), 44100));
	CHECK(uac2_range_supports(r, sizeof(r), 48000));
	CHECK(!uac2_range_supports(r, sizeof(r), 96000));
	CHECK(!uac2_range_supports(r, sizeof(r), 44099));

	// A stepped subrange: 8000..48000 in steps of 4000.
	uint8_t s[2 + 12];
	put16(s, 1);
	put32(s + 2, 8000); put32(s + 6, 48000); put32(s + 10, 4000);
	CHECK(uac2_range_supports(s, sizeof(s), 8000));
	CHECK(uac2_range_supports(s, sizeof(s), 48000));
	CHECK(uac2_range_supports(s, sizeof(s), 12000));
	CHECK(!uac2_range_supports(s, sizeof(s), 44100));   // in span, off the step
	CHECK(!uac2_range_supports(s, sizeof(s), 52000));   // past MAX

	// A continuous subrange: RES 0 with MIN != MAX accepts anything inside.
	uint8_t c[2 + 12];
	put16(c, 1);
	put32(c + 2, 8000); put32(c + 6, 96000); put32(c + 10, 0);
	CHECK(uac2_range_supports(c, sizeof(c), 44100));
	CHECK(uac2_range_supports(c, sizeof(c), 8000));
	CHECK(!uac2_range_supports(c, sizeof(c), 7999));

	// Malformed and truncated replies must not read past the buffer or
	// invent subranges. A device declaring six and sending two is the case
	// our own short wLength would produce.
	CHECK_EQ(uac2_range_count(0, 10), 0);
	CHECK_EQ(uac2_range_count(r, 0), 0);
	CHECK_EQ(uac2_range_count(r, 1), 0);
	CHECK_EQ(uac2_range_count(r, 2), 0);      // header only, no subranges fit
	CHECK_EQ(uac2_range_count(r, 13), 0);     // one byte short of a subrange
	CHECK_EQ(uac2_range_count(r, 14), 1);     // exactly one fits
	uint8_t liar[2 + 12];
	memcpy(liar, r, sizeof(liar));
	put16(liar, 6);                            // claims six, carries one
	CHECK_EQ(uac2_range_count(liar, sizeof(liar)), 1);
	CHECK(uac2_range_supports(liar, sizeof(liar), 44100));
	CHECK(!uac2_range_supports(liar, sizeof(liar), 48000));  // never arrived

	// MIN > MAX is nonsense and must be skipped, not trusted.
	uint8_t bad[2 + 12];
	put16(bad, 1);
	put32(bad + 2, 48000); put32(bad + 6, 44100); put32(bad + 10, 0);
	CHECK(!uac2_range_supports(bad, sizeof(bad), 44100));
	CHECK(!uac2_range_supports(bad, sizeof(bad), 48000));

	// Zero subranges: a device that supports nothing supports nothing.
	uint8_t none[2];
	put16(none, 0);
	CHECK_EQ(uac2_range_count(none, sizeof(none)), 0);
	CHECK(!uac2_range_supports(none, sizeof(none), 44100));
}

// --- P2: the fixture corpus ------------------------------------------------
//
// P2's gate is "host suite green over the corpus". Every captured descriptor
// set goes through the UAC2 parser, including the five UAC1 ones, because what
// they prove is that the taxonomy answers INFORMATIVELY on real devices: each
// gets all the way to the version check and is declined as not-uac2, rather
// than falling out earlier with something vague. Before the taxonomy every one
// of these was an indistinguishable `false`.
static uint8_t corpus_buf[4096];

static size_t load_named(const char *path)
{
	FILE *f = fopen(path, "rb");
	if (!f) {
		// A renamed or missing fixture must FAIL, never silently skip --
		// a corpus that quietly shrinks to nothing is green and worthless.
		printf("FAIL cannot open corpus fixture %s: %s\n", path, strerror(errno));
		failures++; checks++;
		return 0;
	}
	size_t n = fread(corpus_buf, 1, sizeof(corpus_buf), f);
	fclose(f);
	return n;
}

static void test_corpus_through_uac2_parser(void)
{
	static const struct { const char *file; int want; } cases[] = {
		{ "fixtures/dongle_uac1_duplex.bin",          UAC2_REJECT_NOT_UAC2 },
		{ "fixtures/generalplus_uac1_multirate.bin",  UAC2_REJECT_NOT_UAC2 },
		{ "fixtures/headset_uac1_config.bin",         UAC2_REJECT_NOT_UAC2 },
		{ "fixtures/jabra_uac1_multirate.bin",        UAC2_REJECT_NOT_UAC2 },
		{ "fixtures/xmos_uac1_async_feedback.bin",    UAC2_REJECT_NOT_UAC2 },
		{ "fixtures/xmos_uac2_2ami8o8.bin",           UAC2_PARSE_OK        },
	};
	for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
		size_t n = load_named(cases[i].file);
		if (n == 0) continue;
		CHECK(n < sizeof(corpus_buf));       // not silently truncated
		UAC1Topology t;
		CHECK_EQ(uac2_parse_config_ex(corpus_buf, n, &t), cases[i].want);
	}

	// The UAC2 fixture must still yield EXACTLY what P1's minimal parser
	// produced, and what silicon confirmed. This is the regression half of
	// P2's gate: taxonomy, multiplier support, IAD scoping and function
	// iteration all landed underneath it, and none of them may move a field.
	size_t n = load_named("fixtures/xmos_uac2_2ami8o8.bin");
	CHECK(n > 0);
	UAC1Topology t;
	CHECK_EQ(uac2_parse_config_ex(corpus_buf, n, &t), UAC2_PARSE_OK);
	CHECK_EQ(t.clock_source_id,           0x29);   // silicon printed clock=41
	CHECK_EQ(t.control_interface,         0);
	CHECK_EQ(t.streaming_interface,       1);
	CHECK_EQ(t.input_streaming_interface, 2);
	CHECK_EQ(t.alt_count,                 3);
	CHECK_EQ(t.in_alt_count,              2);
	CHECK_EQ(t.bcd_adc,                   0x0200);
}

int main(void)
{
	load_fixture();
	test_corpus_through_uac2_parser();
	test_range_setup();
	test_range_parse();
	test_rejection_taxonomy();
	test_resolves_clock_through_multiplier();
	test_composite_two_audio_functions();
	test_clock_cur_setup();
	test_fixture_is_the_captured_descriptor_set();
	test_rejects_garbage();
	test_identifies_interfaces();
	test_resolves_clock_source_through_selector();
	test_collects_alt_settings();
	test_collects_input_alt_settings();
	test_resolves_input_clock_through_output_terminal();
	test_finds_input_alt_by_format();
	test_output_only_device_reports_no_input();
	test_finds_alt_by_format();
	test_parses_with_config_header_prefix();
	test_survives_truncation();
	test_feedback_mps_high_byte();
	if (failures == 0) { printf("test_uac2_parse: all %d checks passed\n", checks); return 0; }
	printf("test_uac2_parse: %d/%d FAILED\n", failures, checks);
	return 1;
}
