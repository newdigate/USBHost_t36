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
//   AS interface 2 (alt 0 zero-bandwidth, alt 1 one IN endpoint 0x81) is the
//     capture-only interface: it carries no iso OUT endpoint on any alt, so
//     it is not `streaming_interface` and its alts are not collected --
//     exactly mirroring how uac1_parse_config ignores the microphone AS
//     interface in headset_uac1_config.bin.
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

int main(void)
{
	load_fixture();
	test_clock_cur_setup();
	test_fixture_is_the_captured_descriptor_set();
	test_rejects_garbage();
	test_identifies_interfaces();
	test_resolves_clock_source_through_selector();
	test_collects_alt_settings();
	test_finds_alt_by_format();
	test_parses_with_config_header_prefix();
	test_survives_truncation();
	if (failures == 0) { printf("test_uac2_parse: all %d checks passed\n", checks); return 0; }
	printf("test_uac2_parse: %d/%d FAILED\n", failures, checks);
	return 1;
}
