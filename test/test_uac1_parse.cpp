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

int main(void)
{
	load_fixture();
	test_fixture_is_a_config_descriptor();
	test_rejects_garbage();
	test_identifies_interfaces();
	test_collects_alt_settings();
	test_resolves_speaker_feature_unit();
	test_finds_alt_by_format();
	test_parses_without_config_header();
	printf("%d checks, %d failures\n", checks, failures);
	return failures ? 1 : 0;
}
