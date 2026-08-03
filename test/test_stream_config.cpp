// Copyright (c) 2026 Nicholas Newdigate
// SPDX-License-Identifier: MIT
#include "usb_audio_parse.h"
#include <stdio.h>
#include <string.h>

static int failures = 0, checks = 0;
#define CHECK_EQ(a, b) do { checks++; long _ck_a = (long)(a), _ck_b = (long)(b); \
	if (_ck_a != _ck_b) { printf("FAIL %s:%d: %s == %s (got %ld, want %ld)\n", \
	__FILE__, __LINE__, #a, #b, _ck_a, _ck_b); failures++; } } while (0)

// The witness this bench swaps between: the UAC2 personality's 24-bit alt
// (8 channels of 24-in-4 with a feedback endpoint) and the UAC1 personality's
// stereo alt. Re-flashing the device between the two is the hardware form of
// the mismatch this predicate exists to catch.
static UAC1AltSetting uac2_alt(void)
{
	UAC1AltSetting a;
	memset(&a, 0, sizeof(a));
	a.alternate_setting = 1;
	a.endpoint_address  = 0x01;
	a.channels          = 8;
	a.subframe_size     = 4;
	a.bit_resolution    = 24;
	a.max_packet_size   = 800;
	a.feedback_endpoint = 0x82;
	return a;
}

static UAC1AltSetting uac1_alt(void)
{
	UAC1AltSetting a;
	memset(&a, 0, sizeof(a));
	a.alternate_setting = 1;
	a.endpoint_address  = 0x01;
	a.channels          = 2;
	a.subframe_size     = 2;
	a.bit_resolution    = 16;
	a.max_packet_size   = 200;
	a.feedback_endpoint = 0x82;
	return a;
}

static void test_round_trip(void)
{
	UAC1AltSetting a = uac2_alt();
	UACStreamConfig c;
	uac_stream_config(&c, true, &a);

	CHECK_EQ(c.is_uac2, 1);
	CHECK_EQ(c.alternate_setting, 1);
	CHECK_EQ(c.endpoint_address, 0x01);
	CHECK_EQ(c.channels, 8);
	CHECK_EQ(c.subframe_size, 4);
	CHECK_EQ(c.max_packet_size, 800);
	CHECK_EQ(c.feedback_endpoint, 0x82);

	// The same alt driven as UAC1 differs only in the transport flag, which
	// is the whole point: the descriptor rings are not interchangeable.
	UACStreamConfig fs;
	uac_stream_config(&fs, false, &a);
	CHECK_EQ(fs.is_uac2, 0);
	CHECK_EQ(uac_stream_config_equal(&c, &fs), false);
}

static void test_same_device_matches(void)
{
	// Unplug/replug of one device re-parses to an identical alt, so the
	// armed ring keeps running -- the self-heal contract disconnect()
	// documents depends on this comparing equal.
	UAC1AltSetting first = uac2_alt(), again = uac2_alt();
	UACStreamConfig armed, want;
	uac_stream_config(&armed, true, &first);
	uac_stream_config(&want, true, &again);

	CHECK_EQ(uac_stream_config_equal(&armed, &want), true);
	CHECK_EQ(uac_stream_config_equal(&want, &armed), true);   // symmetric
	CHECK_EQ(uac_stream_config_equal(&armed, &armed), true);  // reflexive
}

static void test_transport_swap_differs(void)
{
	// The wedge this predicate was written for: a stream armed for the HS
	// iTD ring, then a UAC1 device claimed in its place. Every descriptor
	// the ring owns belongs to the wrong transport.
	UAC1AltSetting hs = uac2_alt(), fs = uac1_alt();
	UACStreamConfig armed, want;
	uac_stream_config(&armed, true, &hs);
	uac_stream_config(&want, false, &fs);
	CHECK_EQ(uac_stream_config_equal(&armed, &want), false);

	// ...and the reverse direction, which wedges just as silently.
	uac_stream_config(&armed, false, &fs);
	uac_stream_config(&want, true, &hs);
	CHECK_EQ(uac_stream_config_equal(&armed, &want), false);
}

// Every field must be load-bearing: a comparison that forgets one lets a
// mismatched device through, which is the failure mode being fixed. Vary
// exactly one field per case, leaving the rest identical.
static void test_each_field_is_load_bearing(void)
{
	UAC1AltSetting base = uac2_alt();
	UACStreamConfig armed;
	uac_stream_config(&armed, true, &base);

	UAC1AltSetting v;
	UACStreamConfig want;

	v = base; v.alternate_setting = 2;
	uac_stream_config(&want, true, &v);
	CHECK_EQ(uac_stream_config_equal(&armed, &want), false);

	v = base; v.endpoint_address = 0x03;
	uac_stream_config(&want, true, &v);
	CHECK_EQ(uac_stream_config_equal(&armed, &want), false);

	v = base; v.channels = 2;
	uac_stream_config(&want, true, &v);
	CHECK_EQ(uac_stream_config_equal(&armed, &want), false);

	v = base; v.subframe_size = 2;
	uac_stream_config(&want, true, &v);
	CHECK_EQ(uac_stream_config_equal(&armed, &want), false);

	v = base; v.max_packet_size = 192;
	uac_stream_config(&want, true, &v);
	CHECK_EQ(uac_stream_config_equal(&armed, &want), false);

	// A device that stops advertising feedback must rebuild too: the armed
	// stream has a feedback descriptor linked and polling an endpoint the
	// new device does not serve.
	v = base; v.feedback_endpoint = 0;
	uac_stream_config(&want, true, &v);
	CHECK_EQ(uac_stream_config_equal(&armed, &want), false);

	// The transport flag, varied on its own with one identical alt.
	uac_stream_config(&want, false, &base);
	CHECK_EQ(uac_stream_config_equal(&armed, &want), false);

	// Fields the ring does not care about must NOT force a rebuild:
	// bit_resolution rides along with subframe_size for every format this
	// driver arms, and rate is negotiated, not descriptor layout.
	v = base; v.bit_resolution = 32; v.rate_count = 3; v.rates[0] = 96000;
	uac_stream_config(&want, true, &v);
	CHECK_EQ(uac_stream_config_equal(&armed, &want), true);
}

static void test_null_and_unconfigured(void)
{
	UACStreamConfig c;
	memset(&c, 0xA5, sizeof(c));

	// A null alt is the unconfigured device (findAlt(-1) while detached):
	// it must zero the config rather than leave the caller's stack garbage
	// to be compared against an armed stream.
	uac_stream_config(&c, true, NULL);
	CHECK_EQ(c.is_uac2, 0);
	CHECK_EQ(c.endpoint_address, 0);
	CHECK_EQ(c.channels, 0);
	CHECK_EQ(c.max_packet_size, 0);
	CHECK_EQ(c.feedback_endpoint, 0);

	UAC1AltSetting a = uac2_alt();
	UACStreamConfig armed;
	uac_stream_config(&armed, true, &a);
	CHECK_EQ(uac_stream_config_equal(&armed, &c), false);

	// Null config pointers compare false rather than dereferencing: an
	// armed stream is never torn down on the strength of a bad pointer.
	CHECK_EQ(uac_stream_config_equal(NULL, &armed), false);
	CHECK_EQ(uac_stream_config_equal(&armed, NULL), false);
	CHECK_EQ(uac_stream_config_equal(NULL, NULL), false);

	// A null out pointer must not crash.
	uac_stream_config(NULL, true, &a);
}

int main(void)
{
	test_round_trip();
	test_same_device_matches();
	test_transport_swap_differs();
	test_each_field_is_load_bearing();
	test_null_and_unconfigured();
	if (failures == 0) { printf("test_stream_config: all %d checks passed\n", checks); return 0; }
	printf("test_stream_config: %d/%d FAILED\n", failures, checks);
	return 1;
}
