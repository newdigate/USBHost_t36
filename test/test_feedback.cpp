// Copyright (c) 2026 Nicholas Newdigate
// SPDX-License-Identifier: MIT
//
// UAC1 asynchronous feedback: decoding the 3-byte 10.14 sample-rate report
// and the slew that applies it to the packet-sizing rate.
//
// The feedback value is the device's measurement of its own converter rate,
// in samples per (1 ms) frame, as a 10.14 fixed-point number sent
// little-endian in 3 bytes (USB 2.0 section 5.12.4.2, UAC1 section 3.7.2.2).
// 44.1 kHz is 44.1 samples/frame = 722534.4 in 10.14, so a real device
// reports 722534 or nearby. The decoder output is millihertz because that is
// the unit the packet-sizing accumulator already runs on.
#include "usb_audio_feedback.h"
#include <stdio.h>

static int failures = 0, checks = 0;
#define CHECK_EQ(a, b) do { checks++; long _ck_a = (long)(a), _ck_b = (long)(b); \
	if (_ck_a != _ck_b) { printf("FAIL %s:%d: %s == %s (got %ld, want %ld)\n", \
	__FILE__, __LINE__, #a, #b, _ck_a, _ck_b); failures++; } } while (0)

static uint32_t decode3(uint32_t v)
{
	uint8_t fb[3] = { (uint8_t)(v & 0xFF), (uint8_t)((v >> 8) & 0xFF),
	                  (uint8_t)((v >> 16) & 0xFF) };
	return uac1_feedback_to_mhz(fb);
}

static void test_decode(void)
{
	// 44.1 kHz exactly is 44.1 * 16384 = 722534.4; a device rounds to
	// 722534, which is 44099.976 Hz. Millihertz keeps that resolution:
	// 722534 * 1e6 / 16384 = 44099975.5 -> truncated 44099975.
	CHECK_EQ(decode3(722534), 44099975);

	// 48 kHz exactly: 48 * 16384 = 786432 -> precisely 48000000 mHz.
	CHECK_EQ(decode3(786432), 48000000);

	// One 10.14 LSB above 44.1k: (722535 * 1e6) >> 14 = 44100036.6 -> the
	// decoder resolves single-LSB steps (61 mHz, 1.4 ppm at this rate).
	CHECK_EQ(decode3(722535), 44100036);

	// A device measuring itself 83 ppm slow of 44.1k (the sweep's finding)
	// reports about 722474.4: 722474 -> 44096313 mHz = -83.1 ppm.
	CHECK_EQ(decode3(722474), 44096313);

	// Null input.
	CHECK_EQ(uac1_feedback_to_mhz(0), 0);

	// All-zero payload (seen from devices before their measurement engine
	// has produced a value): decodes to 0, which validity must then reject.
	CHECK_EQ(decode3(0), 0);
}

static void test_validity(void)
{
	// Window is +-2% of the negotiated rate: wide enough for any real
	// crystal (hundreds of ppm), narrow enough to reject a wrong-format or
	// garbage packet (a 16.16 value, a zero, a truncated read).
	CHECK_EQ(uac1_feedback_plausible(44099975, 44100000), true);
	CHECK_EQ(uac1_feedback_plausible(44096313, 44100000), true);
	CHECK_EQ(uac1_feedback_plausible(43218000, 44100000), true);   // -2%
	CHECK_EQ(uac1_feedback_plausible(44982000, 44100000), true);   // +2%
	CHECK_EQ(uac1_feedback_plausible(43217999, 44100000), false);
	CHECK_EQ(uac1_feedback_plausible(44982001, 44100000), false);
	CHECK_EQ(uac1_feedback_plausible(0, 44100000), false);
	// A 10.14 value mistakenly containing a 48k report while negotiated at
	// 44.1k is out of window -- caught, not applied.
	CHECK_EQ(uac1_feedback_plausible(48000000, 44100000), false);
}

static void test_slew(void)
{
	// Already at target: no movement.
	CHECK_EQ(uac1_rate_slew(44100000, 44100000, 4410), 44100000);

	// Far below target: move up by exactly the cap.
	CHECK_EQ(uac1_rate_slew(44100000, 44300000, 4410), 44104410);

	// Far above target: move down by the cap.
	CHECK_EQ(uac1_rate_slew(44100000, 43900000, 4410), 44095590);

	// Within one step: land exactly on target, no overshoot.
	CHECK_EQ(uac1_rate_slew(44100000, 44100100, 4410), 44100100);
	CHECK_EQ(uac1_rate_slew(44100000, 44099900, 4410), 44099900);

	// Zero cap pins the rate (a disabled slew must not drift).
	CHECK_EQ(uac1_rate_slew(44100000, 44200000, 0), 44100000);
}

static void test_average(void)
{
	// Seeding: the first report becomes the average outright, so the servo
	// has a sane target immediately rather than slewing up from zero.
	CHECK_EQ(uac1_fb_average(0, 44095703), 44095703);

	// A device dithers its report between adjacent values to express a rate
	// between them; the EMA must settle near the duty-weighted mean, which
	// a rate-limited follower of the raw reports provably cannot do (it
	// converges to the majority state instead -- measured on this bench as
	// a +4.8 ppm sizing bias). One step moves 1/8 of the distance.
	CHECK_EQ(uac1_fb_average(44095703, 44096314), 44095779);   // +611/8=+76
	CHECK_EQ(uac1_fb_average(44096314, 44095703), 44096238);   // -611/8=-76

	// Under a 50/50 dither between two values the average must settle into
	// a narrow band around the midpoint -- this is the whole point: the
	// raw-report follower settled at a RAIL, which was the +4.8 ppm bias.
	// Symmetric rounding keeps the equilibrium centred instead of walking.
	uint32_t a = 44095703;
	for (int i = 0; i < 40; i++) {
		a = uac1_fb_average(a, 44096314);
		a = uac1_fb_average(a, 44095703);
	}
	const uint32_t mid = (44096314u + 44095703u) / 2u;
	CHECK_EQ(a + 80 > mid && a < mid + 80, true);

	// Small deltas still move: 4 mHz rounds to 1, not 0 (no dead zone
	// wider than the rounding radius).
	CHECK_EQ(uac1_fb_average(44100000, 44100004), 44100001);
	CHECK_EQ(uac1_fb_average(44100004, 44100000), 44100003);

	// Steady input is a fixed point.
	CHECK_EQ(uac1_fb_average(44095703, 44095703), 44095703);
}

int main(void)
{
	test_decode();
	test_validity();
	test_slew();
	test_average();
	if (failures == 0) {
		printf("test_feedback: all %d checks passed\n", checks);
		return 0;
	}
	printf("test_feedback: %d/%d FAILED\n", failures, checks);
	return 1;
}
