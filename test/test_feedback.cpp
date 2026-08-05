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

static uint32_t decode4(uint32_t v)
{
	uint8_t fb[4] = { (uint8_t)(v & 0xFF), (uint8_t)((v >> 8) & 0xFF),
	                  (uint8_t)((v >> 16) & 0xFF), (uint8_t)((v >> 24) & 0xFF) };
	return uac2_feedback_to_mhz(fb);
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
	CHECK_EQ(uac1_fb_average(0, 44095703, 8), 44095703);

	// A device dithers its report between adjacent values to express a rate
	// between them; the EMA must settle near the duty-weighted mean, which
	// a rate-limited follower of the raw reports provably cannot do (it
	// converges to the majority state instead -- measured on this bench as
	// a +4.8 ppm sizing bias). One step moves 1/div of the distance.
	CHECK_EQ(uac1_fb_average(44095703, 44096314, 8), 44095779);   // +611/8=+76
	CHECK_EQ(uac1_fb_average(44096314, 44095703, 8), 44096238);   // -611/8=-76

	// Under a 50/50 dither between two values the average must settle into
	// a narrow band around the midpoint -- this is the whole point: the
	// raw-report follower settled at a RAIL, which was the +4.8 ppm bias.
	// Symmetric rounding keeps the equilibrium centred instead of walking.
	uint32_t a = 44095703;
	for (int i = 0; i < 40; i++) {
		a = uac1_fb_average(a, 44096314, 8);
		a = uac1_fb_average(a, 44095703, 8);
	}
	const uint32_t mid = (44096314u + 44095703u) / 2u;
	CHECK_EQ(a + 80 > mid && a < mid + 80, true);

	// Small deltas still move: 4 mHz rounds to 1, not 0 (no dead zone
	// wider than the rounding radius).
	CHECK_EQ(uac1_fb_average(44100000, 44100004, 8), 44100001);
	CHECK_EQ(uac1_fb_average(44100004, 44100000, 8), 44100003);

	// Steady input is a fixed point.
	CHECK_EQ(uac1_fb_average(44095703, 44095703, 8), 44095703);
}

static void test_average_divisor(void)
{
	// div sets the time constant (div / poll_rate); the HS reader runs
	// 1/32 at 250 polls/s so both configurations hold the same ~128 ms
	// horizon. One step moves 1/32 of the distance.
	CHECK_EQ(uac1_fb_average(44095703, 44096314, 32), 44095722);  // +611/32=+19
	CHECK_EQ(uac1_fb_average(44096314, 44095703, 32), 44096295);  // -611/32=-19

	// The dither test at the HS divisor: same midpoint equilibrium, just
	// reached in smaller steps -- shortening the horizon instead is the
	// +4.8 ppm bug's road back in.
	uint32_t a = 44095703;
	for (int i = 0; i < 160; i++) {
		a = uac1_fb_average(a, 44096314, 32);
		a = uac1_fb_average(a, 44095703, 32);
	}
	const uint32_t mid = (44096314u + 44095703u) / 2u;
	CHECK_EQ(a + 80 > mid && a < mid + 80, true);

	// Symmetric rounding at div 32: half of 32 is 16, so 16 mHz rounds
	// away from zero in both directions.
	CHECK_EQ(uac1_fb_average(44100000, 44100016, 32), 44100001);
	CHECK_EQ(uac1_fb_average(44100016, 44100000, 32), 44100015);

	// Seeding and fixed point behave at any divisor.
	CHECK_EQ(uac1_fb_average(0, 44095703, 32), 44095703);
	CHECK_EQ(uac1_fb_average(44095703, 44095703, 32), 44095703);

	// div 0 clamps to adopt-each-sample rather than dividing by zero.
	CHECK_EQ(uac1_fb_average(44095703, 44096314, 0), 44096314);
}

static void test_decode_hs(void)
{
	// 44.1 kHz is 5.5125 samples/microframe = 361267.2 in 16.16; a device
	// rounds to 361267, which decodes to the same truncated millihertz as
	// the UAC1 vector 722534 (= 2 * 361267): both measure 44.1 kHz.
	CHECK_EQ(decode4(361267), 44099975);

	// 48 kHz exactly: 6.0 * 65536 = 393216 -> precisely 48000000 mHz.
	CHECK_EQ(decode4(393216), 48000000);

	// One 16.16 LSB above 44.1k: resolution is 8e6/65536 = 122 mHz.
	CHECK_EQ(decode4(361268), 44100097);

	// The witness's crystal, measured three independent ways at -83..-86
	// ppm: a device 86 ppm slow of 44.1k reports about 361236.1 ->
	// 361236 -> 44096191 mHz = -86.4 ppm. The hardware gate's Step A
	// expects the live EMA within a few ppm of this value.
	CHECK_EQ(decode4(361236), 44096191);

	// Null input and the all-zero payload devices send before their
	// measurement engine runs: decode to 0, which validity rejects.
	CHECK_EQ(uac2_feedback_to_mhz(0), 0);
	CHECK_EQ(decode4(0), 0);

	// Saturation: unlike 10.14-in-3, a 32-bit 16.16 report can encode
	// rates whose millihertz exceed uint32. Exact boundary: 35184372
	// still fits (4294967285); one LSB more overflows and must clamp to
	// UINT32_MAX -- deep in the plausibility gate's rejection band --
	// not wrap around into a plausible value.
	CHECK_EQ(decode4(35184372), 4294967285u);
	CHECK_EQ(decode4(35184373), 4294967295u);
	CHECK_EQ(decode4(0xFFFFFFFFu), 4294967295u);
	CHECK_EQ(uac1_feedback_plausible(4294967295u, 44100000), false);
}

int main(void)
{
	test_decode();
	test_validity();
	test_slew();
	test_average();
	test_average_divisor();
	test_decode_hs();
	if (failures == 0) {
		printf("test_feedback: all %d checks passed\n", checks);
		return 0;
	}
	printf("test_feedback: %d/%d FAILED\n", failures, checks);
	return 1;
}
