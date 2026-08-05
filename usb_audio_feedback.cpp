// Copyright (c) 2026 Nicholas Newdigate
// SPDX-License-Identifier: MIT
#include "usb_audio_feedback.h"

uint32_t uac1_feedback_to_mhz(const uint8_t fb[3])
{
	if (!fb) return 0;

	// 3 bytes little-endian, samples per frame in 10.14 fixed point.
	uint32_t q14 = (uint32_t)fb[0] | ((uint32_t)fb[1] << 8) | ((uint32_t)fb[2] << 16);

	// samples/frame * 1000 frames/s * 1000 (to mHz) = q14 * 1e6 >> 14.
	// 64-bit intermediate: 0xFFFFFF * 1e6 needs 44 bits.
	return (uint32_t)(((uint64_t)q14 * 1000000u) >> 14);
}

uint32_t uac2_feedback_to_mhz(const uint8_t fb[4])
{
	if (!fb) return 0;
	uint32_t v = (uint32_t)fb[0] | ((uint32_t)fb[1] << 8)
	           | ((uint32_t)fb[2] << 16) | ((uint32_t)fb[3] << 24);
	// samples/microframe * 8000 microframes/s * 1000 mHz/Hz, then drop
	// the 16 fractional bits. 64-bit: 0xFFFFFFFF * 8e6 needs 55 bits.
	uint64_t mhz = ((uint64_t)v * 8000000u) >> 16;
	if (mhz > 0xFFFFFFFFu) return 0xFFFFFFFFu;
	return (uint32_t)mhz;
}

bool uac1_feedback_plausible(uint32_t fb_mhz, uint32_t nominal_mhz)
{
	if (fb_mhz == 0 || nominal_mhz == 0) return false;

	// +-2% window. 64-bit products: nominal_mhz * 102 approaches 2^32.
	uint64_t lo = ((uint64_t)nominal_mhz * 98u) / 100u;
	uint64_t hi = ((uint64_t)nominal_mhz * 102u) / 100u;
	return fb_mhz >= lo && fb_mhz <= hi;
}

uint32_t uac1_rate_slew(uint32_t current_mhz, uint32_t target_mhz,
                        uint32_t max_step_mhz)
{
	if (target_mhz > current_mhz) {
		uint32_t d = target_mhz - current_mhz;
		return current_mhz + (d < max_step_mhz ? d : max_step_mhz);
	}
	uint32_t d = current_mhz - target_mhz;
	return current_mhz - (d < max_step_mhz ? d : max_step_mhz);
}

uint32_t uac1_fb_average(uint32_t avg_mhz, uint32_t sample_mhz, uint32_t div)
{
	if (avg_mhz == 0) return sample_mhz;
	if (div < 1) div = 1;

	// 1/div EMA with round-half-away-from-zero, in signed arithmetic: the
	// values differ by at most a few hundred ppm so the delta fits easily.
	//
	// div is the caller's poll rate divided by ~7.8: the filter's time
	// constant is div/poll_rate, and holding it at ~128 ms is load-bearing
	// -- chasing the device's report dither with a shorter horizon measured
	// +4.8 ppm on this bench. 8 at 62.5 polls/s and 32 at 250 polls/s are
	// the same 128 ms filter fed 4x more reports, which is where the noise
	// reduction comes from; a caller raising the poll rate WITHOUT scaling
	// div is quietly shortening the horizon.
	int32_t d = (int32_t)sample_mhz - (int32_t)avg_mhz;
	int32_t half = (int32_t)(div / 2);
	int32_t step = (d >= 0) ? (d + half) / (int32_t)div
	                        : (d - half) / (int32_t)div;
	return (uint32_t)((int32_t)avg_mhz + step);
}
