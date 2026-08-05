// Copyright (c) 2026 Nicholas Newdigate
// SPDX-License-Identifier: MIT
//
// UAC1 asynchronous feedback: pure arithmetic, no Arduino or USBHost_t36
// dependencies, so it can be unit-tested on the host (see
// test/test_feedback.cpp). The driver plumbing that reads the endpoint lives
// in usb_audio.cpp; this file is only the numbers.
#ifndef USB_AUDIO_FEEDBACK_H_
#define USB_AUDIO_FEEDBACK_H_
#include <stdint.h>

// Decode a full-speed UAC1 feedback payload: 3 bytes little-endian holding
// the device's measured rate in samples per frame as 10.14 fixed point
// (USB 2.0 section 5.12.4.2). Returns millihertz -- the unit the packet
// sizing accumulator runs on -- truncated, resolution one 10.14 LSB
// (61 mHz). Returns 0 for a null pointer or an all-zero payload; callers
// gate on uac1_feedback_plausible() before applying.
uint32_t uac1_feedback_to_mhz(const uint8_t fb[3]);

// Decode a high-speed UAC2 feedback payload: 4 bytes little-endian holding
// the device's measured rate in samples per MICROFRAME as Q16.16 fixed
// point (USB 2.0 section 5.12.4.2 at high speed). Returns millihertz,
// truncated; resolution one 16.16 LSB (122 mHz at the 8 kHz microframe
// rate). Returns 0 for a null pointer or all-zero payload, and saturates to
// UINT32_MAX when the encoded rate exceeds the millihertz range -- unlike
// the 3-byte 10.14 format, a 32-bit report can overflow uint32 millihertz,
// and garbage must saturate into the plausibility gate's rejection band
// rather than wrap into it. Callers gate on uac1_feedback_plausible()
// before applying.
uint32_t uac2_feedback_to_mhz(const uint8_t fb[4]);

// True when a decoded rate is within +-2% of the negotiated rate. Real
// crystals err by hundreds of ppm at most; anything outside this window is a
// malformed packet (wrong fixed-point format, zero fill, truncated read),
// and applying it would be worse than coasting on the last good value.
bool uac1_feedback_plausible(uint32_t fb_mhz, uint32_t nominal_mhz);

// One slew step of the sizing rate toward the feedback target, moving at
// most max_step_mhz. Feedback arrives as absolute measurements every
// bRefresh interval; stepping rather than jumping keeps a noisy or
// recovering-from-stale reading from producing an audible pitch bend.
uint32_t uac1_rate_slew(uint32_t current_mhz, uint32_t target_mhz,
                        uint32_t max_step_mhz);

// Exponential moving average of feedback reports, 1/div per step. Devices
// dither the report between adjacent values to express rates in between;
// the servo must follow the duty-weighted mean of that dither, and a
// rate-limited follower of the raw reports cannot -- it converges to the
// majority state instead (measured on this bench as a +4.8 ppm sizing bias,
// which is a buffer excursion every ~45 min). Seeding: avg 0 adopts the
// sample outright. Rounding is symmetric so the equilibrium under
// alternating input is the midpoint, not a walk toward one rail.
//
// div sets the time constant: div/poll_rate, which must stay at ~128 ms
// (the dither lesson above). 8 at 62.5 polls/s (FS), 32 at 250 polls/s
// (HS) -- same horizon, 4x more reports averaged into it. div < 1 is
// clamped to 1 (adopt each sample), never a divide by zero.
uint32_t uac1_fb_average(uint32_t avg_mhz, uint32_t sample_mhz, uint32_t div);

#endif // USB_AUDIO_FEEDBACK_H_
