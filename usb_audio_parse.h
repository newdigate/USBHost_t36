// Copyright (c) 2026 Nicholas Newdigate
// SPDX-License-Identifier: MIT
//
// USB Audio Class 1.0 descriptor parsing. Deliberately free of Arduino and
// USBHost_t36 dependencies so it can be unit-tested on the host.
#ifndef USB_AUDIO_PARSE_H_
#define USB_AUDIO_PARSE_H_
#include <stdint.h>
#include <stddef.h>

#define UAC1_MAX_ALTS 16

// Discrete sample rates recorded per alternate setting. UAC1 allows up to 255;
// real devices list a handful. Anything beyond this is dropped, and rate_count
// saturates -- see uac1_alt_supports_rate().
#define UAC1_MAX_RATES 8

struct UAC1AltSetting {
	uint8_t  alternate_setting;
	uint8_t  endpoint_address;
	uint8_t  endpoint_attributes;
	uint16_t max_packet_size;
	uint8_t  channels;
	uint8_t  subframe_size;
	uint8_t  bit_resolution;

	// Sample rates. UAC1 devices use one of two idioms and both occur in
	// the wild: one rate per alternate setting (Logitech 046D:0A8F lists
	// eight alts), or several rates in one setting (Jabra 0B0E:2301 lists
	// 8000/16000/32000/44100/48000 in a single alt). A parser that only
	// reads tSamFreq[0] sees the second kind as an 8 kHz-only device.
	//
	// rate_count > 0: `rates` holds that many discrete frequencies.
	// rate_count == 0: the device advertises a continuous range, and
	//                  rate_min/rate_max bound it.
	uint8_t  rate_count;
	uint32_t rates[UAC1_MAX_RATES];
	uint32_t rate_min;
	uint32_t rate_max;
	// bmAttributes from the class-specific AS isochronous endpoint descriptor
	// (CS_ENDPOINT / EP_GENERAL). Bit 0 is the sampling frequency control.
	uint8_t  ep_controls;
	// Asynchronous endpoints pair the data endpoint with a feedback endpoint
	// that reports the device's true sample rate. Taken from bSynchAddress in
	// the 9-byte audio endpoint descriptor where present, else from the IN
	// isochronous endpoint on this output interface -- the feedback endpoint's
	// own bmAttributes usage bits read as "data", so they cannot identify it.
	uint8_t  feedback_endpoint;
	uint8_t  feedback_refresh;   // poll period exponent: 2^n ms
};

// True if this alternate setting can carry `rate`, by either idiom. Selecting
// the alt is not sufficient for a multi-rate or continuous setting -- the rate
// must also be set with a SET_CUR sampling-frequency request on the endpoint.
bool uac1_alt_supports_rate(const UAC1AltSetting *alt, uint32_t rate);

// True when the host must issue a class-specific SET_CUR SAMPLING_FREQ request
// to the endpoint after SET_INTERFACE, because the alternate setting alone does
// not determine the rate.
bool uac1_alt_needs_rate_request(const UAC1AltSetting *alt);

struct UAC1Topology {
	uint16_t bcd_adc;
	uint8_t  control_interface;    // 0xFF if none
	uint8_t  streaming_interface;  // 0xFF if none
	uint8_t  feature_unit_id;      // 0 if none
	// UAC2 only: the RESOLVED Clock Source entity (single-input selectors
	// followed through); 0 for UAC1 or when unresolved.
	uint8_t  clock_source_id;
	uint8_t  alt_count;
	// alts[] holds up to UAC1_MAX_ALTS entries. A device advertising more
	// alternate settings (or more feature units, tracked internally during
	// parsing) than that has the excess silently dropped; alt_count == 16
	// cannot be distinguished from "exactly 16" by the caller.
	UAC1AltSetting alts[UAC1_MAX_ALTS];
};

// Parses a full configuration descriptor. Returns false if the descriptor
// contains no audio streaming interface with an isochronous OUT endpoint.
// On failure the contents of *out are unspecified and callers must not
// read it.
bool uac1_parse_config(const uint8_t *desc, size_t len, UAC1Topology *out);

// Returns the alternate setting number matching the format, or -1.
int uac1_find_alt(const UAC1Topology *t, uint32_t rate, uint8_t channels, uint8_t bits);

// Bytes to send in the next 1 ms USB frame for a given sample rate.
//
// Rates that are not a whole number of samples per millisecond need the packet
// size to alternate: 44.1 kHz is 44.1 samples per frame, so 44 samples nine
// times then 45 once, repeating. `accum` carries the fractional remainder
// between calls and must be zeroed when a stream starts.
//
// 48 kHz falls out as exactly 48 samples every frame with the remainder always
// zero, so one code path serves both and the constant-size case needs no
// special handling.
// Same, but the rate is in millihertz so it can be trimmed finer than the
// 22.7 ppm that one whole hertz represents at 44.1 kHz. `accum` is then in
// millihertz units too, and the two functions must not share an accumulator.
uint16_t uac1_frame_bytes_mhz(uint32_t *accum, uint32_t rate_mhz, uint8_t channels,
                              uint8_t bytes_per_sample);

uint16_t uac1_frame_bytes(uint32_t *accum, uint32_t rate, uint8_t channels,
                          uint8_t bytes_per_sample);

// Bytes for the next 125 us microframe at high speed: same fractional
// accumulator as uac1_frame_bytes_mhz but against 8000 microframes/s.
// The accumulator unit is millihertz for both variants; what differs is
// the whole-sample divisor, which is why an accumulator must never be
// shared across variants.
uint16_t uac2_uframe_bytes_mhz(uint32_t *accum, uint32_t rate_mhz, uint8_t channels,
                               uint8_t bytes_per_sample);

// Pack 16-bit interleaved frames into a device subslot layout, with the
// sample LEFT-JUSTIFIED within the subslot as USB Audio Data Formats 2.0
// section 2.3.1 requires: 16-in-2 verbatim little-endian; 24-in-3 emits
// {00, lo, hi}; 24-in-4 emits {00, 00, lo, hi} -- padding always at the
// least-significant end, so a negative sample's sign lives in its own
// high byte and no separate sign handling exists. src carries ch_live
// interleaved channels per frame; device channels beyond ch_live are
// zero-filled. Returns bytes written, 0 for unsupported subslot sizes,
// null pointers, or ch_live > ch_total.
uint32_t uac_pack16(uint8_t *dst, const int16_t *src, uint32_t frames,
                    uint8_t ch_live, uint8_t ch_total, uint8_t subslot);

#endif // USB_AUDIO_PARSE_H_
