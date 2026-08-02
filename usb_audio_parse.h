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
uint16_t uac1_frame_bytes(uint32_t *accum, uint32_t rate, uint8_t channels,
                          uint8_t bytes_per_sample);

#endif // USB_AUDIO_PARSE_H_
