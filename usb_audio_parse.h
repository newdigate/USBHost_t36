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
	// wMaxPacketSize of the feedback endpoint -- the HS iTD reader must
	// program the endpoint's MPS into bufptr[1]. Captured by the UAC2
	// parser; 0 when no feedback endpoint exists or on the UAC1/FS path,
	// whose siTD reader has no MPS field to program.
	uint16_t feedback_max_packet;
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
	uint8_t  streaming_interface;  // OUT; 0xFF if none
	// The INPUT streaming interface -- the one carrying audio from the
	// device to us -- and its alternate settings, parsed into a parallel
	// array so nothing about the output path changes shape.
	//
	// An input interface is identified as one with an isochronous IN
	// endpoint and NO isochronous OUT endpoint. The absence test is the
	// load-bearing half: an asynchronous OUT interface also carries an IN
	// endpoint (its feedback endpoint), and on the XMOS UAC1 witness that
	// endpoint declares usage type 00 = "data", so it is indistinguishable
	// from audio by its own descriptor. Requiring the interface to have no
	// OUT endpoint excludes it cleanly and without trusting usage bits.
	uint8_t  input_streaming_interface;   // 0xFF if none
	uint8_t  in_alt_count;
	UAC1AltSetting in_alts[UAC1_MAX_ALTS];
	uint8_t  feature_unit_id;      // 0 if none
	// UAC2 only: the RESOLVED Clock Source entity (single-input selectors
	// followed through); 0 for UAC1 or when unresolved.
	uint8_t  clock_source_id;
	// The same, resolved from the INPUT interface's terminal. Recorded
	// separately rather than assumed equal: the MC200 does drive both
	// directions from one clock (entity 41, measured from its descriptors),
	// which is what makes a duplex device's two streams share a rate -- but
	// that is a property of that device, not of the class, and a driver that
	// assumed it would set the wrong entity's rate on a device with two.
	// 0 for UAC1, when there is no input interface, or when the chain does
	// not resolve; unlike the output clock, failing to resolve this does not
	// fail the parse, because input is parsed opportunistically.
	uint8_t  in_clock_source_id;
	uint8_t  alt_count;
	// alts[] holds up to UAC1_MAX_ALTS entries. A device advertising more
	// alternate settings (or more feature units, tracked internally during
	// parsing) than that has the excess silently dropped; alt_count == 16
	// cannot be distinguished from "exactly 16" by the caller.
	UAC1AltSetting alts[UAC1_MAX_ALTS];
};

// Which interface carries audio in each direction. Shared by both class
// versions -- the fields read (bInterfaceNumber, bInterfaceClass/SubClass,
// bEndpointAddress, bmAttributes) sit at the same offsets in the UAC1 9-byte
// audio endpoint descriptor and the UAC2 7-byte standard one, so one
// implementation serves both. They were duplicated verbatim between the two
// parsers until the input rule arrived; a subtle rule copied into two files
// is a rule that gets fixed in one of them.
//
// Output: the first AS interface with an isochronous OUT endpoint.
// Input: the first with an isochronous IN endpoint and NO isochronous OUT.
// Both return 0xFF for "none".
//
// The input rule's absence test is the load-bearing half. An asynchronous
// OUT interface also carries an IN endpoint -- its feedback endpoint -- and
// on the XMOS UAC1 witness that endpoint declares bmAttributes usage type
// 00, "data", so no property of the endpoint itself distinguishes it from
// audio. Excluding interfaces that have an OUT endpoint sidesteps the
// question rather than trusting a field real hardware fills in wrongly.
uint8_t uac_find_output_streaming_interface(const uint8_t *d, size_t len);
uint8_t uac_find_input_streaming_interface(const uint8_t *d, size_t len);

// Parses a full configuration descriptor. Returns false if the descriptor
// contains no audio streaming interface with an isochronous OUT endpoint --
// the return value still speaks only for the OUTPUT path, so an input-only
// device parses as a failure today. Input fields are filled opportunistically
// whenever an input interface is present.
// On failure the contents of *out are unspecified and callers must not
// read it.
bool uac1_parse_config(const uint8_t *desc, size_t len, UAC1Topology *out);

// Returns the alternate setting number matching the format, or -1.
int uac1_find_alt(const UAC1Topology *t, uint32_t rate, uint8_t channels, uint8_t bits);

// Same, over the INPUT alternate settings. Separate from uac1_find_alt rather
// than a direction flag on it, because every existing caller means "output"
// and a defaulted flag is how that silently stops being true.
int uac1_find_in_alt(const UAC1Topology *t, uint32_t rate, uint8_t channels, uint8_t bits);

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

// The device-side layout an armed stream is built around. Comparing two of
// these answers the only question that matters when a device is claimed while
// a stream is already running: can the descriptors already linked into the
// periodic schedule keep serving what is attached now?
//
// The device ADDRESS is deliberately absent. Every re-arm reads it from the
// live device, which is what lets an unplug/replug of the same device resume
// without tearing the ring down (the self-heal contract in USBAudioOut::
// disconnect()). What must force a rebuild is descriptor layout the ring
// cannot change under itself: the transport (siTD ring vs iTD ring), and the
// endpoint, geometry and packet ceiling every descriptor was filled with.
struct UACStreamConfig {
	uint8_t  is_uac2;            // transport: 0 = FS/siTD ring, 1 = HS/iTD ring
	uint8_t  alternate_setting;
	uint8_t  endpoint_address;
	uint8_t  channels;
	uint8_t  subframe_size;
	uint16_t max_packet_size;
	uint8_t  feedback_endpoint;
};

// Fill `out` from a resolved alternate setting. A null `alt` -- the
// unconfigured device, which is what findAlt() returns while detached --
// yields an all-zero config rather than leaving the caller's storage
// untouched, so a stale stack value can never be compared against an armed
// stream. A null `out` is a no-op.
void uac_stream_config(UACStreamConfig *out, bool is_uac2, const UAC1AltSetting *alt);

// True when a stream armed for `a` can keep running for `b`. Null pointers
// compare false: a stream is never torn down on the strength of a bad
// pointer, and never kept on one either.
bool uac_stream_config_equal(const UACStreamConfig *a, const UACStreamConfig *b);

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

// Cooperative-mode test pattern for the UAC host validator's R7 rule. Fills
// `frames` frames of ch_total x subslot bytes with the validator's Galois
// LFSR sequence -- one fresh value per SAMPLE across all channels, because
// the device checks every subslot it unpacks in arrival order -- 24-bit
// left-justified so it survives the same packing convention as audio. The
// device (lib_xua observer built with UACV_COOPERATIVE=1) regenerates the
// identical sequence and counts discontinuities: the only check that can
// catch this host dropping or duplicating samples, which no host-side
// counter can see.
//
// Seed and taps must match the device's decouple.xc exactly; they are the
// contract, not a tunable.
//
// *lfsr carries the shift register between calls; *primed false makes the
// next value the seed itself, which is the device's lock target. The seed
// is emitted exactly once per priming -- the sequence does not revisit it
// within any capture -- so re-priming mid-stream guarantees the device
// records a discontinuity, which is the honest outcome for a stream that
// actually restarted.
//
// Returns bytes written; 0 when the geometry cannot carry the sequence
// (subslot < 3 drops LFSR bits the device checks) or for null pointers.
// Callers must treat 0 as "pattern NOT sent", never as an empty success.
#define UACV_PATTERN_SEED 0xACE1u
#define UACV_PATTERN_TAPS 0xB4BCD35Cu
uint32_t uacv_pack_pattern(uint8_t *dst, uint32_t frames, uint8_t ch_total,
                           uint8_t subslot, uint32_t *lfsr, bool *primed);

#endif // USB_AUDIO_PARSE_H_
