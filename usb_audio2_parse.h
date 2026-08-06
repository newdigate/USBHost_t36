// Copyright (c) 2026 Nicholas Newdigate
// SPDX-License-Identifier: MIT
//
// USB Audio Class 2.0 descriptor parsing and control-plane helpers.
// Deliberately free of Arduino and USBHost_t36 dependencies so it can be
// unit-tested on the host, like usb_audio_parse.{h,cpp} (UAC1) beside it.
#ifndef USB_AUDIO2_PARSE_H_
#define USB_AUDIO2_PARSE_H_
#include <stdint.h>
#include <stddef.h>
#include "usb_audio_parse.h"

// Build the 8-byte SETUP packet for a UAC2 Clock Source CUR write of
// CS_SAM_FREQ_CONTROL (UAC2 5.2.5.1.1): class request to the AC interface,
// wValue = control selector 0x01 in the high byte, wIndex = clock entity id
// in the high byte over the AC interface number, wLength = 4 (the rate
// payload is a separate 4-byte little-endian sample rate in Hz).
void uac2_clock_cur_setup(uint8_t setup[8], uint8_t ac_interface, uint8_t clock_id);

// --- CS_SAM_FREQ_CONTROL RANGE (UAC2 5.2.2) --------------------------------
//
// UAC2 puts sample rates in a runtime conversation with the clock entity, not
// in the descriptors -- which is why UAC1Topology's rate_count/rate_min/max
// are always 0 for a UAC2 device. Until now the driver simply wrote CUR 44100
// and assumed; a device that does not support 44100 would have been set to a
// rate it never advertised, and the first evidence of that would have been
// wrong-pitch audio.
//
// Build the 8-byte SETUP for GET RANGE. `wlen` is the caller's buffer size:
// the reply is 2 + 12*wNumSubRanges bytes and the device truncates to wlen,
// so a short buffer yields a short (still parseable) reply rather than an
// error.
void uac2_clock_range_setup(uint8_t setup[8], uint8_t ac_interface,
                            uint8_t clock_id, uint16_t wlen);

// Number of subranges the reply actually CONTAINS -- that is, both declared by
// wNumSubRanges and present within `len`. A device that declares six and sends
// two (a truncated reply) reports two, because those are the only two that can
// be read. Returns 0 for a malformed or too-short buffer.
uint16_t uac2_range_count(const uint8_t *buf, size_t len);

// Subrange `idx` as MIN/MAX/RES in Hz. Any pointer may be null. False if idx
// is out of range or the buffer is malformed.
bool uac2_range_get(const uint8_t *buf, size_t len, uint16_t idx,
                    uint32_t *min, uint32_t *max, uint32_t *res);

// Whether `rate` is one the device says it supports.
//
// Per UAC2 5.2.1 a subrange is a triple: values MIN, MIN+RES, MIN+2*RES ...
// up to MAX. RES of 0 means the subrange is not enumerable that way -- either
// a single discrete value (MIN == MAX, which is how most devices list their
// rates) or a continuous span -- and any rate within [MIN,MAX] is accepted.
bool uac2_range_supports(const uint8_t *buf, size_t len, uint32_t rate);

// Why a descriptor set was refused.
//
// uac2_parse_config() collapsed six genuinely different failures into one
// `false`, which is the same defect usb_audio_capture_test's cfg= field had to
// work around at the sketch level: "this device was offered and rejected" and
// "no device ever arrived" are not the same event and must not look alike. A
// bench session spent guessing between them is the cost of not having this.
//
// Ordered as the parse encounters them, so a larger value means the device got
// further before being turned away.
typedef enum {
	UAC2_PARSE_OK = 0,
	UAC2_REJECT_BAD_ARGS,           // null desc/out, or len < 9
	UAC2_REJECT_NO_OUT_STREAM,      // no AS interface owning an iso OUT endpoint
	UAC2_REJECT_NO_AC_HEADER,       // walked the set, never saw an AC HEADER
	UAC2_REJECT_NOT_UAC2,           // AC header present but bcdADC != 0x0200
	UAC2_REJECT_NO_ALTS,            // streaming interface has no operational alt
	UAC2_REJECT_NO_TERMINAL_LINK,   // no AS_GENERAL named a terminal to clock
	UAC2_REJECT_CLOCK_UNRESOLVED,   // chain never reached a CLOCK_SOURCE
} uac2_parse_result;

// Stable short names for the above, for console/log use. Never returns null.
const char *uac2_parse_result_str(uac2_parse_result r);

// The parse, reporting WHY on failure. uac2_parse_config() below is this with
// the reason discarded.
uac2_parse_result uac2_parse_config_ex(const uint8_t *desc, size_t len,
                                       UAC1Topology *out);

// Parse a UAC2 configuration descriptor set (claim-time form: may start at
// the config header or at the first interface/IAD). Fills the shared
// topology struct: bcd_adc, control/streaming interface numbers, resolved
// clock_source_id, and one UAC1AltSetting per operational AS alt (channels,
// subslot, resolution from FORMAT_TYPE I; iso OUT data endpoint + max
// packet; iso IN endpoint on the same interface recorded as feedback).
// rate_count is 0 and rate_min/max are 0: UAC2 rates are a runtime RANGE
// conversation with the clock, not descriptor data. Returns false when no
// AS interface with an iso OUT endpoint exists, the header is not bcdADC
// 0x0200, or the clock chain does not resolve to a single CLOCK_SOURCE.
// The chain is followed through single-input clock SELECTORs and through
// clock MULTIPLIERs (P2); a MULTI-input selector remains out of scope and
// fails closed, because which input is live depends on a runtime CUR
// selection this parser does not perform. Descriptors are never read past `len`; a truncated buffer fails
// only by starving those criteria, so a cut AFTER the needed descriptors
// still parses (the truncation tests pin the exact boundary).
bool uac2_parse_config(const uint8_t *desc, size_t len, UAC1Topology *out);

// The alt setting matching a channel count and bit resolution, or -1. Rate
// is deliberately absent from the match: it is negotiated with the clock at
// runtime.
int uac2_find_alt(const UAC1Topology *t, uint8_t channels, uint8_t bits);

// The same over the INPUT alternate settings, and separate for the same
// reason uac1_find_in_alt is: every existing caller means "output", and a
// defaulted direction flag is how that silently stops being true.
int uac2_find_in_alt(const UAC1Topology *t, uint8_t channels, uint8_t bits);

#endif // USB_AUDIO2_PARSE_H_
