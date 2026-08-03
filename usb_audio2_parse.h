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

// Parse a UAC2 configuration descriptor set (claim-time form: may start at
// the config header or at the first interface/IAD). Fills the shared
// topology struct: bcd_adc, control/streaming interface numbers, resolved
// clock_source_id, and one UAC1AltSetting per operational AS alt (channels,
// subslot, resolution from FORMAT_TYPE I; iso OUT data endpoint + max
// packet; iso IN endpoint on the same interface recorded as feedback).
// rate_count is 0 and rate_min/max are 0: UAC2 rates are a runtime RANGE
// conversation with the clock, not descriptor data. Returns false when no
// AS interface with an iso OUT endpoint exists, the header is not bcdADC
// 0x0200, or the clock chain does not resolve to a single CLOCK_SOURCE
// (multi-input selectors and multipliers are out of scope and must fail
// closed). Descriptors are never read past `len`; a truncated buffer fails
// only by starving those criteria, so a cut AFTER the needed descriptors
// still parses (the truncation tests pin the exact boundary).
bool uac2_parse_config(const uint8_t *desc, size_t len, UAC1Topology *out);

// The alt setting matching a channel count and bit resolution, or -1. Rate
// is deliberately absent from the match: it is negotiated with the clock at
// runtime.
int uac2_find_alt(const UAC1Topology *t, uint8_t channels, uint8_t bits);

#endif // USB_AUDIO2_PARSE_H_
