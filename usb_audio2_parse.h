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

// Build the 8-byte SETUP packet for a UAC2 Clock Source CUR write of
// CS_SAM_FREQ_CONTROL (UAC2 5.2.5.1.1): class request to the AC interface,
// wValue = control selector 0x01 in the high byte, wIndex = clock entity id
// in the high byte over the AC interface number, wLength = 4 (the rate
// payload is a separate 4-byte little-endian sample rate in Hz).
void uac2_clock_cur_setup(uint8_t setup[8], uint8_t ac_interface, uint8_t clock_id);

#endif // USB_AUDIO2_PARSE_H_
