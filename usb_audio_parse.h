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

struct UAC1AltSetting {
	uint8_t  alternate_setting;
	uint8_t  endpoint_address;
	uint8_t  endpoint_attributes;
	uint16_t max_packet_size;
	uint8_t  channels;
	uint8_t  subframe_size;
	uint8_t  bit_resolution;
	uint32_t sample_rate;
};

struct UAC1Topology {
	uint16_t bcd_adc;
	uint8_t  control_interface;    // 0xFF if none
	uint8_t  streaming_interface;  // 0xFF if none
	uint8_t  feature_unit_id;      // 0 if none
	uint8_t  alt_count;
	UAC1AltSetting alts[UAC1_MAX_ALTS];
};

// Parses a full configuration descriptor. Returns false if the descriptor
// contains no audio streaming interface with an isochronous OUT endpoint.
// On failure the contents of *out are unspecified and callers must not
// read it.
bool uac1_parse_config(const uint8_t *desc, size_t len, UAC1Topology *out);

// Returns the alternate setting number matching the format, or -1.
int uac1_find_alt(const UAC1Topology *t, uint32_t rate, uint8_t channels, uint8_t bits);

#endif // USB_AUDIO_PARSE_H_
