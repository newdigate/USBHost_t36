// Copyright (c) 2026 Nicholas Newdigate
// SPDX-License-Identifier: MIT
#include "usb_audio_parse.h"
#include <string.h>

#define DT_INTERFACE       0x04
#define DT_ENDPOINT        0x05
#define DT_CS_INTERFACE    0x24
#define AC_HEADER          0x01
#define AC_OUTPUT_TERMINAL 0x03
#define AC_FEATURE_UNIT    0x06
#define AS_FORMAT_TYPE     0x02
#define AUDIO_CLASS        0x01
#define SUBCLASS_CONTROL   0x01
#define SUBCLASS_STREAM    0x02

// Finds the audio streaming interface carrying an isochronous OUT endpoint.
// Returns 0xFF if there is none.
static uint8_t find_output_streaming_interface(const uint8_t *d, size_t len)
{
	uint8_t cur = 0xFF;
	bool cur_is_stream = false;
	size_t i = 0;
	while (i + 1 < len && d[i] >= 2 && i + d[i] <= len) {
		uint8_t l = d[i], t = d[i + 1];
		if (t == DT_INTERFACE && l >= 9) {
			cur_is_stream = (d[i+5] == AUDIO_CLASS && d[i+6] == SUBCLASS_STREAM);
			cur = d[i+2];
		} else if (t == DT_ENDPOINT && l >= 7 && cur_is_stream) {
			if ((d[i+2] & 0x80) == 0 && (d[i+3] & 0x03) == 0x01) return cur;
		}
		i += l;
	}
	return 0xFF;
}

bool uac1_parse_config(const uint8_t *desc, size_t len, UAC1Topology *out)
{
	if (!desc || !out || len < 9) return false;
	memset(out, 0, sizeof(*out));
	out->control_interface = 0xFF;
	out->streaming_interface = 0xFF;

	uint8_t stream_if = find_output_streaming_interface(desc, len);
	if (stream_if == 0xFF) return false;
	out->streaming_interface = stream_if;

	bool in_control = false;
	size_t i = 0;
	while (i + 1 < len && desc[i] >= 2 && i + desc[i] <= len) {
		const uint8_t *b = desc + i;
		uint8_t l = b[0], t = b[1];
		if (t == DT_INTERFACE && l >= 9) {
			in_control = (b[5] == AUDIO_CLASS && b[6] == SUBCLASS_CONTROL);
			if (in_control && out->control_interface == 0xFF)
				out->control_interface = b[2];
		} else if (t == DT_CS_INTERFACE && l >= 8 && in_control && b[2] == AC_HEADER) {
			out->bcd_adc = (uint16_t)b[3] | ((uint16_t)b[4] << 8);
		}
		i += l;
	}
	return true;
}

int uac1_find_alt(const UAC1Topology *t, uint32_t rate, uint8_t channels, uint8_t bits)
{
	(void)t; (void)rate; (void)channels; (void)bits;
	return -1;
}
