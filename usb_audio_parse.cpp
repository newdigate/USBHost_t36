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

static bool is_speaker_terminal(uint16_t tt)
{
	return tt == 0x0301   // Speaker
	    || tt == 0x0302   // Headphones
	    || tt == 0x0402;  // Headset
}

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

	bool in_control = false, in_stream = false;
	UAC1AltSetting *alt = 0;
	uint8_t fu_ids[UAC1_MAX_ALTS];
	uint8_t fu_count = 0;
	uint8_t speaker_src = 0;
	size_t i = 0;
	while (i + 1 < len && desc[i] >= 2 && i + desc[i] <= len) {
		const uint8_t *b = desc + i;
		uint8_t l = b[0], t = b[1];
		if (t == DT_INTERFACE && l >= 9) {
			in_control = (b[5] == AUDIO_CLASS && b[6] == SUBCLASS_CONTROL);
			in_stream  = (b[5] == AUDIO_CLASS && b[6] == SUBCLASS_STREAM
			              && b[2] == stream_if);
			alt = 0;
			if (in_control && out->control_interface == 0xFF)
				out->control_interface = b[2];
			if (in_stream && out->alt_count < UAC1_MAX_ALTS) {
				alt = &out->alts[out->alt_count++];
				memset(alt, 0, sizeof(*alt));
				alt->alternate_setting = b[3];
			}
		} else if (t == DT_CS_INTERFACE && l >= 3) {
			if (in_control && b[2] == AC_HEADER && l >= 8) {
				out->bcd_adc = (uint16_t)b[3] | ((uint16_t)b[4] << 8);
			} else if (in_control && b[2] == AC_OUTPUT_TERMINAL && l >= 9) {
				uint16_t tt = (uint16_t)b[4] | ((uint16_t)b[5] << 8);
				if (is_speaker_terminal(tt)) speaker_src = b[7];  // bSourceID
			} else if (in_control && b[2] == AC_FEATURE_UNIT && l >= 4) {
				if (fu_count < UAC1_MAX_ALTS) fu_ids[fu_count++] = b[3];
			} else if (in_stream && alt && b[2] == AS_FORMAT_TYPE && l >= 11) {
				alt->channels       = b[4];
				alt->subframe_size  = b[5];
				alt->bit_resolution = b[6];
				if (b[7] >= 1)  // bSamFreqType: discrete frequencies
					alt->sample_rate = (uint32_t)b[8]
					                 | ((uint32_t)b[9] << 8)
					                 | ((uint32_t)b[10] << 16);
			}
		} else if (t == DT_ENDPOINT && l >= 7 && in_stream && alt) {
			alt->endpoint_address    = b[2];
			alt->endpoint_attributes = b[3];
			alt->max_packet_size     = (uint16_t)b[4] | ((uint16_t)b[5] << 8);
		}
		i += l;
	}
	for (uint8_t k = 0; k < fu_count; k++) {
		if (fu_ids[k] == speaker_src) { out->feature_unit_id = speaker_src; break; }
	}
	return out->alt_count > 0;
}

int uac1_find_alt(const UAC1Topology *t, uint32_t rate, uint8_t channels, uint8_t bits)
{
	if (!t) return -1;
	for (uint8_t k = 0; k < t->alt_count; k++) {
		const UAC1AltSetting *a = &t->alts[k];
		if (a->sample_rate == rate && a->channels == channels &&
		    a->bit_resolution == bits && a->endpoint_address != 0)
			return (int)a->alternate_setting;
	}
	return -1;
}
