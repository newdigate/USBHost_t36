// Copyright (c) 2026 Nicholas Newdigate
// SPDX-License-Identifier: MIT
#include "usb_audio_parse.h"
#include <string.h>

#define DT_INTERFACE       0x04
#define DT_ENDPOINT        0x05
#define DT_CS_ENDPOINT     0x25
#define AS_EP_GENERAL      0x01
#define DT_CS_INTERFACE    0x24
#define AC_HEADER          0x01
#define AC_OUTPUT_TERMINAL 0x03
#define AC_FEATURE_UNIT    0x06
#define AS_FORMAT_TYPE     0x02
#define AUDIO_CLASS        0x01
#define SUBCLASS_CONTROL   0x01
#define SUBCLASS_STREAM    0x02
#define EP_DIR_MASK        0x80
#define EP_DIR_OUT         0x00
#define EP_XFER_TYPE_MASK  0x03
#define EP_XFER_ISO        0x01

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
			if ((d[i+2] & EP_DIR_MASK) == EP_DIR_OUT
			    && (d[i+3] & EP_XFER_TYPE_MASK) == EP_XFER_ISO) return cur;
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
				// bUnitID 0 is not a valid UAC1 unit/terminal ID (IDs run
				// 1..255); reject it here so it can never be confused with
				// the feature_unit_id "not found" sentinel below.
				if (fu_count < UAC1_MAX_ALTS && b[3] != 0) fu_ids[fu_count++] = b[3];
			} else if (in_stream && alt && b[2] == AS_FORMAT_TYPE && l >= 11) {
				alt->channels       = b[4];
				alt->subframe_size  = b[5];
				alt->bit_resolution = b[6];
				// bSamFreqType == 0 means a continuous min/max range
				// (tSamFreqMin/tSamFreqMax). Otherwise it is a count of
				// discrete frequencies, each three bytes little-endian
				// from b[8].
				uint8_t nfreq = b[7];
				if (nfreq == 0) {
					if (l >= 14) {
						alt->rate_count = 0;
						alt->rate_min = (uint32_t)b[8] | ((uint32_t)b[9] << 8)
						              | ((uint32_t)b[10] << 16);
						alt->rate_max = (uint32_t)b[11] | ((uint32_t)b[12] << 8)
						              | ((uint32_t)b[13] << 16);
					}
				} else {
					for (uint8_t k = 0; k < nfreq && alt->rate_count < UAC1_MAX_RATES; k++) {
						uint32_t o = 8u + 3u * (uint32_t)k;
						if (o + 2u >= (uint32_t)l) break;   // descriptor truncated
						alt->rates[alt->rate_count++] =
							(uint32_t)b[o] | ((uint32_t)b[o + 1] << 8)
							| ((uint32_t)b[o + 2] << 16);
					}
				}
			}
		} else if (t == DT_CS_ENDPOINT && l >= 4 && in_stream && alt
		           && b[2] == AS_EP_GENERAL) {
			alt->ep_controls = b[3];
		} else if (t == DT_ENDPOINT && l >= 7 && in_stream && alt) {
			bool is_out = (b[2] & EP_DIR_MASK) == EP_DIR_OUT;
			bool is_iso = (b[3] & EP_XFER_TYPE_MASK) == EP_XFER_ISO;
			if (is_out && is_iso) {
				// The data endpoint. Assigning unconditionally here used to
				// let a trailing feedback endpoint overwrite it, which only
				// showed up on the first device with two endpoints in one
				// alternate setting.
				alt->endpoint_address    = b[2];
				alt->endpoint_attributes = b[3];
				alt->max_packet_size     = (uint16_t)b[4] | ((uint16_t)b[5] << 8);
				// bLength 9 is the audio endpoint descriptor, which adds
				// bRefresh and bSynchAddress. Zero means no feedback endpoint.
				if (l >= 9 && b[8] != 0) alt->feedback_endpoint = b[8];
			} else if (!is_out && is_iso) {
				if (alt->feedback_endpoint == 0) alt->feedback_endpoint = b[2];
				if (l >= 9) alt->feedback_refresh = b[7];
			}
		}
		i += l;
	}
	for (uint8_t k = 0; k < fu_count; k++) {
		if (fu_ids[k] == speaker_src) { out->feature_unit_id = speaker_src; break; }
	}
	// Defensive, not reachable in practice: once find_output_streaming_interface
	// has located stream_if, the main pass above always records at least one
	// alt for that interface, so alt_count > 0 here always holds.
	return out->alt_count > 0;
}

bool uac1_alt_supports_rate(const UAC1AltSetting *alt, uint32_t rate)
{
	if (!alt || rate == 0) return false;

	if (alt->rate_count == 0) {
		// Continuous range. Both bounds zero means the format descriptor
		// was missing or malformed, which is not the same as "any rate".
		if (alt->rate_min == 0 && alt->rate_max == 0) return false;
		return rate >= alt->rate_min && rate <= alt->rate_max;
	}

	for (uint8_t i = 0; i < alt->rate_count; i++) {
		if (alt->rates[i] == rate) return true;
	}
	return false;
}

bool uac1_alt_needs_rate_request(const UAC1AltSetting *alt)
{
	if (!alt) return false;
	// Bit 0 of the class-specific endpoint bmAttributes is the sampling
	// frequency control. Without it the device has no way to be told a rate,
	// so the alternate setting is all there is.
	if (!(alt->ep_controls & 0x01)) return false;
	// A single discrete rate is already fully determined by the alt setting.
	return alt->rate_count != 1;
}

int uac1_find_alt(const UAC1Topology *t, uint32_t rate, uint8_t channels, uint8_t bits)
{
	if (!t) return -1;
	for (uint8_t k = 0; k < t->alt_count; k++) {
		const UAC1AltSetting *a = &t->alts[k];
		if (a->endpoint_address == 0) continue;      // zero-bandwidth alt 0
		if (a->channels != channels) continue;
		if (a->bit_resolution != bits) continue;
		if (!uac1_alt_supports_rate(a, rate)) continue;
		return (int)a->alternate_setting;
	}
	return -1;
}

// One frame's worth of samples, carrying the fraction forward. `units_per_frame`
// is how many rate units make one frame: 1000 when the rate is in hertz,
// 1000000 when it is in millihertz.
static uint16_t frame_bytes_scaled(uint32_t *accum, uint32_t rate_units,
                                   uint32_t units_per_frame, uint8_t channels,
                                   uint8_t bytes_per_sample)
{
	if (!accum || rate_units == 0 || channels == 0 || bytes_per_sample == 0) return 0;

	*accum += rate_units;
	uint32_t samples = *accum / units_per_frame;   // whole samples this frame
	*accum -= samples * units_per_frame;           // carry the fraction forward
	return (uint16_t)(samples * channels * bytes_per_sample);
}

uint16_t uac1_frame_bytes(uint32_t *accum, uint32_t rate, uint8_t channels,
                          uint8_t bytes_per_sample)
{
	return frame_bytes_scaled(accum, rate, 1000u, channels, bytes_per_sample);
}

uint16_t uac1_frame_bytes_mhz(uint32_t *accum, uint32_t rate_mhz, uint8_t channels,
                              uint8_t bytes_per_sample)
{
	return frame_bytes_scaled(accum, rate_mhz, 1000000u, channels, bytes_per_sample);
}
