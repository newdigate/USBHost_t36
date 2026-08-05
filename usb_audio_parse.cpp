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
// USB 2.0 section 9.6.6, bmAttributes bits 5..4 Usage Type. Only 01 is an
// explicit feedback endpoint: 00 is a data endpoint and 10 is an IMPLICIT
// FEEDBACK DATA endpoint, which carries audio. Classifying on direction
// instead -- "any IN iso endpoint is the feedback endpoint" -- is right only
// while a device has at most one IN endpoint per alt, and points the rate
// decoder at audio samples the moment one carries both directions.
#define EP_USAGE_MASK      0x30
#define EP_USAGE_FEEDBACK  0x10

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

// The audio streaming interface carrying audio TO us: one with an isochronous
// IN endpoint and NO isochronous OUT endpoint. Returns 0xFF if there is none.
//
// The "no OUT endpoint" half is what makes this safe. An asynchronous OUT
// interface also carries an IN endpoint -- its feedback endpoint -- and on the
// XMOS UAC1 witness that endpoint declares bmAttributes usage type 00,
// "data", so no property of the endpoint itself distinguishes it from audio.
// Excluding interfaces that have an OUT endpoint sidesteps the question rather
// than trusting a field real hardware fills in wrongly.
static uint8_t find_input_streaming_interface(const uint8_t *d, size_t len)
{
	uint8_t cur = 0xFF, best = 0xFF;
	bool cur_is_stream = false, saw_in = false, saw_out = false;
	size_t i = 0;
	while (i + 1 < len && d[i] >= 2 && i + d[i] <= len) {
		uint8_t l = d[i], t = d[i + 1];
		if (t == DT_INTERFACE && l >= 9) {
			if (cur_is_stream && saw_in && !saw_out && best == 0xFF) best = cur;
			cur_is_stream = (d[i+5] == AUDIO_CLASS && d[i+6] == SUBCLASS_STREAM);
			cur = d[i+2];
			saw_in = saw_out = false;
		} else if (t == DT_ENDPOINT && l >= 7 && cur_is_stream) {
			if ((d[i+3] & EP_XFER_TYPE_MASK) == EP_XFER_ISO) {
				if ((d[i+2] & EP_DIR_MASK) == EP_DIR_OUT) saw_out = true;
				else saw_in = true;
			}
		}
		i += l;
	}
	if (cur_is_stream && saw_in && !saw_out && best == 0xFF) best = cur;
	return best;
}

bool uac1_parse_config(const uint8_t *desc, size_t len, UAC1Topology *out)
{
	if (!desc || !out || len < 9) return false;
	memset(out, 0, sizeof(*out));
	out->control_interface = 0xFF;
	out->streaming_interface = 0xFF;
	out->input_streaming_interface = 0xFF;
	out->in_alt_count = 0;

	uint8_t stream_if = find_output_streaming_interface(desc, len);
	uint8_t in_if = find_input_streaming_interface(desc, len);
	out->input_streaming_interface = in_if;
	if (stream_if == 0xFF) return false;
	out->streaming_interface = stream_if;

	bool in_control = false, in_stream = false, is_in_stream = false;
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
			is_in_stream = (b[5] == AUDIO_CLASS && b[6] == SUBCLASS_STREAM
			                && in_if != 0xFF && b[2] == in_if);
			alt = 0;
			if (in_control && out->control_interface == 0xFF)
				out->control_interface = b[2];
			if (in_stream && out->alt_count < UAC1_MAX_ALTS) {
				alt = &out->alts[out->alt_count++];
				memset(alt, 0, sizeof(*alt));
				alt->alternate_setting = b[3];
			} else if (is_in_stream && out->in_alt_count < UAC1_MAX_ALTS) {
				// Input alts are filled by the same descriptor handlers
				// below; only the array differs. in_stream stays false, so
				// nothing that means "output" starts meaning both.
				alt = &out->in_alts[out->in_alt_count++];
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
			} else if (alt && b[2] == AS_FORMAT_TYPE && l >= 11) {
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
		} else if (t == DT_CS_ENDPOINT && l >= 4 && alt
		           && b[2] == AS_EP_GENERAL) {
			alt->ep_controls = b[3];
		} else if (t == DT_ENDPOINT && l >= 7 && alt) {
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
			} else if (!is_out && is_iso && is_in_stream) {
				// On the INPUT interface an IN isochronous endpoint is the
				// audio data endpoint, not feedback. An input stream has no
				// feedback endpoint at all: the device is the source and
				// sets the rate, so it has nothing to report back.
				alt->endpoint_address    = b[2];
				alt->endpoint_attributes = b[3];
				alt->max_packet_size     = (uint16_t)b[4] | ((uint16_t)b[5] << 8);
			} else if (!is_out && is_iso) {
				// Two ways to be the feedback endpoint, and BOTH are needed.
				//
				// bSynchAddress on the data endpoint is the authority, because
				// real hardware does not set its own usage bits honestly: the
				// XMOS UAC1 witness declares its feedback endpoint 0x82 with
				// bmAttributes 0x01, usage type 00 = "data" (fixture
				// xmos_uac1_async_feedback.bin). Believing the usage bits alone
				// would lose that device's feedback entirely.
				//
				// The usage bits are the fallback for a device that declares
				// 01 = Feedback but names no bSynchAddress, which is legal.
				//
				// What neither test admits is an IN endpoint that is plain
				// audio -- usage 00 or 10, unnamed by bSynchAddress -- which is
				// what a full-duplex device puts here. That used to be captured
				// as the feedback endpoint on direction alone, pointing the
				// rate decoder at samples.
				bool named  = (alt->feedback_endpoint == b[2]);
				bool claims = (b[3] & EP_USAGE_MASK) == EP_USAGE_FEEDBACK;
				if (named || claims) {
					if (alt->feedback_endpoint == 0) alt->feedback_endpoint = b[2];
					if (l >= 9) alt->feedback_refresh = b[7];
				}
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

int uac1_find_in_alt(const UAC1Topology *t, uint32_t rate, uint8_t channels, uint8_t bits)
{
	if (!t) return -1;
	for (uint8_t k = 0; k < t->in_alt_count; k++) {
		const UAC1AltSetting *a = &t->in_alts[k];
		if (a->endpoint_address == 0) continue;      // zero-bandwidth alt 0
		if (a->channels != channels) continue;
		if (a->bit_resolution != bits) continue;
		if (!uac1_alt_supports_rate(a, rate)) continue;
		return (int)a->alternate_setting;
	}
	return -1;
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

uint16_t uac2_uframe_bytes_mhz(uint32_t *accum, uint32_t rate_mhz, uint8_t channels,
                               uint8_t bytes_per_sample)
{
	return frame_bytes_scaled(accum, rate_mhz, 8000000u, channels, bytes_per_sample);
}

uint32_t uac_pack16(uint8_t *dst, const int16_t *src, uint32_t frames,
                    uint8_t ch_live, uint8_t ch_total, uint8_t subslot)
{
	if (!dst || !src || subslot < 2 || subslot > 4 || ch_live > ch_total) return 0;
	uint8_t *p = dst;
	for (uint32_t f = 0; f < frames; f++) {
		for (uint8_t c = 0; c < ch_total; c++) {
			int16_t s = (c < ch_live) ? src[f * ch_live + c] : 0;
			uint8_t lo = (uint8_t)s;
			uint8_t hi = (uint8_t)((uint16_t)s >> 8);
			switch (subslot) {
			case 2: *p++ = lo; *p++ = hi; break;
			case 3: *p++ = 0; *p++ = lo; *p++ = hi; break;
			case 4: *p++ = 0; *p++ = 0; *p++ = lo; *p++ = hi; break;
			}
		}
	}
	return (uint32_t)(p - dst);
}

uint32_t uacv_pack_pattern(uint8_t *dst, uint32_t frames, uint8_t ch_total,
                           uint8_t subslot, uint32_t *lfsr, bool *primed)
{
	if (!dst || !lfsr || !primed || subslot < 3 || subslot > 4) return 0;
	uint8_t *p = dst;
	for (uint32_t f = 0; f < frames; f++) {
		for (uint8_t c = 0; c < ch_total; c++) {
			if (!*primed) {
				*lfsr = UACV_PATTERN_SEED;
				*primed = true;
			} else {
				uint32_t lsb = *lfsr & 1u;
				*lfsr >>= 1;
				if (lsb) *lfsr ^= UACV_PATTERN_TAPS;
			}
			// The device's expected word is (lfsr << 8): 24 significant
			// bits, low byte clear. Emit exactly that, little-endian, with
			// the pad byte at the least-significant end like uac_pack16.
			uint32_t v = *lfsr << 8;
			if (subslot == 4) *p++ = 0;
			*p++ = (uint8_t)(v >> 8);
			*p++ = (uint8_t)(v >> 16);
			*p++ = (uint8_t)(v >> 24);
		}
	}
	return (uint32_t)(p - dst);
}

void uac_stream_config(UACStreamConfig *out, bool is_uac2, const UAC1AltSetting *alt)
{
	if (!out) return;
	memset(out, 0, sizeof(*out));
	if (!alt) return;
	out->is_uac2           = is_uac2 ? 1 : 0;
	out->alternate_setting = alt->alternate_setting;
	out->endpoint_address  = alt->endpoint_address;
	out->channels          = alt->channels;
	out->subframe_size     = alt->subframe_size;
	out->max_packet_size   = alt->max_packet_size;
	out->feedback_endpoint = alt->feedback_endpoint;
}

bool uac_stream_config_equal(const UACStreamConfig *a, const UACStreamConfig *b)
{
	if (!a || !b) return false;
	// Field by field rather than memcmp: the padding a compiler inserts
	// around these members is not part of the comparison, and memcmp would
	// only be safe for as long as every config in existence came from
	// uac_stream_config()'s memset. Listing the fields also makes it
	// visible which ones force a rebuild.
	return a->is_uac2           == b->is_uac2
	    && a->alternate_setting == b->alternate_setting
	    && a->endpoint_address  == b->endpoint_address
	    && a->channels          == b->channels
	    && a->subframe_size     == b->subframe_size
	    && a->max_packet_size   == b->max_packet_size
	    && a->feedback_endpoint == b->feedback_endpoint;
}
