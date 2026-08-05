// Copyright (c) 2026 Nicholas Newdigate
// SPDX-License-Identifier: MIT
#include "usb_audio2_parse.h"
#include <string.h>

#define DT_INTERFACE       0x04
#define DT_ENDPOINT        0x05
#define DT_CS_INTERFACE    0x24
#define AUDIO_CLASS        0x01
#define SUBCLASS_CONTROL   0x01
#define SUBCLASS_STREAM    0x02
#define EP_DIR_MASK        0x80
#define EP_DIR_OUT         0x00
// USB 2.0 section 9.6.6 bits 5..4: 01 is an explicit feedback endpoint; 00 is
// data and 10 is implicit-feedback DATA. See the note in usb_audio_parse.cpp.
#define EP_USAGE_MASK      0x30
#define EP_USAGE_FEEDBACK  0x10
#define EP_XFER_TYPE_MASK  0x03
#define EP_XFER_ISO        0x01

// AC (AudioControl) class-specific interface descriptor subtypes, UAC2 4.7.2.
#define AC_HEADER          0x01
#define AC_INPUT_TERMINAL  0x02
#define AC_OUTPUT_TERMINAL 0x03
#define AC_CLOCK_SOURCE    0x0A
#define AC_CLOCK_SELECTOR  0x0B

// AS (AudioStreaming) class-specific interface descriptor subtypes, UAC2 4.9.2.
#define AS_GENERAL         0x01
#define AS_FORMAT_TYPE     0x02

// Small fixed pools for the clock-chain resolution below. Real devices carry
// a handful of clock entities and input terminals each; overflow silently
// drops entries (bounds-checked at every insertion) rather than corrupting
// memory, same convention as fu_ids in uac1_parse_config.
#define UAC2_MAX_CLOCKS    8
#define UAC2_MAX_TERMINALS 8

// id -> upstream id. Serves the selector table (clock selector -> clock) and
// the terminal table (terminal -> its bCSourceID) alike; both are one hop of
// the same walk.
struct uac2_id_pair { uint8_t id; uint8_t src; };

// Resolve a bTerminalLink to the CLOCK_SOURCE entity that ultimately clocks
// it, following single-input clock selectors. Returns 0 -- not a valid UAC2
// entity ID, so unambiguous -- when the terminal is unknown or the chain ends
// somewhere that is not a recorded source.
//
// Bounded by UAC2_MAX_CLOCKS hops so a malformed cyclic map cannot loop
// forever; real chains are one or two hops deep.
static uint8_t resolve_clock(uint8_t terminal_link,
                             const uac2_id_pair *terminals, uint8_t terminal_count,
                             const uac2_id_pair *selectors, uint8_t selector_count,
                             const uint8_t *sources, uint8_t source_count)
{
	uint8_t cid = 0;
	bool found = false;
	for (uint8_t k = 0; k < terminal_count; k++) {
		if (terminals[k].id == terminal_link) { cid = terminals[k].src; found = true; break; }
	}
	if (!found) return 0;

	for (uint8_t hop = 0; hop < UAC2_MAX_CLOCKS; hop++) {
		for (uint8_t k = 0; k < source_count; k++)
			if (sources[k] == cid) return cid;
		bool advanced = false;
		for (uint8_t k = 0; k < selector_count; k++) {
			if (selectors[k].id == cid) { cid = selectors[k].src; advanced = true; break; }
		}
		if (!advanced) return 0;   // neither a source nor a resolvable selector
	}
	return 0;
}

void uac2_clock_cur_setup(uint8_t setup[8], uint8_t ac_interface, uint8_t clock_id)
{
	setup[0] = 0x21;              // class, interface recipient, host-to-device
	setup[1] = 0x01;              // CUR
	setup[2] = 0x00;
	setup[3] = 0x01;              // CS_SAM_FREQ_CONTROL << 8
	setup[4] = ac_interface;
	setup[5] = clock_id;
	setup[6] = 0x04;              // wLength: 4-byte rate follows
	setup[7] = 0x00;
}

// The two interface finders live in usb_audio_parse.cpp and serve both class
// versions -- this file used to carry a verbatim copy of the output one. The
// UAC2 standard endpoint descriptor is 7 bytes instead of UAC1's 9, but the
// fields they read (bEndpointAddress at [2], bmAttributes at [3]) sit at the
// same offsets in both, so one implementation is correct for both and the
// input rule's absence test exists in exactly one place.

bool uac2_parse_config(const uint8_t *desc, size_t len, UAC1Topology *out)
{
	if (!desc || !out || len < 9) return false;
	memset(out, 0, sizeof(*out));
	out->control_interface = 0xFF;
	out->streaming_interface = 0xFF;
	out->input_streaming_interface = 0xFF;

	uint8_t stream_if = uac_find_output_streaming_interface(desc, len);
	uint8_t in_if = uac_find_input_streaming_interface(desc, len);
	out->input_streaming_interface = in_if;
	if (stream_if == 0xFF) return false;
	out->streaming_interface = stream_if;

	bool in_control = false, in_stream = false, is_in_stream = false;
	bool got_header = false;
	UAC1AltSetting *alt = 0;

	// Clock chain tables, filled while walking the AC interface.
	uint8_t clock_source_ids[UAC2_MAX_CLOCKS];
	uint8_t clock_source_count = 0;
	uac2_id_pair selectors[UAC2_MAX_CLOCKS];
	uint8_t selector_count = 0;
	// bTerminalLink -> bCSourceID, for BOTH terminal kinds. An OUT stream's
	// alt links to an Input Terminal (USB is the source); an IN stream's
	// links to an Output Terminal of type USB streaming (UAC2 3.13.2), so a
	// table of input terminals alone cannot resolve the input clock. The two
	// descriptors put bCSourceID at different offsets -- b[7] for INPUT
	// (4.7.2.4), b[8] for OUTPUT (4.7.2.5), because OUTPUT carries a
	// bSourceID before it -- which is the whole reason this needs two
	// handlers rather than one.
	uac2_id_pair terminals[UAC2_MAX_TERMINALS];
	uint8_t terminal_count = 0;

	// The operational streaming alt's AS_GENERAL names the terminal it
	// feeds; resolved against the tables above once the whole set is walked.
	uint8_t terminal_link = 0;
	bool have_terminal_link = false;
	uint8_t in_terminal_link = 0;
	bool have_in_terminal_link = false;

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
				// Same handlers below fill it; only the array differs, as
				// on the UAC1 side. in_stream stays false, so nothing that
				// means "output" quietly starts meaning both.
				alt = &out->in_alts[out->in_alt_count++];
				memset(alt, 0, sizeof(*alt));
				alt->alternate_setting = b[3];
			}
		} else if (t == DT_CS_INTERFACE && l >= 3) {
			uint8_t subtype = b[2];
			if (in_control && subtype == AC_HEADER && l >= 5) {
				out->bcd_adc = (uint16_t)b[3] | ((uint16_t)b[4] << 8);
				got_header = true;
			} else if (in_control && subtype == AC_CLOCK_SOURCE && l >= 4) {
				if (clock_source_count < UAC2_MAX_CLOCKS)
					clock_source_ids[clock_source_count++] = b[3];
			} else if (in_control && subtype == AC_CLOCK_SELECTOR && l >= 6) {
				// l >= 6 is enough for the fields read (id, pin count, first
				// source) -- not the descriptor's canonical 8-byte length.
				// Only a single-input selector resolves unambiguously to one
				// upstream clock; a multi-input selector's actual source
				// depends on a runtime CUR selection this parser does not
				// perform, so it is deliberately left out of the map and
				// any chain running through it fails closed below.
				if (b[4] == 1 && selector_count < UAC2_MAX_CLOCKS) {
					selectors[selector_count].id  = b[3];
					selectors[selector_count].src = b[5];
					selector_count++;
				}
			} else if (in_control && subtype == AC_INPUT_TERMINAL && l >= 8) {
				if (terminal_count < UAC2_MAX_TERMINALS) {
					terminals[terminal_count].id  = b[3];
					terminals[terminal_count].src = b[7];
					terminal_count++;
				}
			} else if (in_control && subtype == AC_OUTPUT_TERMINAL && l >= 9) {
				// bCSourceID at b[8], one further along than INPUT's --
				// see the table comment above.
				if (terminal_count < UAC2_MAX_TERMINALS) {
					terminals[terminal_count].id  = b[3];
					terminals[terminal_count].src = b[8];
					terminal_count++;
				}
			} else if (alt && subtype == AS_GENERAL && l >= 11) {
				// A streaming interface's alts must all name the same
				// terminal per the class spec; the last one seen wins here
				// rather than cross-checking agreement.
				if (in_stream) {
					terminal_link = b[3];
					have_terminal_link = true;
				} else {
					in_terminal_link = b[3];
					have_in_terminal_link = true;
				}
				alt->channels = b[10];
			} else if (alt && subtype == AS_FORMAT_TYPE && l >= 6) {
				alt->subframe_size  = b[4];
				alt->bit_resolution = b[5];
			}
		} else if (t == DT_ENDPOINT && l >= 7 && alt) {
			bool is_out = (b[2] & EP_DIR_MASK) == EP_DIR_OUT;
			bool is_iso = (b[3] & EP_XFER_TYPE_MASK) == EP_XFER_ISO;
			if (is_out && is_iso) {
				alt->endpoint_address    = b[2];
				alt->endpoint_attributes = b[3];
				alt->max_packet_size  = (uint16_t)b[4] | ((uint16_t)b[5] << 8);
			} else if (!is_out && is_iso && is_in_stream) {
				// On the INPUT interface an IN isochronous endpoint is the
				// audio data endpoint, not feedback -- and there is no
				// feedback endpoint to confuse it with, because the device
				// is the source and has nothing to report back about a
				// rate it sets itself.
				alt->endpoint_address    = b[2];
				alt->endpoint_attributes = b[3];
				alt->max_packet_size  = (uint16_t)b[4] | ((uint16_t)b[5] << 8);
			} else if (!is_out && is_iso
			           && (b[3] & EP_USAGE_MASK) == EP_USAGE_FEEDBACK) {
				// First explicit FEEDBACK endpoint wins, and its
				// wMaxPacketSize rides along for the iTD reader. An IN data
				// endpoint on the same alt (full duplex) is not this.
				if (alt->feedback_endpoint == 0) {
					alt->feedback_endpoint = b[2];
					alt->feedback_max_packet =
					    (uint16_t)b[4] | ((uint16_t)b[5] << 8);
				}
			}
		}
		i += l;
	}

	if (!got_header || out->bcd_adc != 0x0200) return false;
	if (out->alt_count == 0) return false;
	if (!have_terminal_link) return false;

	uint8_t cid = resolve_clock(terminal_link, terminals, terminal_count,
	                            selectors, selector_count,
	                            clock_source_ids, clock_source_count);
	if (cid == 0) return false;
	out->clock_source_id = cid;

	// The input side is opportunistic: an unresolvable input clock leaves
	// in_clock_source_id 0 and the parse still succeeds, because everything
	// the output path needs is already established. A driver that finds 0
	// here must not issue a rate request for the input stream -- which on a
	// device sharing one clock between directions costs nothing, and on a
	// device with two is exactly the request that would go to the wrong
	// entity.
	if (have_in_terminal_link) {
		out->in_clock_source_id =
		    resolve_clock(in_terminal_link, terminals, terminal_count,
		                  selectors, selector_count,
		                  clock_source_ids, clock_source_count);
	}

	return true;
}

int uac2_find_alt(const UAC1Topology *t, uint8_t channels, uint8_t bits)
{
	if (!t) return -1;
	for (uint8_t k = 0; k < t->alt_count; k++) {
		const UAC1AltSetting *a = &t->alts[k];
		if (a->endpoint_address == 0) continue;      // zero-bandwidth alt 0
		if (a->channels != channels) continue;
		if (a->bit_resolution != bits) continue;
		return (int)a->alternate_setting;
	}
	return -1;
}

int uac2_find_in_alt(const UAC1Topology *t, uint8_t channels, uint8_t bits)
{
	if (!t) return -1;
	for (uint8_t k = 0; k < t->in_alt_count; k++) {
		const UAC1AltSetting *a = &t->in_alts[k];
		if (a->endpoint_address == 0) continue;      // zero-bandwidth alt 0
		if (a->channels != channels) continue;
		if (a->bit_resolution != bits) continue;
		return (int)a->alternate_setting;
	}
	return -1;
}
