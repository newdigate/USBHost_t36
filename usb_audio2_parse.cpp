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
#define AC_CLOCK_MULTIPLIER 0x0C

// AS (AudioStreaming) class-specific interface descriptor subtypes, UAC2 4.9.2.
#define AS_GENERAL         0x01
#define AS_FORMAT_TYPE     0x02

// Small fixed pools for the clock-chain resolution below. Real devices carry
// a handful of clock entities and input terminals each; overflow silently
// drops entries (bounds-checked at every insertion) rather than corrupting
// memory, same convention as fu_ids in uac1_parse_config.
#define UAC2_MAX_CLOCKS    8
#define UAC2_MAX_TERMINALS 8
// Audio functions tried before giving up. Two is the realistic maximum (a
// composite playback + capture device); four leaves room without letting a
// malformed set with a repeating IAD pattern cost an unbounded number of full
// descriptor walks.
#define UAC2_MAX_FUNCTIONS 4

// id -> upstream id. Serves the one-hop clock map (single-input clock
// SELECTORs and clock MULTIPLIERs alike -- both have exactly one resolvable
// upstream) and the terminal table (terminal -> its bCSourceID); all of them
// are one hop of the same walk.
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

// Byte span of ONE audio function's interface descriptors, delimited by its
// Interface Association Descriptor (USB 2.0 ECN; UAC2 3.1 requires one).
//
// Why this matters: every finder and the walk below identify interfaces by
// CLASS. On a device exposing two audio functions -- a composite headset with
// separate speaker and microphone functions is the ordinary case -- that means
// function A's AudioControl interface and function B's AudioStreaming
// interface can be assembled into a single topology that describes no real
// device, and the clock chain resolved across the boundary between them. The
// MC200 has exactly one audio function and its IAD is the first descriptor in
// the captured set, so nothing here has ever been exercised against the
// failure it prevents.
//
// Interfaces belonging to a function are contiguous, so slicing the buffer to
// [begin,end) scopes BOTH the shared finders and the walk without either
// growing a range parameter or acquiring a second copy of the search.
//
// Returns false when there is no audio-function IAD, and the caller then uses
// the whole buffer -- today's behaviour, kept for descriptor sets that begin
// at the first interface with the IAD already stripped by the caller.
// `nth` selects which audio function (0-based), so the caller can TRY each in
// turn rather than betting on the first. Betting on the first is wrong in an
// ordinary case: a composite device whose capture function is declared before
// its playback function would be rejected for having no output stream, while
// the stream it needs sat in the next function along.
static bool uac2_function_span(const uint8_t *d, size_t len, unsigned nth,
                               size_t *begin, size_t *end)
{
	const uint8_t DT_IAD = 0x0B;
	uint8_t first_if = 0, if_count = 0;
	bool have = false;
	unsigned seen = 0;

	size_t i = 0;
	while (i + 1 < len && d[i] >= 2 && i + d[i] <= len) {
		// bFunctionClass AUDIO + bFunctionProtocol AF_VERSION_02_00.
		// bFunctionSubClass is deliberately NOT checked: UAC2 4.6 defines it
		// as FUNCTION_SUBCLASS_UNDEFINED (0x00) for an audio function, not
		// AUDIOCONTROL. Requiring 0x01 here -- the obvious guess, and the one
		// made first -- matched nothing on the MC200, whose IAD really does
		// carry 0x00. The captured fixture is what caught it.
		if (d[i + 1] == DT_IAD && d[i] >= 8
		    && d[i + 4] == AUDIO_CLASS && d[i + 6] == 0x20) {
			if (seen++ == nth) {
				first_if = d[i + 2];
				if_count = d[i + 3];
				have = true;
				break;
			}
		}
		i += d[i];
	}
	if (!have || if_count == 0) return false;

	size_t b = len, e = len;
	bool started = false;
	i = 0;
	while (i + 1 < len && d[i] >= 2 && i + d[i] <= len) {
		if (d[i + 1] == DT_INTERFACE && d[i] >= 9) {
			uint8_t n = d[i + 2];
			bool member = (n >= first_if) && (n < (uint8_t)(first_if + if_count));
			if (member && !started) { b = i; started = true; }
			else if (!member && started) { e = i; break; }
		}
		i += d[i];
	}
	if (!started) return false;
	*begin = b;
	*end = e;
	return true;
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

void uac2_clock_range_setup(uint8_t setup[8], uint8_t ac_interface,
                            uint8_t clock_id, uint16_t wlen)
{
	setup[0] = 0xA1;              // class, interface recipient, DEVICE-to-host
	setup[1] = 0x02;              // RANGE (UAC2 5.2.1 table 5-4; CUR is 0x01)
	setup[2] = 0x00;
	setup[3] = 0x01;              // CS_SAM_FREQ_CONTROL << 8
	setup[4] = ac_interface;
	setup[5] = clock_id;
	setup[6] = (uint8_t)(wlen & 0xFF);
	setup[7] = (uint8_t)(wlen >> 8);
}

static uint32_t rd32(const uint8_t *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8)
	     | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

uint16_t uac2_range_count(const uint8_t *buf, size_t len)
{
	if (!buf || len < 2) return 0;
	uint16_t declared = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
	// Trust the buffer over the declaration. A device that says six and sends
	// two has been truncated somewhere -- by our own wLength, most likely --
	// and reading the two that arrived is right where believing the header
	// and walking off the end is not.
	size_t fit = (len - 2) / 12;
	return (declared < fit) ? declared : (uint16_t)fit;
}

bool uac2_range_get(const uint8_t *buf, size_t len, uint16_t idx,
                    uint32_t *min, uint32_t *max, uint32_t *res)
{
	if (idx >= uac2_range_count(buf, len)) return false;
	const uint8_t *p = buf + 2 + (size_t)idx * 12;
	if (min) *min = rd32(p);
	if (max) *max = rd32(p + 4);
	if (res) *res = rd32(p + 8);
	return true;
}

bool uac2_range_supports(const uint8_t *buf, size_t len, uint32_t rate)
{
	uint16_t n = uac2_range_count(buf, len);
	for (uint16_t i = 0; i < n; i++) {
		uint32_t lo = 0, hi = 0, step = 0;
		if (!uac2_range_get(buf, len, i, &lo, &hi, &step)) continue;
		if (lo > hi) continue;                 // malformed subrange, skip it
		if (rate < lo || rate > hi) continue;
		if (step == 0) return true;            // discrete point, or continuous
		if (((rate - lo) % step) == 0) return true;
	}
	return false;
}

const char *uac2_parse_result_str(uac2_parse_result r)
{
	switch (r) {
	case UAC2_PARSE_OK:                 return "ok";
	case UAC2_REJECT_BAD_ARGS:          return "bad-args";
	case UAC2_REJECT_NO_OUT_STREAM:     return "no-out-stream";
	case UAC2_REJECT_NO_AC_HEADER:      return "no-ac-header";
	case UAC2_REJECT_NOT_UAC2:          return "not-uac2";
	case UAC2_REJECT_NO_ALTS:           return "no-alts";
	case UAC2_REJECT_NO_TERMINAL_LINK:  return "no-terminal-link";
	case UAC2_REJECT_CLOCK_UNRESOLVED:  return "clock-unresolved";
	}
	return "unknown";
}

static uac2_parse_result parse_one(const uint8_t *desc, size_t len,
                                   UAC1Topology *out);

bool uac2_parse_config(const uint8_t *desc, size_t len, UAC1Topology *out)
{
	return uac2_parse_config_ex(desc, len, out) == UAC2_PARSE_OK;
}

// Try each audio function in declaration order and take the first that yields
// a usable topology. Reported reason on total failure is the FIRST function's,
// not the last: the first is the device's primary audio function and its
// complaint is the one worth printing.
uac2_parse_result uac2_parse_config_ex(const uint8_t *desc, size_t len,
                                       UAC1Topology *out)
{
	if (!desc || !out || len < 9) return UAC2_REJECT_BAD_ARGS;

	uac2_parse_result first = UAC2_PARSE_OK;
	bool any_span = false;

	for (unsigned n = 0; n < UAC2_MAX_FUNCTIONS; n++) {
		size_t b = 0, e = 0;
		if (!uac2_function_span(desc, len, n, &b, &e)) break;
		any_span = true;
		uac2_parse_result r = parse_one(desc + b, e - b, out);
		if (r == UAC2_PARSE_OK) return r;
		if (n == 0) first = r;
	}

	// No IAD at all: a descriptor set already sliced to one function by the
	// caller, which is the claim-time form this parser has always accepted.
	if (!any_span) return parse_one(desc, len, out);

	memset(out, 0, sizeof(*out));
	out->control_interface = 0xFF;
	out->streaming_interface = 0xFF;
	out->input_streaming_interface = 0xFF;
	return first;
}

// One audio function's worth of descriptors, already sliced. Knows nothing
// about IADs -- the caller decides what "one function" means.
static uac2_parse_result parse_one(const uint8_t *desc, size_t len,
                                   UAC1Topology *out)
{
	memset(out, 0, sizeof(*out));
	out->control_interface = 0xFF;
	out->streaming_interface = 0xFF;
	out->input_streaming_interface = 0xFF;

	uint8_t stream_if = uac_find_output_streaming_interface(desc, len);
	uint8_t in_if = uac_find_input_streaming_interface(desc, len);
	out->input_streaming_interface = in_if;
	if (stream_if == 0xFF) return UAC2_REJECT_NO_OUT_STREAM;
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
			} else if (in_control && subtype == AC_CLOCK_MULTIPLIER && l >= 5) {
				// UAC2 4.7.2.9. A multiplier has exactly one upstream clock,
				// bCSourceID at b[4] -- one byte earlier than a selector's
				// first source, because it has no bNrInPins to precede it.
				// So it is unambiguous where a multi-input selector is not,
				// and joins the same one-hop map.
				//
				// It does NOT change the RATE this parser reports. The rate
				// is whatever CS_SAM_FREQ_CONTROL on the resolved source
				// says; the multiplier's numerator/denominator scale the
				// clock the device derives internally, and are not something
				// the host is entitled to infer. Resolving THROUGH one is the
				// whole job -- before this, a device clocked
				// source -> multiplier -> terminal failed to parse at all,
				// which the header documented as "must fail closed".
				if (selector_count < UAC2_MAX_CLOCKS) {
					selectors[selector_count].id  = b[3];
					selectors[selector_count].src = b[4];
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

	// Split deliberately: "no AC header at all" is a different device from
	// "an AC header declaring UAC1". The first is not an audio function we
	// recognise; the second is one we understand and are declining by
	// version. Collapsing them was the taxonomy's motivating example.
	if (!got_header) return UAC2_REJECT_NO_AC_HEADER;
	if (out->bcd_adc != 0x0200) return UAC2_REJECT_NOT_UAC2;
	if (out->alt_count == 0) return UAC2_REJECT_NO_ALTS;
	if (!have_terminal_link) return UAC2_REJECT_NO_TERMINAL_LINK;

	uint8_t cid = resolve_clock(terminal_link, terminals, terminal_count,
	                            selectors, selector_count,
	                            clock_source_ids, clock_source_count);
	if (cid == 0) return UAC2_REJECT_CLOCK_UNRESOLVED;
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

	return UAC2_PARSE_OK;
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
