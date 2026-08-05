// Copyright (c) 2026 Nicholas Newdigate
// SPDX-License-Identifier: MIT
#include "usb_audio.h"
#include <string.h>

void USBAudioOut::init()
{
	sitd_pool_init();
	itd_pool_init();
	contribute_Transfers(mytransfers, sizeof(mytransfers)/sizeof(Transfer_t));
	driver_ready_for_device(this);
}

// Claims at device level (type 0) so the whole descriptor set is visible at
// once -- the audio topology spans several interfaces and the streaming
// interface must be chosen by endpoint direction.
//
// Note what claim_drivers() passes at type 0: `enumbuf + 9, enumlen - 9`, so
// `descriptors` starts at the FIRST INTERFACE descriptor, not at the
// configuration header. uac1_parse_config() walks descriptors by bLength and
// ignores the configuration descriptor entirely, so it accepts either form --
// which is what the round-trip test in Task 6 pins down.
//
// Known limitation: claiming at device level takes the whole device, so the
// test headset's HID interface (volume keys) will not get its own driver.
// Acceptable for enumeration; revisit when the HID controls are wanted.
bool USBAudioOut::claim(Device_t *dev, int type, const uint8_t *descriptors, uint32_t len)
{
	if (type != 0) return false;

	// Capture the raw descriptors for lastConfig() before any rejection, so
	// unsupported devices can be dumped from loop() and become parser
	// fixtures. memcpy only: this runs in enumeration context, where a
	// Serial print is fatal to the enumeration itself (measured).
	cfg_dump_truncated = len > sizeof(cfg_dump);
	cfg_dump_len = cfg_dump_truncated ? (uint16_t)sizeof(cfg_dump) : (uint16_t)len;
	memcpy(cfg_dump, descriptors, cfg_dump_len);

	is_uac2 = false;
	if (!uac1_parse_config(descriptors, len, &topo) || topo.bcd_adc == 0x0200) {
		// Not parseable as UAC1 (or the header says 2.0): try the UAC2 walk.
		if (!uac2_parse_config(descriptors, len, &topo)) return false;
		// High speed only: FS UAC2 exists in principle but has no witness
		// on this bench and is out of scope (design spec section 2).
		if (dev->speed != 2) return false;   // Device_t::speed: 0=FS 1=LS 2=HS
		is_uac2 = true;
	} else if (topo.bcd_adc != 0x0100) {
		return false;
	}

	// P1 targets the witness's 24-bit alt explicitly: stereo from the graph
	// is packed into the device's native frames (design spec section 1).
	// Broader alt negotiation is P2.
	int alt = is_uac2 ? uac2_find_alt(&topo, 8, 24)
	                  : uac1_find_alt(&topo, req_rate, req_channels, req_bits);
	if (alt < 0) return false;

	// Do not assign `device` here -- claim_drivers() sets it after we
	// return true.
	active_alt = -1;         // becomes valid once configuration completes

	ctrl_attempts = 0;
	return startConfigure(dev, alt);
}

// Issue the first request of the post-claim configuration sequence, and stamp
// the watchdog. Split out of claim() because the watchdog reissues it: a
// request that errors is never reported to this driver at all (see
// usb_audio_ctrl.h), so retrying from the top is the only way back.
bool USBAudioOut::startConfigure(Device_t *dev, int alt)
{
	if (!dev) return false;

	active_alt = -1;
	pending_alt = alt;
	ctrl_started_ms = millis();

	if (is_uac2) {
		// UAC2 control sequence: the sample rate lives on the Clock Source
		// entity (UAC2 5.2.5.1.1), so it must be set with a CUR write before
		// SET_INTERFACE rather than after, unlike UAC1's endpoint SET_CUR.
		uint32_t r = req_rate;
		rate4_buf[0] = (uint8_t)r; rate4_buf[1] = (uint8_t)(r >> 8);
		rate4_buf[2] = (uint8_t)(r >> 16); rate4_buf[3] = (uint8_t)(r >> 24);
		uac2_clock_cur_setup((uint8_t *)&setup, topo.control_interface,
		                     topo.clock_source_id);
		ctrl_state = CTRL_SET_CLOCK;
		if (!queue_Control_Transfer(dev, &setup, rate4_buf, this)) {
			ctrl_state = CTRL_IDLE;
			return false;
		}
		return true;
	}

	ctrl_state = CTRL_SET_INTERFACE;
	if (!USBHost::setInterface(dev, setup, topo.streaming_interface, (uint8_t)alt, this)) {
		ctrl_state = CTRL_IDLE;
		return false;
	}
	return true;
}

// The watchdog itself, polled from service(). Each step of the sequence gets
// its own deadline; expiry means the completion is never coming, because the
// only path back to a driver is a clean completion.
void USBAudioOut::serviceControl()
{
	uac_ctrl_action_t action = uac_ctrl_poll(ctrl_state != CTRL_IDLE,
	                                         ctrl_started_ms, millis(),
	                                         CTRL_TIMEOUT_MS, ctrl_attempts,
	                                         CTRL_MAX_ATTEMPTS);
	if (action == UAC_CTRL_WAIT) return;

	ctrl_timeouts++;
	ctrl_state = CTRL_IDLE;
	if (action == UAC_CTRL_GIVE_UP) return;   // alt stays -1; counter shows why

	ctrl_attempts++;
	// pending_alt is the alt this device was claimed for; the topology it
	// came from is still the live one, so the sequence can simply restart.
	if (!startConfigure(device, pending_alt)) ctrl_queue_fails++;
}

const UAC1AltSetting *USBAudioOut::findAlt(int alt_number) const
{
	if (alt_number < 0) return 0;
	for (uint8_t i = 0; i < topo.alt_count; i++) {
		if (topo.alts[i].alternate_setting == (uint8_t)alt_number)
			return &topo.alts[i];
	}
	return 0;
}

// Class-specific SET_CUR of SAMPLING_FREQ_CONTROL on the streaming endpoint
// (UAC1 5.2.3.2). The rate is three bytes little-endian, not four.
//
// Needed because an alternate setting that offers several rates -- the Jabra
// 0B0E:2301 offers five in one alt -- leaves the device at whatever rate it
// last had. Without this the stream plays at the wrong speed rather than
// failing, which is much harder to notice.
bool USBAudioOut::requestSampleRate(const UAC1AltSetting *alt)
{
	if (!alt || !device) return false;
	rate_buf[0] = (uint8_t)(req_rate & 0xFF);
	rate_buf[1] = (uint8_t)((req_rate >> 8) & 0xFF);
	rate_buf[2] = (uint8_t)((req_rate >> 16) & 0xFF);
	mk_setup(setup, 0x22, 0x01, 0x0100, alt->endpoint_address, 3);
	return queue_Control_Transfer(device, &setup, rate_buf, this);
}

void USBAudioOut::control(const Transfer_t *transfer)
{
	(void)transfer;

	if (ctrl_state == CTRL_SET_CLOCK) {
		// Clock CUR has landed (or at least completed); proceed to
		// SET_INTERFACE regardless, same as the UAC1 path never checks
		// requestSampleRate()'s transfer status beyond queuing it.
		ctrl_state = CTRL_SET_INTERFACE;
		// Each step gets its own deadline: the sequence is only as alive
		// as its outstanding request, and the next one can stall too.
		ctrl_started_ms = millis();
		if (!USBHost::setInterface(device, setup, topo.streaming_interface,
		                           (uint8_t)pending_alt, this)) {
			ctrl_state = CTRL_IDLE;
		}
		return;
	}

	if (ctrl_state == CTRL_SET_INTERFACE) {
		const UAC1AltSetting *alt = findAlt(pending_alt);
		// The UAC1 endpoint SET_CUR must never fire for UAC2 (the rate went
		// to the clock entity already). Today that holds incidentally --
		// UAC2 alts carry ep_controls==0 from memset -- but an explicit
		// guard keeps a future UAC2 bmControls parser from silently
		// reintroducing a bogus request.
		if (!is_uac2 && uac1_alt_needs_rate_request(alt) && requestSampleRate(alt)) {
			ctrl_state = CTRL_SET_RATE;
			ctrl_started_ms = millis();
			return;      // active_alt stays invalid until the rate lands
		}
	}

	// Either the device needed no rate request, or the rate request just
	// completed. Nothing else issues control transfers on this driver.
	ctrl_state = CTRL_IDLE;
	active_alt = pending_alt;
}

void USBAudioOut::disconnect()
{
	active_alt = -1;
	pending_alt = -1;
	is_uac2 = false;
	ctrl_state = CTRL_IDLE;
	// The retry budget belongs to the device that just left; the next one
	// starts with a full allowance.
	ctrl_attempts = 0;
	err_xact = err_babble = err_buffer = short_sends = 0;
	// Feedback state is deliberately NOT cleared here: like the OUT ring,
	// the feedback descriptors stay linked across a detach and self-heal
	// on re-attach (beginStreaming early-returns while is_streaming, and
	// the re-claimed device's address flows into every re-arm). While the
	// device is absent service() pauses entirely -- the framework nulls
	// `device`, so nothing is harvested or re-armed and the rate state is
	// frozen rather than aged. On re-attach harvesting resumes where it
	// stopped; a pre-detach feedback average may count as fresh for up to
	// FB_FRESH_FRAMES more frames, which is benign -- it is the same
	// device's crystal, and the slew re-converges within a second.
	// Re-attaching a device that needs a different transport, alt or
	// geometry does not resume this stream: the next beginStreaming()
	// compares the live device against what the descriptors were armed for
	// and rebuilds instead (which is why a sketch must call it again after
	// a re-attach, as the graph example does).
	memset(&topo, 0, sizeof(topo));
}

void USBAudioOut::fillTestBuffer(uint8_t value)
{
	for (unsigned i = 0; i < sizeof(test_buf); i++) test_buf[i] = value;
}

// Post one isochronous OUT packet. Deliberately single-shot: this is the step
// that proves the siTD path works at all, before any ring exists.
bool USBAudioOut::postTestPacket(uint16_t len)
{
	if (active_alt < 0 || !device) return false;
	if (len == 0 || len > sizeof(test_buf)) return false;

	// Find the alternate setting we activated, for its endpoint address.
	const UAC1AltSetting *alt = findAlt(active_alt);
	if (!alt || alt->endpoint_address == 0) return false;

	if (!test_sitd) {
		test_sitd = sitd_alloc();
		if (!test_sitd) return false;
	}

	// hub_addr and port are 0: the device is attached directly to the root
	// port, so the root hub is the split target (RM Table 62-56).
	if (!sitd_fill_out(test_sitd, device->address,
	                   alt->endpoint_address & 0x0F,
	                   0, 0, test_buf, len, 0, true)) {
		return false;
	}
	test_len = len;

	// Schedule far enough ahead that the controller has not already walked
	// this frame. Two frames is comfortable at a 1 ms frame time.
	// periodic_frame_slot() masks into range, so no wrap handling here --
	// the frame list size is private to ehci.cpp and should stay that way.
	uint32_t frame = periodic_current_frame() + 2;
	sitd_link(periodic_frame_slot(frame), test_sitd, (uint16_t)frame);
	return true;
}

bool USBAudioOut::testPacketStatus(sitd_status_t *out) const
{
	if (!out || !test_sitd) return false;
	sitd_get_status(test_sitd, out);
	return true;
}

// Generate one frame of payload. A sine when a tone is requested, silence
// otherwise. 48 samples of stereo 16-bit little-endian = 192 bytes.
void USBAudioOut::fillFrame(uint8_t *dst, uint16_t bytes)
{
	const uint32_t want = bytes / 2;      // int16 samples across both channels

	// One route to the wire: whatever is in the FIFO. A short read takes
	// nothing -- a partial frame would shift every later sample and the
	// device has no way to resync -- so an underrun is a full frame of
	// silence plus a counter bump. Visible, rather than papered over.
	if (usb_audio_fifo_read(&fifo, (int16_t *)dst, want)) return;

	for (uint32_t i = 0; i < bytes; i++) dst[i] = 0;
	underrun_count++;
}

// Generate one high-speed frame: eight microframe transactions packed into
// ring_buf_hs[slot], sized from the live FIFO with the device's native
// channel count and subslot width. Mirrors fillFrame()'s underrun handling
// per microframe rather than per frame, since each microframe is its own
// transaction on the wire.
void USBAudioOut::fillFrameHS(uint32_t slot)
{
	uint32_t off = 0;
	for (int k = 0; k < 8; k++) {
		uint16_t bytes = uac2_uframe_bytes_mhz(&frame_accum, fb_sizing_mhz,
		                                       ch_total_out, subslot_out);
		if (off + bytes > sizeof(ring_buf_hs[0])) bytes = 0;  // cannot happen past the begin guard; belt+braces
		uframe_len[slot][k] = bytes;
		if (bytes == 0) continue;
		uint32_t frames = bytes / ((uint32_t)ch_total_out * subslot_out);
		int16_t staged[8 * 2];        // up to 8 frames of live stereo per uframe at 48k+slack
		if (frames * 2 <= sizeof(staged) / sizeof(staged[0])
		    && usb_audio_fifo_read(&fifo, staged, frames * 2)) {
			// Pattern mode swaps only the payload: the FIFO read above
			// still paces the stream, so a starved producer underruns
			// exactly as it would with real audio. A geometry the pattern
			// cannot carry falls back to the staged audio and is counted --
			// silently sending non-pattern data would read at the judge as
			// "host not bit-exact", an accusation with the wrong culprit.
			if (!pat_on || !uacv_pack_pattern(ring_buf_hs[slot] + off,
			                                  frames, ch_total_out,
			                                  subslot_out, &pat_lfsr,
			                                  &pat_primed)) {
				if (pat_on) pat_fallbacks++;
				uac_pack16(ring_buf_hs[slot] + off, staged, frames, 2,
				           ch_total_out, subslot_out);
			}
		} else {
			for (uint32_t b = 0; b < bytes; b++) ring_buf_hs[slot][off + b] = 0;
			underrun_count++;
		}
		off += bytes;
	}
}

// Test-tone producer. Feeds the same FIFO an external writer would, so what
// gets exercised is the real streaming path rather than a parallel one.
void USBAudioOut::topUpFromTone()
{
	if (tone_hz == 0) return;

	uint32_t inc = (uint32_t)(((uint64_t)tone_hz << 32) / req_rate);

	while (usb_audio_fifo_free(&fifo) >= 256) {
		int16_t chunk[128];
		for (uint32_t i = 0; i < 128; i += 2) {
			// Cheap triangle: audible, and obviously wrong if the rate
			// handling breaks. The real generator is the Audio library.
			int32_t tri = (int32_t)(tone_phase >> 16) - 32768;
			if (tri < 0) tri = -tri;
			int16_t v = (int16_t)((tri - 16384) * 3 / 2);
			tone_phase += inc;
			chunk[i]     = v;      // left
			chunk[i + 1] = v;      // right
		}
		if (usb_audio_fifo_write(&fifo, chunk, 128) == 0) break;
	}
}

uint32_t USBAudioOut::write(const int16_t *samples, uint32_t count)
{
	return usb_audio_fifo_write(&fifo, samples, count);
}

uint32_t USBAudioOut::available() const
{
	return usb_audio_fifo_free(&fifo);
}

// req_rate scaled by the trim. Signed intermediate: a negative bias must
// subtract, and 64-bit keeps the product exact for any rate this driver can
// negotiate.
uint32_t USBAudioOut::effectiveRateMilliHz() const
{
	int64_t v = (int64_t)req_rate * 1000
	          + ((int64_t)req_rate * rate_bias_ppm) / 1000;
	if (v < 1000) v = 1000;      // never let the trim stall the stream
	return (uint32_t)v;
}

uint32_t USBAudioOut::queued() const
{
	return usb_audio_fifo_used(&fifo);
}

bool USBAudioOut::beginStreaming()
{
	if (is_streaming) {
		// No configured device to compare against -- detached, or a
		// re-claim still working through its control sequence. Coast: the
		// self-heal contract keeps the armed descriptors linked, and
		// service() is paused until the device comes back (disconnect()).
		if (active_alt < 0 || !device) return true;

		// A device is configured, so ask whether the linked descriptors
		// actually fit it. Replugging the SAME device re-parses to the
		// same layout and keeps streaming, which is the whole point of
		// self-heal. Claiming a device that needs the other transport, a
		// different alt, or a different geometry does not: every armed
		// descriptor belongs to the previous device, service() would
		// dispatch on the new is_uac2 and walk the other transport's null
		// arrays forever, and this function used to report success while
		// it happened.
		UACStreamConfig want;
		uac_stream_config(&want, is_uac2, findAlt(active_alt));
		if (uac_stream_config_equal(&armed, &want)) return true;
		stopStreaming();
	}
	if (active_alt < 0 || !device) return false;

	const UAC1AltSetting *alt = findAlt(active_alt);
	if (!alt || alt->endpoint_address == 0) return false;

	// UAC2 streams over iTDs, a different transport with its own sizing
	// state (channels/subslot/max-packet). Dispatch here, with `alt`
	// resolved but before any of the FS-only setup below -- the single-
	// packet test siTD release in particular is FS bookkeeping that must
	// not run against a device that never had one armed.
	if (is_uac2) {
		if (!beginStreamingHS(alt)) return false;
		uac_stream_config(&armed, true, alt);
		return true;
	}

	iso_endpoint = alt->endpoint_address & 0x0F;

	// Release the single-packet test descriptor first. It holds one of the
	// pool's entries and is still linked into a frame, so leaving it would
	// both starve the ring by one slot and double-book the frame it sits in.
	if (test_sitd) {
		sitd_unlink(periodic_frame_slot(test_sitd->frame), test_sitd);
		sitd_free(test_sitd);
		test_sitd = 0;
	}

	// One descriptor per periodic slot. A slot is revisited every
	// RING_SLOTS frames, so anything less transmits in only that fraction
	// of frames rather than continuously.
	for (uint32_t i = 0; i < RING_SLOTS; i++) {
		ring[i] = sitd_alloc();
		if (!ring[i]) {              // out of pool: unwind rather than half-run
			stopStreaming();
			return false;
		}
	}

	frame_accum = 0;
	usb_audio_fifo_reset(&fifo);
	topUpFromTone();
	// Prime in WIRE order, not slot order. The controller starts
	// transmitting at its CURRENT frame position, so filling 0..31 in index
	// order plays the whole first ring revolution rotated -- chunks F..31
	// first, then 0..F-1 -- with a payload seam at the first refill. For
	// audio that is 32 ms of out-of-order samples once per stream start,
	// inaudible and invisible to every counter; under the validator's
	// pattern it is a guaranteed discontinuity, which is how it was found
	// (first cooperative run: first_error_index landed at exactly one ring
	// revolution plus one). +2 is the same not-already-walked margin as
	// postTestPacket(); the last-filled slots wrap to positions the
	// controller just passed, which are not revisited for a revolution.
	// Ring index == frame-list index: RING_SLOTS matches the controller's
	// PERIODIC_LIST_SIZE, which the 0..31 linking below always assumed.
	uint32_t start = periodic_current_frame() + 2;
	for (uint32_t k = 0; k < RING_SLOTS; k++) {
		uint32_t i = (start + k) % RING_SLOTS;
		uint16_t bytes = uac1_frame_bytes_mhz(&frame_accum, effectiveRateMilliHz(),
		                                      req_channels, req_bits / 8);
		if (bytes == 0 || bytes > MAX_FRAME_BYTES) { stopStreaming(); return false; }
		fillFrame(ring_buf[i], bytes);
		// ioc=false: service() polls the status word, so interrupt-on-
		// complete would only add ~1000 IRQ/s into an ISR with no siTD
		// handling.
		if (!sitd_fill_out(ring[i], device->address, iso_endpoint, 0, 0,
		                   ring_buf[i], bytes, 0, false)) {
			stopStreaming();
			return false;
		}
		sitd_link(periodic_frame_slot(i), ring[i], (uint16_t)i);
	}
	ring_next = start % RING_SLOTS;

	// Arm the feedback reader if this alternate setting advertises one.
	// Failure to arm is not failure to stream: the loop just stays open,
	// exactly as it was before feedback support existed.
	fb_endpoint = alt->feedback_endpoint & 0x0F;
	fb_rate_mhz = 0;
	fb_avg_mhz = 0;
	fb_frames_since = 0xFFFFFF;
	fb_packets = fb_rejects = fb_errors = 0;
	fb_sizing_mhz = effectiveRateMilliHz();
	if (fb_endpoint) {
		for (uint32_t k = 0; k < FB_SLOTS; k++) {
			fb_sitd[k] = sitd_alloc();
			if (!fb_sitd[k] ||
			    !sitd_fill_in(fb_sitd[k], device->address, fb_endpoint, 0, 0,
			                  fb_buf[k], sizeof(fb_buf[k]), 0, false)) {
				// This node was never linked (alloc failed, or the fill
				// refused), so free it directly; stopFeedback() then
				// unlinks and frees the slots that DID arm, leaving no
				// descriptor behind pointing at a now-disowned endpoint.
				if (fb_sitd[k]) { sitd_free(fb_sitd[k]); fb_sitd[k] = 0; }
				stopFeedback();
				break;
			}
			// Slots 16 frames apart: each recurs every 32 ms, together
			// they poll at the 16 ms cadence of bRefresh=4.
			uint16_t frame = (uint16_t)(k * 16);
			sitd_link(periodic_frame_slot(frame), fb_sitd[k], frame);
		}
	}

	packets_sent = 0;
	underrun_count = 0;
	uac_stream_config(&armed, false, alt);
	is_streaming = true;
	return true;
}

// High-speed/UAC2 counterpart of beginStreaming(): one iTD per periodic slot
// instead of an siTD, eight microframe transactions per iTD instead of one
// packet per frame. Called with `alt` already resolved and validated by the
// caller (beginStreaming()).
bool USBAudioOut::beginStreamingHS(const UAC1AltSetting *alt)
{
	iso_endpoint = alt->endpoint_address & 0x0F;
	ch_total_out = alt->channels;
	subslot_out  = alt->subframe_size;

	// Guard on the NEGOTIATED rate's per-microframe need, not the
	// advertised ceiling: the witness advertises wMaxPacketSize 800
	// (sized for 192 kHz) and at 44.1 kHz uses at most 192 B of it.
	uint32_t worst = (req_rate / 8000u + 1u) * (uint32_t)ch_total_out * subslot_out;
	if (worst > MAX_UFRAME_BYTES) return false;
	if (ch_total_out < 2 || subslot_out < 2 || subslot_out > 4) return false;
	alt_mps_hs = alt->max_packet_size < MAX_UFRAME_BYTES
	           ? alt->max_packet_size : MAX_UFRAME_BYTES;

	// Reset the rate brain before arming. This block must run BEFORE the
	// arming loop: fillFrameHS() sizes its microframes from fb_sizing_mhz,
	// and an unseeded (zero) rate makes every length zero, which
	// itd_fill_out correctly refuses -- the first hardware gate failed
	// exactly there.
	fb_endpoint = 0;
	fb_rate_mhz = 0;
	fb_avg_mhz = 0;
	fb_frames_since = 0xFFFFFF;
	fb_packets = fb_rejects = fb_errors = 0;
	fb_sizing_mhz = effectiveRateMilliHz();
	fb_mps_hs = 0;

	frame_accum = 0;
	usb_audio_fifo_reset(&fifo);
	// Re-prime the pattern with the ring: the seed must be the first sample
	// on the wire, or the device hunts for a lock target that already went
	// past. A re-begin mid-capture still reads as a discontinuity at the
	// device -- the honest verdict for a stream that genuinely restarted.
	pat_primed = false;
	topUpFromTone();
	// Prime in WIRE order -- same seam as the FS ring, and where the
	// validator's pattern actually caught it: filled 0..31 in index order,
	// the first revolution transmits rotated and the first refill is a
	// payload discontinuity, R7's first_error_index landing at exactly one
	// ring revolution plus one sample.
	uint32_t start = periodic_current_frame() + 2;
	for (uint32_t k = 0; k < RING_SLOTS; k++) {
		uint32_t i = (start + k) % RING_SLOTS;
		ring_hs[i] = itd_alloc();
		if (!ring_hs[i]) { stopStreaming(); return false; }
		fillFrameHS(i);
		if (!itd_fill_out(ring_hs[i], device->address, iso_endpoint,
		                  ring_buf_hs[i], uframe_len[i], alt_mps_hs, false)) {
			stopStreaming(); return false;
		}
		itd_link(periodic_frame_slot(i), ring_hs[i], (uint16_t)i);
	}
	ring_next = start % RING_SLOTS;

	// Arm the feedback reader if this alternate setting advertises one --
	// same contract as the FS arm: failure to arm is not failure to
	// stream, the loop just stays open at nominal + trim. The HS report
	// is 4 bytes of Q16.16 samples-per-microframe on an iso IN endpoint;
	// one iTD transaction in microframe 0 of each polled slot reads it.
	// The witness refreshes every 1 ms (bInterval 4); the 16 ms slot
	// cadence subsamples that, and the EMA's ~128 ms horizon needs no
	// more. An MPS of 0 (malformed descriptor) or beyond the read buffer
	// leaves the loop open rather than arming a read that cannot land.
	if (alt->feedback_endpoint && alt->feedback_max_packet &&
	    alt->feedback_max_packet <= sizeof(fb_buf[0])) {
		fb_endpoint = alt->feedback_endpoint & 0x0F;
		fb_mps_hs = alt->feedback_max_packet;
		for (uint32_t k = 0; k < FB_SLOTS; k++) {
			fb_itd[k] = itd_alloc();
			if (!fb_itd[k] ||
			    !itd_fill_in(fb_itd[k], device->address, fb_endpoint,
			                 fb_buf[k], sizeof(fb_buf[k]), fb_mps_hs,
			                 false)) {
				// Never linked -- free it here, then let stopFeedback()
				// reclaim the slots that already armed (same reasoning
				// as the FS arm above).
				if (fb_itd[k]) { itd_free(fb_itd[k]); fb_itd[k] = 0; }
				stopFeedback();
				break;
			}
			// Slots 16 frames apart, like the FS reader.
			uint16_t frame = (uint16_t)(k * 16);
			itd_link(periodic_frame_slot(frame), fb_itd[k], frame);
		}
	}

	packets_sent = 0;
	underrun_count = 0;
	is_streaming = true;
	return true;
}

// Unlink, free and forget every feedback descriptor, whichever transport
// armed them, and put the pipe back to "none advertised".
//
// Both arm loops use this when a slot fails partway through. Freeing only
// the node that failed would leave the slots already armed LINKED while
// fb_endpoint reads 0, and the harvest keys on the pointer: those leftovers
// would be refilled every pass with endpoint 0 -- the device's control
// endpoint -- for the rest of the stream's life.
//
// Callers must free a node that was allocated but never linked themselves,
// before calling this: a fresh pool entry's `frame` is whatever the previous
// user left, so unlinking one would walk an arbitrary periodic slot.
void USBAudioOut::stopFeedback()
{
	for (uint32_t k = 0; k < FB_SLOTS; k++) {
		if (fb_sitd[k]) {
			sitd_unlink(periodic_frame_slot(fb_sitd[k]->frame), fb_sitd[k]);
			sitd_free(fb_sitd[k]);
			fb_sitd[k] = 0;
		}
		if (fb_itd[k]) {
			itd_unlink(periodic_frame_slot(fb_itd[k]->frame), fb_itd[k]);
			itd_free(fb_itd[k]);
			fb_itd[k] = 0;
		}
	}
	fb_endpoint = 0;
	fb_mps_hs = 0;
}

void USBAudioOut::stopStreaming()
{
	for (uint32_t i = 0; i < RING_SLOTS; i++) {
		if (!ring[i]) continue;
		sitd_unlink(periodic_frame_slot(i), ring[i]);
		sitd_free(ring[i]);
		ring[i] = 0;
	}
	for (uint32_t i = 0; i < RING_SLOTS; i++) {
		if (!ring_hs[i]) continue;
		itd_unlink(periodic_frame_slot(i), ring_hs[i]);
		itd_free(ring_hs[i]);
		ring_hs[i] = 0;
	}
	stopFeedback();
	// Nothing is linked any more, so nothing is armed for: a later
	// beginStreaming() must build from scratch rather than match against
	// what this stream used to be.
	memset(&armed, 0, sizeof(armed));
	is_streaming = false;
}

// Re-arm whatever the controller has finished. Each descriptor stays linked in
// its slot for the lifetime of the stream -- only the payload and the status
// word are rewritten, so there is no unlink/relink churn on the hot path.
//
// Must be called often enough to get round the whole ring within the 32 ms a
// slot takes to come back. Slots not re-armed in time are simply skipped by
// the controller: no packet that frame, which shows up as packetsSent()
// climbing more slowly than 1000/second.
void USBAudioOut::service()
{
	// Before the streaming early-return: a configuration sequence can be
	// outstanding on a device that has never streamed, and that is exactly
	// the case the watchdog exists for.
	serviceControl();

	if (!is_streaming) return;

	// The framework nulls `device` after disconnect() while the self-heal
	// contract deliberately keeps descriptors linked and is_streaming set.
	// Harvesting while absent would re-arm reads through the null device
	// pointer (non-faulting on RT1176 only because address 0 is ITCM);
	// pause instead -- descriptors sit retired until re-claim restores
	// the device and its address flows into the next refill.
	if (!device) return;

	// Never stream to a device whose alternate setting is not selected.
	// `device` alone is not enough: after a re-claim the descriptors are
	// refilled with the NEW device's address, so a stream left armed from
	// the previous device retargets itself at one sitting in alt 0 and
	// blasts isochronous audio at an endpoint that has no bandwidth
	// reserved. Pausing until the sequence completes is what the ring
	// already does at first start; this makes a re-claim behave the same.
	//
	// This was investigated as a cause of the post-re-enumeration control
	// wedge and is NOT that (gating it changed pkts/s from 1000 to 0 and
	// the wedge persisted, measured 2026-08-03). It is kept because
	// sending audio to an unconfigured device is wrong on its own terms.
	if (active_alt < 0) return;

	topUpFromTone();

	if (is_uac2) {
		// Harvest whatever the controller has finished, then refill and
		// re-link -- same shape as the FS loop below, but a whole iTD (up
		// to eight microframe transactions) is one unit of harvest/refill
		// instead of one siTD per packet.

		// Collect feedback reports before re-arming audio frames, so a
		// report that just landed steers the very next microframe sizing
		// below -- same ordering contract as the FS loop.
		for (uint32_t k = 0; k < FB_SLOTS; k++) {
			itd_t *f = fb_itd[k];
			// fb_endpoint is checked as well as the pointer: the two are
			// kept in step by stopFeedback(), and if they ever were not,
			// refilling would re-arm a read against endpoint 0.
			if (!f || !fb_endpoint) continue;

			itd_txn_status_t fst;
			itd_get_txn_status(f, 0, &fst);
			if (fst.active) continue;

			if (fst.err_xact || fst.err_babble || fst.err_buffer) {
				fb_errors++;
			} else {
				// The HS report is exactly 4 bytes of Q16.16; the
				// controller wrote the received count back into the
				// length field. Anything else -- including a zero-
				// length response from a not-yet-armed endpoint --
				// is counted and skipped, never applied.
				uint32_t mhz = (fst.length == 4)
				             ? uac2_feedback_to_mhz(fb_buf[k]) : 0;
				if (mhz && uac1_feedback_plausible(mhz,
				                                   effectiveRateMilliHz())) {
					fb_rate_mhz = mhz;
					// Average before use: same dither rationale
					// as FS (raw-chasing measured +4.8 ppm on
					// this bench).
					fb_avg_mhz = uac1_fb_average(fb_avg_mhz, mhz);
					fb_frames_since = 0;
					fb_packets++;
				} else {
					fb_rejects++;
				}
			}
			itd_fill_in(f, device->address, fb_endpoint,
			            fb_buf[k], sizeof(fb_buf[k]), fb_mps_hs, false);
		}

		// Harvest in WIRE order from ring_next, stopping at the first slot
		// still active: the controller walks the list sequentially, so
		// nothing after it can be done either. The old 0..31 index scan
		// refilled out of wire order whenever completions straddled the
		// ring's wrap -- payload chunks landing rotated on the wire, the
		// same defect as index-order priming, just latency-triggered. A
		// slot that never completes now stalls the ring visibly instead of
		// leaving a silent one-frame hole per revolution; a starved device
		// is a defect someone notices.
		uint32_t serviced = 0;
		while (serviced < RING_SLOTS) {
			uint32_t i = (ring_next + serviced) % RING_SLOTS;
			itd_t *n = ring_hs[i];
			if (!n) break;

			bool done = true;
			for (unsigned k = 0; k < 8 && done; k++) {
				if (!uframe_len[i][k]) continue;
				itd_txn_status_t st;
				itd_get_txn_status(n, k, &st);
				if (st.active) done = false;
			}
			if (!done) break;            // hardware has not run it yet

			// Record what the controller reported about the microframes
			// just finished, before the descriptor is reused -- same
			// rationale as the FS ring's error bookkeeping below.
			for (unsigned k = 0; k < 8; k++) {
				if (!uframe_len[i][k]) continue;
				itd_txn_status_t st;
				itd_get_txn_status(n, k, &st);
				if (st.err_xact)   err_xact++;
				if (st.err_babble) err_babble++;
				if (st.err_buffer) err_buffer++;
			}

			// One frame consumed: age the feedback and take one slew
			// step toward the device's own report when following and
			// fresh, else the manual nominal + trim -- the same target
			// selection as the FS loop, fed by the iTD reader above.
			if (fb_frames_since < 0xFFFFFF) fb_frames_since++;
			uint32_t target = (follow_fb && fb_frames_since < FB_FRESH_FRAMES
			                   && fb_avg_mhz)
			                  ? fb_avg_mhz : effectiveRateMilliHz();
			fb_sizing_mhz = uac1_rate_slew(fb_sizing_mhz, target,
			                               FB_SLEW_MHZ_PER_FRAME);

			fillFrameHS(i);
			if (itd_fill_out(n, device->address, iso_endpoint, ring_buf_hs[i],
			                 uframe_len[i], alt_mps_hs, false)) {
				packets_sent++;
				if (frame_cb) frame_cb();
			}
			serviced++;
		}
		ring_next = (ring_next + serviced) % RING_SLOTS;
		return;
	}

	// Collect feedback reports before re-arming audio frames, so a report
	// that just landed steers the very next packet sizing below.
	for (uint32_t k = 0; k < FB_SLOTS; k++) {
		sitd_t *s = fb_sitd[k];
		// Pointer and endpoint both, as in the HS harvest above.
		if (!s || !fb_endpoint) continue;

		sitd_status_t st;
		sitd_get_status(s, &st);
		if (st.active) continue;

		if (st.err_transaction || st.err_babble || st.err_buffer) {
			fb_errors++;
		} else {
			uint16_t rx = (uint16_t)(sizeof(fb_buf[k]) - st.bytes_left);
			// The FS report is exactly 3 bytes of 10.14. Anything else --
			// including a zero-length response from a not-yet-armed
			// endpoint -- is counted and skipped, never applied.
			uint32_t mhz = (rx == 3) ? uac1_feedback_to_mhz(fb_buf[k]) : 0;
			if (mhz && uac1_feedback_plausible(mhz, effectiveRateMilliHz())) {
				fb_rate_mhz = mhz;
				// Average before use: the raw report dithers between
				// adjacent values, and slewing after the mean rather
				// than the instantaneous report is what nulls the
				// residual (raw-chasing measured +4.8 ppm on this
				// bench).
				fb_avg_mhz = uac1_fb_average(fb_avg_mhz, mhz);
				fb_frames_since = 0;
				fb_packets++;
			} else {
				fb_rejects++;
			}
		}
		sitd_fill_in(s, device->address, fb_endpoint, 0, 0,
		             fb_buf[k], sizeof(fb_buf[k]), 0, false);
	}

	// Wire order with early exit, exactly as the HS loop above -- the FS ring
	// has the same wrap-straddle rotation otherwise.
	uint32_t serviced = 0;
	while (serviced < RING_SLOTS) {
		uint32_t i = (ring_next + serviced) % RING_SLOTS;
		sitd_t *s = ring[i];
		if (!s) break;

		sitd_status_t st;
		sitd_get_status(s, &st);
		if (st.active) break;                // hardware has not run it yet

		// Record what the controller reported about the packet just finished,
		// before the descriptor is reused. A failed split transaction leaves
		// the siTD inactive exactly like a good one, so without this the only
		// symptom is missing audio at the far end.
		if (st.err_transaction) err_xact++;
		if (st.err_babble)      err_babble++;
		if (st.err_buffer)      err_buffer++;
		if (st.bytes_left != 0) short_sends++;

		// One frame consumed: age the feedback and take one slew step of
		// the sizing rate toward the current target. The target is the
		// device's own report when following and fresh, else the manual
		// nominal + trim. Slew rather than jump: ~90 ppm/s keeps a
		// recovering-from-stale transition inaudible, and it converges
		// from a cold start (85 ppm measured on this bench) in about a
		// second.
		if (fb_frames_since < 0xFFFFFF) fb_frames_since++;
		uint32_t target = (follow_fb && fb_frames_since < FB_FRESH_FRAMES
		                   && fb_avg_mhz)
		                  ? fb_avg_mhz : effectiveRateMilliHz();
		fb_sizing_mhz = uac1_rate_slew(fb_sizing_mhz, target,
		                               FB_SLEW_MHZ_PER_FRAME);

		// Packet size is not constant at every rate: 44.1 kHz needs 44
		// samples nine frames out of ten and 45 on the tenth, or the
		// stream drifts against the device's clock.
		uint16_t bytes = uac1_frame_bytes_mhz(&frame_accum, fb_sizing_mhz,
		                                      req_channels, req_bits / 8);
		if (bytes == 0 || bytes > MAX_FRAME_BYTES) { serviced++; continue; }
		fillFrame(ring_buf[i], bytes);
		if (sitd_fill_out(s, device->address, iso_endpoint, 0, 0,
		                  ring_buf[i], bytes, 0, false)) {
			packets_sent++;
			// The USB frame clock is the master: this is what lets the
			// Audio library adapter run its graph at the bus rate rather
			// than a free-running one (design spec section 8).
			if (frame_cb) frame_cb();
		}
		serviced++;
	}
	ring_next = (ring_next + serviced) % RING_SLOTS;
}
