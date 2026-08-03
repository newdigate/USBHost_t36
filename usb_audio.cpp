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

	if (is_uac2) {
		// UAC2 control sequence: the sample rate lives on the Clock Source
		// entity (UAC2 5.2.5.1.1), so it must be set with a CUR write before
		// SET_INTERFACE rather than after, unlike UAC1's endpoint SET_CUR.
		uint32_t r = req_rate;
		rate4_buf[0] = (uint8_t)r; rate4_buf[1] = (uint8_t)(r >> 8);
		rate4_buf[2] = (uint8_t)(r >> 16); rate4_buf[3] = (uint8_t)(r >> 24);
		uac2_clock_cur_setup((uint8_t *)&setup, topo.control_interface,
		                     topo.clock_source_id);
		active_alt = -1;
		pending_alt = alt;
		ctrl_state = CTRL_SET_CLOCK;
		if (!queue_Control_Transfer(dev, &setup, rate4_buf, this)) {
			ctrl_state = CTRL_IDLE;
			return false;
		}
		return true;
	}

	pending_alt = alt;
	ctrl_state = CTRL_SET_INTERFACE;
	if (!USBHost::setInterface(dev, setup, topo.streaming_interface, (uint8_t)alt, this)) {
		ctrl_state = CTRL_IDLE;
		return false;
	}
	return true;
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
	err_xact = err_babble = err_buffer = short_sends = 0;
	// Feedback state is deliberately NOT cleared here: like the OUT ring,
	// the feedback descriptors stay linked across a detach and self-heal on
	// re-attach (beginStreaming early-returns while is_streaming, and the
	// re-claimed device's address flows into every re-arm). While the
	// device is absent the reads complete with transaction errors, no
	// report lands, and fb_frames_since ages the last one out -- sizing
	// slews back to nominal + trim on its own.
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
			uac_pack16(ring_buf_hs[slot] + off, staged, frames, 2,
			           ch_total_out, subslot_out);
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
	if (is_streaming) return true;
	if (active_alt < 0 || !device) return false;

	const UAC1AltSetting *alt = findAlt(active_alt);
	if (!alt || alt->endpoint_address == 0) return false;

	// UAC2 streams over iTDs, a different transport with its own sizing
	// state (channels/subslot/max-packet). Dispatch here, with `alt`
	// resolved but before any of the FS-only setup below -- the single-
	// packet test siTD release in particular is FS bookkeeping that must
	// not run against a device that never had one armed.
	if (is_uac2) return beginStreamingHS(alt);

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
	for (uint32_t i = 0; i < RING_SLOTS; i++) {
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
				if (fb_sitd[k]) { sitd_free(fb_sitd[k]); fb_sitd[k] = 0; }
				fb_endpoint = 0;
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

	// The feedback reader is transport-independent in principle, but its
	// siTD splits are FS machinery; the UAC2 feedback endpoint needs an
	// iTD reader (P3). Leave feedback unarmed: open-loop at nominal is
	// this phase's declared operating point and the gate's expected
	// signature (the device drifts at its measured ~-86 ppm).
	//
	// This block must run BEFORE the arming loop: fillFrameHS() sizes its
	// microframes from fb_sizing_mhz, and an unseeded (zero) rate makes
	// every length zero, which itd_fill_out correctly refuses -- the
	// first hardware gate failed exactly there.
	fb_endpoint = 0;
	fb_rate_mhz = 0;
	fb_avg_mhz = 0;
	fb_frames_since = 0xFFFFFF;
	fb_packets = fb_rejects = fb_errors = 0;
	fb_sizing_mhz = effectiveRateMilliHz();

	frame_accum = 0;
	usb_audio_fifo_reset(&fifo);
	topUpFromTone();
	for (uint32_t i = 0; i < RING_SLOTS; i++) {
		ring_hs[i] = itd_alloc();
		if (!ring_hs[i]) { stopStreaming(); return false; }
		fillFrameHS(i);
		if (!itd_fill_out(ring_hs[i], device->address, iso_endpoint,
		                  ring_buf_hs[i], uframe_len[i], alt_mps_hs, false)) {
			stopStreaming(); return false;
		}
		itd_link(periodic_frame_slot(i), ring_hs[i], (uint16_t)i);
	}

	packets_sent = 0;
	underrun_count = 0;
	is_streaming = true;
	return true;
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
	for (uint32_t k = 0; k < FB_SLOTS; k++) {
		if (!fb_sitd[k]) continue;
		sitd_unlink(periodic_frame_slot(fb_sitd[k]->frame), fb_sitd[k]);
		sitd_free(fb_sitd[k]);
		fb_sitd[k] = 0;
	}
	fb_endpoint = 0;
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
	if (!is_streaming) return;

	topUpFromTone();

	if (is_uac2) {
		// Harvest whatever the controller has finished, then refill and
		// re-link -- same shape as the FS loop below, but a whole iTD (up
		// to eight microframe transactions) is one unit of harvest/refill
		// instead of one siTD per packet. The FS feedback-siTD block does
		// not apply here: feedback is deliberately left unarmed for UAC2
		// in this phase (see beginStreamingHS), so this early return also
		// skips it rather than spinning it uselessly over null pointers.
		for (uint32_t i = 0; i < RING_SLOTS; i++) {
			itd_t *n = ring_hs[i];
			if (!n) continue;

			bool done = true;
			for (unsigned k = 0; k < 8 && done; k++) {
				if (!uframe_len[i][k]) continue;
				itd_txn_status_t st;
				itd_get_txn_status(n, k, &st);
				if (st.active) done = false;
			}
			if (!done) continue;         // hardware has not run it yet

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

			// No feedback target in this phase: slew toward nominal + trim
			// only. fb_frames_since is left to free-run (it never resets
			// below FB_FRESH_FRAMES because no feedback report ever lands),
			// which is the correct feedbackFresh() == false signature for
			// an intentionally open-loop transport.
			if (fb_frames_since < 0xFFFFFF) fb_frames_since++;
			// fb_sizing_mhz is seeded by beginStreamingHS() before any
			// descriptor is armed, so it is never zero here; the slew
			// tracks trim changes at the same bounded rate as FS.
			fb_sizing_mhz = uac1_rate_slew(fb_sizing_mhz,
			                               effectiveRateMilliHz(),
			                               FB_SLEW_MHZ_PER_FRAME);

			fillFrameHS(i);
			if (itd_fill_out(n, device->address, iso_endpoint, ring_buf_hs[i],
			                 uframe_len[i], alt_mps_hs, false)) {
				packets_sent++;
				if (frame_cb) frame_cb();
			}
		}
		return;
	}

	// Collect feedback reports before re-arming audio frames, so a report
	// that just landed steers the very next packet sizing below.
	for (uint32_t k = 0; k < FB_SLOTS; k++) {
		sitd_t *s = fb_sitd[k];
		if (!s) continue;

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

	for (uint32_t i = 0; i < RING_SLOTS; i++) {
		sitd_t *s = ring[i];
		if (!s) continue;

		sitd_status_t st;
		sitd_get_status(s, &st);
		if (st.active) continue;             // hardware has not run it yet

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
		if (bytes == 0 || bytes > MAX_FRAME_BYTES) continue;
		fillFrame(ring_buf[i], bytes);
		if (sitd_fill_out(s, device->address, iso_endpoint, 0, 0,
		                  ring_buf[i], bytes, 0, false)) {
			packets_sent++;
			// The USB frame clock is the master: this is what lets the
			// Audio library adapter run its graph at the bus rate rather
			// than a free-running one (design spec section 8).
			if (frame_cb) frame_cb();
		}
	}
}
