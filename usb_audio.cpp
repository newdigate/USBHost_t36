// Copyright (c) 2026 Nicholas Newdigate
// SPDX-License-Identifier: MIT
#include "usb_audio.h"
#include <string.h>

void USBAudioOut::init()
{
	sitd_pool_init();
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
	if (!uac1_parse_config(descriptors, len, &topo)) return false;
	if (topo.bcd_adc != 0x0100) return false;   // UAC1 only

	int alt = uac1_find_alt(&topo, req_rate, req_channels, req_bits);
	if (alt < 0) return false;

	// Do not assign `device` here -- claim_drivers() sets it after we
	// return true.
	active_alt = -1;         // becomes valid once SET_INTERFACE completes
	pending_alt = alt;
	if (!USBHost::setInterface(dev, setup, topo.streaming_interface, (uint8_t)alt, this))
		return false;
	return true;
}

void USBAudioOut::control(const Transfer_t *transfer)
{
	// The only control transfer this driver issues so far is SET_INTERFACE.
	(void)transfer;
	active_alt = pending_alt;
}

void USBAudioOut::disconnect()
{
	active_alt = -1;
	pending_alt = -1;
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
	const UAC1AltSetting *alt = 0;
	for (uint8_t i = 0; i < topo.alt_count; i++) {
		if (topo.alts[i].alternate_setting == (uint8_t)active_alt) {
			alt = &topo.alts[i];
			break;
		}
	}
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

bool USBAudioOut::beginStreaming()
{
	if (is_streaming) return true;
	if (active_alt < 0 || !device) return false;

	const UAC1AltSetting *alt = 0;
	for (uint8_t i = 0; i < topo.alt_count; i++) {
		if (topo.alts[i].alternate_setting == (uint8_t)active_alt) {
			alt = &topo.alts[i];
			break;
		}
	}
	if (!alt || alt->endpoint_address == 0) return false;
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
		uint16_t bytes = uac1_frame_bytes(&frame_accum, req_rate,
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

	for (uint32_t i = 0; i < RING_SLOTS; i++) {
		sitd_t *s = ring[i];
		if (!s) continue;

		sitd_status_t st;
		sitd_get_status(s, &st);
		if (st.active) continue;             // hardware has not run it yet

		// Packet size is not constant at every rate: 44.1 kHz needs 44
		// samples nine frames out of ten and 45 on the tenth, or the
		// stream drifts against the device's clock.
		uint16_t bytes = uac1_frame_bytes(&frame_accum, req_rate,
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
