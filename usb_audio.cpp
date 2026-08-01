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
