// Copyright (c) 2026 Nicholas Newdigate
// SPDX-License-Identifier: MIT
#include "usb_audio.h"
#include <string.h>

void USBAudioOut::init()
{
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
	USBHost::setInterface(dev, setup, topo.streaming_interface, (uint8_t)alt, this);
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
