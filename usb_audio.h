// Copyright (c) 2026 Nicholas Newdigate
// SPDX-License-Identifier: MIT
#ifndef USB_AUDIO_H_
#define USB_AUDIO_H_
#include "USBHost_t36.h"
#include "usb_audio_parse.h"

class USBAudioOut : public USBDriver {
public:
	USBAudioOut(USBHost &host) { init(); }
	USBAudioOut(USBHost *host) { init(); }

	// Requested format. Must be called before the device is attached.
	void format(uint32_t rate, uint8_t channels, uint8_t bits) {
		req_rate = rate; req_channels = channels; req_bits = bits;
	}

	bool ready() const { return active_alt >= 0; }
	const UAC1Topology &topology() const { return topo; }
	int alternateSetting() const { return active_alt; }

protected:
	virtual bool claim(Device_t *device, int type, const uint8_t *descriptors, uint32_t len);
	virtual void disconnect();
	virtual void control(const Transfer_t *transfer);

private:
	void init();

	UAC1Topology topo;
	uint32_t req_rate     = 48000;
	uint8_t  req_channels = 2;
	uint8_t  req_bits     = 16;
	int      active_alt   = -1;
	int      pending_alt  = -1;
	setup_t  setup;   // must outlive the control transfer
};

#endif // USB_AUDIO_H_
