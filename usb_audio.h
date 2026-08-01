// Copyright (c) 2026 Nicholas Newdigate
// SPDX-License-Identifier: MIT
#ifndef USB_AUDIO_H_
#define USB_AUDIO_H_
#include "USBHost_t36.h"
#include "usb_audio_parse.h"
#include "ehci_iso.h"

class USBAudioOut : public USBDriver {
public:
    USBAudioOut(USBHost &host) { init(); }
    USBAudioOut(USBHost *host) { init(); }

    // Requested format. Must be called before the device is attached; has
    // no effect once a device is already attached.
    void format(uint32_t rate, uint8_t channels, uint8_t bits) {
        req_rate = rate; req_channels = channels; req_bits = bits;
    }

    bool ready() const { return active_alt >= 0; }

    // Task 4 milestone: post ONE isochronous OUT packet of `len` bytes and
    // let the controller run it. Not streaming -- this exists to prove the
    // siTD path end to end. Returns false if no device is ready, the pool is
    // empty, or the packet cannot be scheduled.
    bool postTestPacket(uint16_t len);

    // Completion status written back by the controller. Valid a frame or two
    // after postTestPacket(). Active false with no error bits and bytes_left
    // zero means the packet went out -- this is the primary verification,
    // stronger than a logic capture.
    bool testPacketStatus(sitd_status_t *out) const;

    // Fill the outgoing test buffer with a value, so what lands on the wire
    // is recognisable.
    void fillTestBuffer(uint8_t value);
    // Returned reference points at driver-owned state that disconnect()
    // clears; callers should copy out what they need rather than holding
    // the reference across a disconnect.
    const UAC1Topology &topology() const { return topo; }
    int alternateSetting() const { return active_alt; }

protected:
    virtual bool claim(Device_t *device, int type, const uint8_t *descriptors, uint32_t len);
    virtual void disconnect();
    virtual void control(const Transfer_t *transfer);

private:
    void init();

    UAC1Topology topo = {};
    uint32_t req_rate     = 48000;
    uint8_t  req_channels = 2;
    uint8_t  req_bits     = 16;
    int      active_alt   = -1;
    int      pending_alt  = -1;
    setup_t  setup;   // must outlive the control transfer
    Transfer_t mytransfers[2] __attribute__ ((aligned(32)));

    // Task 4 scaffolding. The whole object is placed in DMAMEM by the sketch
    // (it already had to be, for setup/mytransfers), so this payload buffer
    // is DMA-reachable too -- required on RT1176 where .bss is DTCM.
    sitd_t  *test_sitd = nullptr;
    uint16_t test_len  = 0;
    uint8_t  test_buf[256];
};

#endif // USB_AUDIO_H_
