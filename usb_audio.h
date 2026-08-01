// Copyright (c) 2026 Nicholas Newdigate
// SPDX-License-Identifier: MIT
#ifndef USB_AUDIO_H_
#define USB_AUDIO_H_
#include "USBHost_t36.h"
#include "usb_audio_parse.h"
#include "ehci_iso.h"
#include "usb_audio_fifo.h"

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

    // --- continuous streaming (Task 5) ---
    //
    // begin() fills every periodic frame slot with a live siTD so a packet
    // goes out each 1 ms frame. service() re-arms whichever have completed and
    // must be called often enough to keep ahead of the hardware -- a slot is
    // revisited every 32 ms, which is the whole margin.
    bool beginStreaming();
    void stopStreaming();
    void service();                       // call from Task()
    bool streaming() const { return is_streaming; }

    // Packets the controller has actually transmitted. At 48 kHz this must
    // climb by ~1000/second; anything less means frames went out empty. This
    // is the correctness measure for the ring, replacing capture gap analysis.
    uint32_t packetsSent() const { return packets_sent; }
    uint32_t underruns() const { return underrun_count; }

    // --- audio source ---
    //
    // Push interleaved samples (L,R,L,R...). Returns how many were accepted;
    // fewer than asked means the FIFO is full, and the caller decides whether
    // to drop or retry. This is the seam the Audio library adapter writes to.
    uint32_t write(const int16_t *samples, uint32_t count);
    uint32_t available() const;          // free space, in samples

    // Called once per USB frame consumed, from service(). The Audio library
    // adapter uses this to run the graph, which makes the USB frame clock the
    // master -- see the design spec section 8.
    void onFrameConsumed(void (*cb)(void)) { frame_cb = cb; }

    // Built-in test tone. It is a *producer into the same FIFO*, not a
    // separate path: keeping one route to the wire means a genuine underrun
    // shows up as silence and an underrun count, instead of being masked by
    // falling back to generated audio. 0 disables.
    void tone(uint32_t hz) { tone_hz = hz; }

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

    // Streaming ring: one siTD and one payload buffer per periodic slot.
    // 192 bytes = 48 samples of 48 kHz stereo 16-bit = exactly one frame.
    static const uint32_t RING_SLOTS = 32;
    // Sized for the largest frame any supported rate needs. 48 kHz stereo
    // 16-bit is a constant 192; 44.1 kHz alternates 176/180. The headroom
    // covers a device advertising a larger wMaxPacketSize.
    static const uint16_t MAX_FRAME_BYTES = 256;
    sitd_t  *ring[RING_SLOTS] = {};
    uint8_t  ring_buf[RING_SLOTS][MAX_FRAME_BYTES];
    bool     is_streaming   = false;
    uint32_t packets_sent   = 0;
    uint32_t underrun_count = 0;
    uint32_t tone_hz        = 0;
    uint32_t tone_phase     = 0;
    uint8_t  iso_endpoint   = 0;
    uint32_t frame_accum    = 0;   // fractional samples-per-frame carry

    void fillFrame(uint8_t *dst, uint16_t bytes);
    void topUpFromTone();

    usb_audio_fifo_t fifo;
    void (*frame_cb)(void) = 0;
};

#endif // USB_AUDIO_H_
