// Copyright (c) 2026 Nicholas Newdigate
// SPDX-License-Identifier: MIT
#ifndef USB_AUDIO_H_
#define USB_AUDIO_H_
#include "USBHost_t36.h"
#include "usb_audio_parse.h"
#include "usb_audio2_parse.h"
#include "usb_audio_feedback.h"
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

    // --- transport errors ---
    //
    // The controller writes completion status back into each siTD, and
    // service() was reading it only to ask "has the hardware run this yet",
    // discarding the error bits. A split transaction that fails still leaves
    // the descriptor inactive, so the packet counter keeps reading a clean
    // 1000/s while audio data is quietly not arriving. These count what was
    // being thrown away.
    //
    // shortSends() is the subtle one: bytes_left non-zero means the controller
    // did not transfer the whole payload, which no error bit reports.
    uint32_t xactErrors() const { return err_xact; }
    uint32_t babbleErrors() const { return err_babble; }
    uint32_t bufferErrors() const { return err_buffer; }
    uint32_t shortSends() const { return short_sends; }
    uint32_t transportErrors() const {
        return err_xact + err_babble + err_buffer + short_sends;
    }

    // --- audio source ---
    //
    // Push interleaved samples (L,R,L,R...). Returns how many were accepted;
    // fewer than asked means the FIFO is full, and the caller decides whether
    // to drop or retry. This is the seam the Audio library adapter writes to.
    uint32_t write(const int16_t *samples, uint32_t count);
    uint32_t available() const;          // free space, in samples
    uint32_t queued() const;             // samples waiting to go out

    // The negotiated format. A graph feeding this must produce at rate(),
    // not at whatever it defaults to -- see AudioOutputUSBHost.
    uint32_t rate() const { return req_rate; }
    uint8_t  channels() const { return req_channels; }

    // --- rate trim ---
    //
    // rate() is what gets negotiated: it selects the alternate setting and is
    // sent in SET_CUR, so it must be a rate the device actually advertises.
    // The rate the packets are *sized* for is a separate thing, because an
    // asynchronous device's converter runs on its own oscillator and is not
    // obliged to agree with ours. Two crystals a couple of hundred ppm apart
    // is ordinary, and the difference accumulates in the device's buffer until
    // it drops or repeats samples -- an audible click every few seconds.
    //
    // setRateBias() trims the sizing rate without touching the negotiated one.
    // Positive sends marginally more samples per second. This is the knob the
    // feedback endpoint will eventually drive; until then it can be swept by
    // ear to find where the device actually is.
    void setRateBias(int32_t ppm) {
        if (ppm >  MAX_RATE_BIAS_PPM) ppm =  MAX_RATE_BIAS_PPM;
        if (ppm < -MAX_RATE_BIAS_PPM) ppm = -MAX_RATE_BIAS_PPM;
        rate_bias_ppm = ppm;
    }
    int32_t rateBiasPpm() const { return rate_bias_ppm; }

    // The rate packets are actually sized for, in millihertz.
    uint32_t effectiveRateMilliHz() const;

    // 1% is far beyond any real crystal error and keeps the arithmetic well
    // inside 32 bits; it exists to stop a typo silently destroying the stream.
    static const int32_t MAX_RATE_BIAS_PPM = 10000;

    // --- feedback endpoint ---
    //
    // An asynchronous device measures its own converter and reports it on a
    // paired isochronous IN endpoint: 3 bytes of 10.14 samples-per-frame
    // every 2^bRefresh ms (UAC1 3.7.2.2).
    // At high speed (UAC2) the same idea is 4 bytes of Q16.16 samples per
    // MICROFRAME on the alt's iso IN endpoint (USB 2.0 5.12.4.2), read by
    // an iTD at the same 16 ms slot cadence; the servo below is shared.
    // Reading the report and sizing packets from it is what closes the rate
    // loop permanently; the manual trim above covers only the crystal offset
    // it was measured at, and this device was measured drifting -85.7 ppm on
    // 2026-08-02 -- a number with no reason to survive temperature or unit
    // swaps.
    //
    // The feedback pipe is armed whenever the active alternate setting
    // advertises one (via bSynchAddress); followFeedback() only controls
    // whether the *sizing* follows it. Measurement stays on either way, so a
    // locked-bias drift run still sees what the device reports.
    void followFeedback(bool on) { follow_fb = on; }
    bool followingFeedback() const { return follow_fb; }

    // Averaged decoded report, in millihertz; 0 before the first one. This
    // is the servo target: devices dither the raw report between adjacent
    // values, so the raw value is only for debugging.
    uint32_t feedbackRateMilliHz() const { return fb_avg_mhz; }
    uint32_t feedbackRawMilliHz() const { return fb_rate_mhz; }
    // The rate packets are currently sized for (the slewed value actually
    // fed to the accumulator; equals effectiveRateMilliHz() when feedback is
    // off or stale).
    uint32_t sizingRateMilliHz() const { return fb_sizing_mhz; }
    uint32_t feedbackPackets() const { return fb_packets; }   // good decodes
    uint32_t feedbackRejects() const { return fb_rejects; }   // wrong size/implausible
    uint32_t feedbackErrors() const { return fb_errors; }     // transport errors
    // Fresh = a good report within the last 250 frames (15+ refresh
    // periods); stale sizing falls back to nominal + trim.
    bool feedbackFresh() const { return fb_frames_since < FB_FRESH_FRAMES; }
    uint8_t feedbackEndpoint() const { return fb_endpoint; }

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
    bool isUAC2() const { return is_uac2; }

    // Raw configuration descriptors of the last device offered to claim(),
    // captured whether or not the claim succeeded. This exists for compat
    // work: a sketch can hex-dump an unrecognised device's descriptors from
    // loop() context and turn them into a parser fixture. claim() only
    // memcpys here -- printing from enumeration context is not survivable
    // (measured: a Serial print inside claim() kills enumeration outright).
    // Truncated captures report the buffer size; check configWasTruncated().
    const uint8_t *lastConfig(uint16_t *len) const {
        if (len) *len = cfg_dump_len;
        return cfg_dump;
    }
    bool configWasTruncated() const { return cfg_dump_truncated; }

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

    // Two-step configuration: SET_INTERFACE always, then SET_CUR
    // SAMPLING_FREQ on devices whose alternate setting does not by itself
    // determine the rate. active_alt only becomes valid once the last step
    // completes, so ready() stays false until the device is really at
    // req_rate.
    enum CtrlState { CTRL_IDLE, CTRL_SET_CLOCK, CTRL_SET_INTERFACE, CTRL_SET_RATE };
    CtrlState ctrl_state = CTRL_IDLE;
    uint8_t  rate_buf[3];   // SET_CUR payload, 24-bit LE rate; needs DMA reach
    bool     is_uac2 = false;
    uint8_t  rate4_buf[4];   // UAC2 clock CUR payload; needs DMA reach like rate_buf

    setup_t  setup;   // must outlive the control transfer
    // queue_Control_Transfer() takes two Transfer_t for a setup-only request
    // and three when there is a data stage. control() runs before the
    // completed transfers are freed (followup_Transfer calls back, the caller
    // frees afterwards), so issuing SET_CUR from the SET_INTERFACE callback
    // has both in flight at once: 2 + 3. The pool is shared across drivers, so
    // undersizing this fails only intermittently, when nothing else has
    // contributed spares.
    Transfer_t mytransfers[5] __attribute__ ((aligned(32)));

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

    // HS/UAC2 ring: one iTD per periodic slot, eight microframe transactions
    // each -- the same 32 ms revolution as the siTD ring. Buffers hold eight
    // microframes at the ceiling; 224 covers 48 kHz x 8ch x 4B subslots
    // ((48000/8000 + 1) * 32). The negotiated rate's need is guarded at
    // beginStreaming; alts/rates needing more are refused there.
    static const uint16_t MAX_UFRAME_BYTES = 224;
    itd_t   *ring_hs[RING_SLOTS] = {};
    uint8_t  ring_buf_hs[RING_SLOTS][8 * MAX_UFRAME_BYTES];
    uint16_t uframe_len[RING_SLOTS][8];
    uint8_t  ch_total_out = 0;    // device channels on the active alt
    uint8_t  subslot_out  = 0;    // bytes per device sample
    uint16_t alt_mps_hs   = 0;    // min(advertised MPS, MAX_UFRAME_BYTES)

    // What the currently linked descriptors were built for. Compared against
    // the live device on every beginStreaming() so a re-claim that needs a
    // different transport, alt or geometry rebuilds instead of silently
    // running the previous device's ring; all-zero while nothing is armed.
    UACStreamConfig armed = {};

    bool     is_streaming   = false;
    uint32_t packets_sent   = 0;
    uint32_t underrun_count = 0;
    uint32_t err_xact       = 0;
    uint32_t err_babble     = 0;
    uint32_t err_buffer     = 0;
    uint32_t short_sends    = 0;
    uint32_t tone_hz        = 0;
    uint32_t tone_phase     = 0;
    uint8_t  iso_endpoint   = 0;
    uint32_t frame_accum    = 0;   // fractional samples-per-frame carry, in mHz
    int32_t  rate_bias_ppm  = 0;

    // Feedback pipe state. Two descriptors 16 frames apart give a 16 ms
    // cadence on a 32-slot periodic list where each slot recurs every
    // 32 ms: bRefresh=4's exact rate at FS, and a 16x subsample of the
    // witness's 1 ms bInterval at HS -- the EMA's ~128 ms horizon needs
    // no faster feed (UAC1 soak: +0.1 ppm at this cadence). 8 bytes of
    // room per read: the FS report is 3 bytes, the HS report 4, and
    // anything else that arrives is counted as a reject rather than a
    // buffer error.
    static const uint32_t FB_SLOTS = 2;
    static const uint32_t FB_FRESH_FRAMES = 250;
    static const uint32_t FB_SLEW_MHZ_PER_FRAME = 4;   // ~90 ppm/s at 44.1k
    sitd_t  *fb_sitd[FB_SLOTS] = {};
    itd_t   *fb_itd[FB_SLOTS] = {};   // HS reader: same slots, iTD transport
    uint16_t fb_mps_hs = 0;           // feedback EP wMaxPacketSize (HS arm/refill)
    uint8_t  fb_buf[FB_SLOTS][8];
    uint8_t  fb_endpoint    = 0;   // 0 = none advertised
    bool     follow_fb      = true;
    uint32_t fb_rate_mhz    = 0;   // last plausible decode (display/debug)
    uint32_t fb_avg_mhz     = 0;   // EMA of decodes -- the servo target
    uint32_t fb_sizing_mhz  = 0;   // slewed rate the accumulator uses
    uint32_t fb_frames_since = 0xFFFFFF;   // frames since last good decode
    uint32_t fb_packets     = 0;
    uint32_t fb_rejects     = 0;
    uint32_t fb_errors      = 0;

    const UAC1AltSetting *findAlt(int alt_number) const;
    bool requestSampleRate(const UAC1AltSetting *alt);
    void fillFrame(uint8_t *dst, uint16_t bytes);
    void fillFrameHS(uint32_t slot);
    bool beginStreamingHS(const UAC1AltSetting *alt);
    void stopFeedback();
    void topUpFromTone();

    // Descriptor capture for lastConfig(). 768 covers every UAC1/UAC2
    // config seen on this bench (a UAC2 8ch topology is ~400 bytes) with
    // headroom; larger sets are truncated and flagged, never overrun.
    uint8_t  cfg_dump[768];
    uint16_t cfg_dump_len = 0;
    bool     cfg_dump_truncated = false;

    usb_audio_fifo_t fifo;
    void (*frame_cb)(void) = 0;
};

#endif // USB_AUDIO_H_
