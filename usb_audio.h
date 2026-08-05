// Copyright (c) 2026 Nicholas Newdigate
// SPDX-License-Identifier: MIT
#ifndef USB_AUDIO_H_
#define USB_AUDIO_H_
#include "USBHost_t36.h"
#include "usb_audio_parse.h"
#include "usb_audio2_parse.h"
#include "usb_audio_feedback.h"
#include "usb_audio_ctrl.h"
#include "ehci_iso.h"
#include "usb_audio_fifo.h"

// One driver, both directions. The name says OUT because that is all it did
// for its first eight months, and renaming it would churn every example and
// gate for no behavioural gain -- but the input path below lives here rather
// than in a USBAudioIn beside it, and that is a decision rather than an
// accident. claim(type=0) takes the WHOLE device, so a second driver would
// simply never be offered one this driver had already claimed; the framework
// gives no way for two drivers to share a device. A duplex device is one
// device, its two directions share a control sequence and (on every device
// seen so far) a converter clock, so they share a driver.
class USBAudioOut : public USBDriver {
public:
    USBAudioOut(USBHost &host) { init(); }
    USBAudioOut(USBHost *host) { init(); }

    // Requested format. Must be called before the device is attached; has
    // no effect once a device is already attached.
    void format(uint32_t rate, uint8_t channels, uint8_t bits) {
        req_rate = rate; req_channels = channels; req_bits = bits;
    }

    // --- input (capture) ---
    //
    // Requesting an input format is what turns the input path on at all:
    // req_in_channels stays 0 until this is called, and every input step --
    // the alt search in claim(), the extra control requests, the IN ring --
    // is gated on it. So a sketch that never calls formatIn() runs exactly
    // the code it ran before input existed.
    //
    // `channels` and `bits` must match an alternate setting the device
    // advertises on its INPUT interface exactly. Duplex does not imply
    // symmetric and the two bench devices both prove it: the MC200 plays
    // 8ch/16 or 8ch/24 and captures 8ch/24 only; the dongle plays 2ch/16 and
    // captures 1ch/16. Asking for the output format on the input interface
    // is the mistake this refuses rather than approximates.
    void formatIn(uint32_t rate, uint8_t channels, uint8_t bits) {
        req_in_rate = rate; req_in_channels = channels; req_in_bits = bits;
    }

    // How many of the device's channels reach the FIFO, taken from the
    // start of each frame. The MC200 sends eight; storing all of them at
    // 44.1 kHz fills the 4096-sample FIFO in 11.6 ms, which is inside the
    // ring's own 32 ms revolution -- so the default of two is not a
    // simplification, it is what keeps the consumer's deadline longer than
    // the producer's. Clamped to the device's channel count at
    // beginRecording().
    void captureChannels(uint8_t n) { req_capture_ch = n ? n : 1; }
    uint8_t captureChannels() const { return capture_ch; }

    bool ready() const { return active_alt >= 0; }

    // Input configured and ready to record. Separate from ready() because
    // the input interface is selected by control requests that run AFTER
    // the output ones complete -- so there is a window where ready() is
    // true and this is not.
    bool readyIn() const { return active_in_alt >= 0; }

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

    // --- capture stream ---
    //
    // beginRecording() arms an IN descriptor in every periodic slot and the
    // device starts filling them; service() harvests. Nothing paces this:
    // an isochronous IN endpoint is a device free-running its own converter
    // (the MC200's declares bmAttributes 0x05, asynchronous), so the host's
    // only job is to consume what arrives before the ring comes round again.
    // A slot not harvested within its 32 ms revolution is overwritten, which
    // shows up as packetsReceived() climbing more slowly than 1000/s.
    bool beginRecording();
    void stopRecording();
    bool recording() const { return is_recording; }

    // Pop up to `count` interleaved int16 samples, captureChannels() wide.
    // Returns how many were taken; fewer than asked means the FIFO ran dry.
    uint32_t read(int16_t *samples, uint32_t count);
    uint32_t recorded() const;           // samples waiting, in int16

    // Frames the controller has actually collected -- 1000/s at high speed,
    // one per iTD, each carrying up to eight microframe transactions.
    uint32_t packetsReceived() const { return packets_recv; }
    // Device samples that arrived and were thrown away because the FIFO was
    // full. This is the consumer being too slow, not a transport fault, and
    // it is counted separately for exactly that reason: an overrun blames
    // the sketch, a transport error blames the wire.
    uint32_t inOverruns() const { return in_overruns; }
    uint32_t inTransportErrors() const { return in_err_xact + in_err_babble + in_err_buffer; }
    // Microframes that completed carrying nothing. Not an error on its own
    // -- at 44.1 kHz the device legitimately sends 5 frames in some
    // microframes and 6 in others, never zero -- but a count that tracks
    // packetsReceived() * 8 means the endpoint is producing silence, which
    // looks identical to working audio in every other counter here.
    uint32_t inEmptyMicroframes() const { return in_empty_uframes; }
    uint32_t inBytes() const { return in_bytes; }     // payload received, total
    uint32_t rateIn() const { return req_in_rate; }
    int inAlternateSetting() const { return active_in_alt; }

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

    // --- cooperative-mode test pattern (UAC validator R7) ---
    //
    // When on, the HS packing path swaps the payload for the validator's
    // LFSR sequence (uacv_pack_pattern); a device built with
    // UACV_COOPERATIVE=1 regenerates it and counts discontinuities, which is
    // the only instrument that can see this host drop or duplicate samples.
    // Everything else is unchanged on purpose: packets are still sized by
    // the feedback servo and the FIFO is still drained sample-for-sample, so
    // a producer must keep it fed exactly as in normal streaming and an
    // underrun still sends silence WITHOUT advancing the pattern -- the
    // device records the discontinuity instead of the host papering over it.
    //
    // The FS path and 16-bit subslots cannot carry the 24-bit sequence;
    // frames that had to fall back to normal packing are counted so a
    // misconfigured bench reads as a climbing number here rather than as a
    // mystery never-locked SKIP in the judge's report.
    void patternMode(bool on) { pat_on = on; pat_primed = false; }
    bool patternModeActive() const { return pat_on; }
    uint32_t patternFallbacks() const { return pat_fallbacks; }

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

    // Where the post-claim configuration sequence has got to. 0 = idle
    // (either never started or finished); non-zero means a control transfer
    // is outstanding. A value that never returns to 0 while alternateSetting()
    // stays -1 is a stuck sequence -- the driver's control() callback only
    // ever fires on CLEAN completion, because the control pipe's error
    // callback belongs to the core's enumeration retry (enumeration.cpp), so
    // a stalled or errored request is never reported to a driver at all.
    uint8_t controlState() const { return (uint8_t)ctrl_state; }

    // How many times the watchdog has had to abandon an outstanding request.
    // Non-zero means the device stalled or ignored a configuration request;
    // climbing without alternateSetting() ever going valid means it never
    // answered at all.
    uint32_t controlTimeouts() const { return ctrl_timeouts; }

    // Retries that could not even be queued. A control transfer needs three
    // Transfer_t entries from a shared pool of about nine (memory.cpp's four
    // plus this driver's five), and a request that never completes holds its
    // entries until the pipe is deleted at disconnect -- there is no cancel.
    // So this climbing means the retry budget outran the pool, which bounds
    // how many times it is safe to retry at all.
    uint32_t controlQueueFails() const { return ctrl_queue_fails; }

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

    // Input request. req_in_channels == 0 means "no input wanted" and is
    // the master switch for every input code path.
    uint32_t req_in_rate     = 0;
    uint8_t  req_in_channels = 0;
    uint8_t  req_in_bits     = 0;
    uint8_t  req_capture_ch  = 2;
    int      active_in_alt   = -1;
    int      pending_in_alt  = -1;

    // Two-step configuration: SET_INTERFACE always, then SET_CUR
    // SAMPLING_FREQ on devices whose alternate setting does not by itself
    // determine the rate. active_alt only becomes valid once the last step
    // completes, so ready() stays false until the device is really at
    // req_rate.
    //
    // The three IN_ states run the same two steps against the input
    // interface, and only after the output ones have finished: they are a
    // continuation of the same sequence rather than a parallel one, because
    // the device has one control endpoint and this driver has one `setup`.
    enum CtrlState { CTRL_IDLE, CTRL_SET_CLOCK, CTRL_SET_INTERFACE, CTRL_SET_RATE,
                     CTRL_SET_IN_CLOCK, CTRL_SET_IN_INTERFACE, CTRL_SET_IN_RATE };
    CtrlState ctrl_state = CTRL_IDLE;

    // Watchdog for the sequence above. A driver is only ever told about a
    // control transfer that completes CLEANLY -- errors go to the core's
    // enumeration retry and never reach the driver that issued the request
    // (see usb_audio_ctrl.h), so without this a stalled request freezes the
    // sequence until the host reboots. Measured on the bench, not deduced.
    //
    // 500 ms is far longer than any control transfer this driver issues and
    // short enough that a device which re-enumerates mid-sequence is back in
    // service inside a couple of seconds; three attempts then gives up rather
    // than hammering a device that is simply not answering.
    static const uint32_t CTRL_TIMEOUT_MS   = 500;
    static const uint8_t  CTRL_MAX_ATTEMPTS = 3;
    uint32_t ctrl_started_ms = 0;   // when the outstanding request was issued
    uint8_t  ctrl_attempts   = 0;   // retries spent on this device
    uint32_t ctrl_timeouts   = 0;   // lifetime count, for the heartbeat
    uint32_t ctrl_queue_fails = 0;  // retries the Transfer_t pool refused
    uint8_t  rate_buf[3];   // SET_CUR payload, 24-bit LE rate; needs DMA reach
    bool     is_uac2 = false;
    uint8_t  rate4_buf[4];   // UAC2 clock CUR payload; needs DMA reach like rate_buf
    // Separate payload buffers for the input requests. The output request
    // has always completed before the input one is issued, so one buffer
    // would serve -- but the watchdog can reissue, and a retry racing a
    // late completion would have two transfers pointing at one buffer.
    uint8_t  rate_in_buf[3];
    uint8_t  rate4_in_buf[4];

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

    // Capture ring, the mirror image of the two above: one descriptor per
    // periodic slot, so a slot is revisited every 32 ms and that is the
    // whole margin service() has to consume it before the device overwrites
    // it. FS uses one siTD per frame, HS one iTD carrying eight microframe
    // transactions of in_stride bytes each.
    //
    // Descriptor budget, and why Stage B needs no pool growth: an IN ring is
    // 32 more descriptors, and ITD_POOL_SIZE 64 is already spent exactly --
    // 32 OUT ring + 32 feedback at 1000 polls/s. Input-only leaves both of
    // those unarmed, so the 32 are there. Running BOTH directions at once
    // needs the pool grown to 96, which is Stage C's problem and is written
    // down in the design spec rather than pre-empted here.
    sitd_t  *in_ring[RING_SLOTS] = {};
    uint8_t  in_buf[RING_SLOTS][MAX_FRAME_BYTES];
    itd_t   *in_ring_hs[RING_SLOTS] = {};
    uint8_t  in_buf_hs[RING_SLOTS][8 * MAX_UFRAME_BYTES];
    uint16_t in_stride    = 0;    // bytes of room per microframe (HS) / frame (FS)
    uint8_t  ch_total_in  = 0;    // device channels on the active input alt
    uint8_t  subslot_in   = 0;    // bytes per device sample
    uint8_t  capture_ch   = 0;    // channels actually unpacked into in_fifo
    uint8_t  in_endpoint  = 0;
    // Wire order, and required for the same reason the HS OUT ring needs it
    // -- more so, because here it is the SAMPLES that would come out
    // rotated. The controller starts wherever its frame pointer happens to
    // be, so slots complete in that rotated order; harvesting by index would
    // splice a revolution of audio out of sequence into the FIFO. The two
    // slots the controller reaches BEFORE the first harvest position are
    // linked disarmed, exactly as the OUT ring's priming margin does, so
    // their first pass carries nothing rather than carrying samples that
    // would be read last.
    uint32_t in_ring_next = 0;
    // Which slots hold a live read. False for the two priming-margin slots
    // on their opening pass, and the harvest must know: an siTD's status
    // word carries the bytes REMAINING, which a completed full read and a
    // never-armed slot both leave at zero. Without this the margin slots
    // would drain a stride of uninitialised buffer into the recording on
    // the first pass, indistinguishable from audio.
    bool     in_armed[RING_SLOTS] = {};
    bool     is_recording = false;
    uint32_t packets_recv     = 0;
    uint32_t in_overruns      = 0;
    uint32_t in_empty_uframes = 0;
    uint32_t in_bytes         = 0;
    uint32_t in_err_xact      = 0;
    uint32_t in_err_babble    = 0;
    uint32_t in_err_buffer    = 0;

    // What the currently linked descriptors were built for. Compared against
    // the live device on every beginStreaming() so a re-claim that needs a
    // different transport, alt or geometry rebuilds instead of silently
    // running the previous device's ring; all-zero while nothing is armed.
    UACStreamConfig armed = {};
    UACStreamConfig armed_in = {};

    // HS RING ONLY. The ring position expected to complete next, in wire
    // order: the controller walks the periodic list from wherever it happens
    // to be at beginStreaming() -- not from slot 0 -- so slots complete in
    // that rotated order, and priming and refilling must both follow it or
    // the payload is stitched together rotated. The cooperative pattern
    // measured exactly that on this ring (R7 first_error_index at one ring
    // revolution) and measured it green afterwards.
    //
    // The FS ring deliberately does NOT use this. The same change there
    // stopped the stream dead on silicon and was reverted -- see the note in
    // beginStreaming().
    uint32_t ring_next = 0;

    bool     is_streaming   = false;
    uint32_t packets_sent   = 0;
    uint32_t underrun_count = 0;
    uint32_t err_xact       = 0;
    uint32_t err_babble     = 0;
    uint32_t err_buffer     = 0;
    uint32_t short_sends    = 0;
    uint32_t tone_hz        = 0;
    uint32_t tone_phase     = 0;
    bool     pat_on         = false;
    bool     pat_primed     = false;
    uint32_t pat_lfsr       = 0;
    uint32_t pat_fallbacks  = 0;
    uint8_t  iso_endpoint   = 0;
    uint32_t frame_accum    = 0;   // fractional samples-per-frame carry, in mHz
    int32_t  rate_bias_ppm  = 0;

    // Feedback pipe state. FS: two descriptors 16 frames apart on the
    // 32-slot periodic list = 62.5 polls/s -- bRefresh=4's exact rate, so
    // every report is read once. HS: one descriptor per frame = 1000
    // polls/s, the device's own refresh rate, for the same reason. This is
    // not about bandwidth: the report dithers between adjacent quanta and
    // the information is the dither's DUTY CYCLE, so a host that
    // subsamples aliases the duty against its poll phase and integrates
    // the resulting slow wander into the device's FIFO. Measured at
    // 250 polls/s: the entire >10 s fill-drift band, predicted from the
    // sampled reports with correlation +0.91, invariant across filter
    // horizons -- no filter fixes sampling (transcript_uacv_servo_
    // isolation.txt). Reading every report is the fix; 32 slots plus the
    // 32-slot OUT ring drain ITD_POOL_SIZE (64) to exactly zero.
    // The EMA divisor scales with the poll rate to hold the ~128 ms
    // horizon (see uac1_fb_average): more reports into the same filter,
    // never a shorter filter.
    //
    // 8 bytes of room per read: the FS report is 3 bytes, the HS report
    // 4, and anything else that arrives is counted as a reject rather
    // than a buffer error.
    static const uint32_t FB_SLOTS = 2;          // FS reader
    static const uint32_t FB_SLOTS_HS = 32;      // HS reader: every frame
    static const uint32_t FB_SLOTS_MAX = 32;     // array sizing
    static const uint32_t FB_EMA_DIV = 8;        // 1/8 at 62.5 polls/s
    static const uint32_t FB_EMA_DIV_HS = 128;   // 1/128 at 1000 polls/s
    static const uint32_t FB_FRESH_FRAMES = 250;
    static const uint32_t FB_SLEW_MHZ_PER_FRAME = 4;   // ~90 ppm/s at 44.1k
    sitd_t  *fb_sitd[FB_SLOTS_MAX] = {};
    itd_t   *fb_itd[FB_SLOTS_MAX] = {};   // HS reader: iTD transport
    uint16_t fb_mps_hs = 0;           // feedback EP wMaxPacketSize (HS arm/refill)
    uint8_t  fb_buf[FB_SLOTS_MAX][8];
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
    const UAC1AltSetting *findInAlt(int alt_number) const;
    bool requestSampleRateOn(const UAC1AltSetting *alt, uint32_t rate, uint8_t *buf3);
    void fillFrame(uint8_t *dst, uint16_t bytes);
    void fillFrameHS(uint32_t slot);
    bool beginStreamingHS(const UAC1AltSetting *alt);
    void stopFeedback();
    bool startConfigure(Device_t *dev, int alt);
    bool startConfigureIn();
    void serviceControl();
    void serviceRecording();
    uint32_t drainInFrames(const uint8_t *src, uint32_t bytes);
    void topUpFromTone();

    // Descriptor capture for lastConfig(). 768 covers every UAC1/UAC2
    // config seen on this bench (a UAC2 8ch topology is ~400 bytes) with
    // headroom; larger sets are truncated and flagged, never overrun.
    uint8_t  cfg_dump[768];
    uint16_t cfg_dump_len = 0;
    bool     cfg_dump_truncated = false;

    usb_audio_fifo_t fifo;
    usb_audio_fifo_t in_fifo;
    void (*frame_cb)(void) = 0;
};

#endif // USB_AUDIO_H_
