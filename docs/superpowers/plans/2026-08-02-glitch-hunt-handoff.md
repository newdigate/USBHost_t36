# Handoff: remove the periodic glitch in UAC1 output

Paste this as the opening prompt of the next session.

---

## The task

A UAC1 asynchronous USB audio device, driven by our RT1176 host, produces an
audible click roughly **every 5 seconds** (~0.2/s). Find the cause and remove
it. Everything below is the state as of 2026-08-02; treat the "ruled out"
section as evidence, not opinion, and do not re-run those experiments without
a reason.

## Bench

| Piece | State |
|---|---|
| MIMXRT1170-EVKB | USB host. Runs `examples/usb/usb_audio_graph_test` in the evkb repo. |
| XMOS xcore-200 MC Audio | The device. Reflashed 2026-08-02 with `sw_usb_audio` `app_usb_aud_xk_216_mc`, config `1AMi2o2xxxxxx`, plus `-DSTREAM_FORMAT_OUTPUT_1_RESOLUTION_BITS=16 -DXSCOPE`. UAC1, full speed, stereo 16-bit, asynchronous, 3-byte feedback endpoint on `EP 0x82` (`bRefresh=4`, so 16 ms). Enumerates `20B1:000F`. |
| RME Fireface UFX | Analogue capture, 44.1 kHz native. MC200 line out -> RME ch1/2. |
| XTAG-3 | Adapter id `3LajHPG5`. Free for `xrun --xscope`. |

The MC200's **original 2016-era firmware** is backed up at
`~/Development/xmos/xcore200-mc-ORIGINAL-firmware.bin` (2 MB, md5
`3a262c67f4810d1707b8e31d7c799698`). Its source was never on disk and the
config was customised, so this file is the only copy. Restore with
`xflash --adapter-id 3LajHPG5 --target-file <xn> --write-all <file>`.

## Ruled out, with evidence

**Rate mismatch between host and device.** A/B alternating −56 and +250 ppm
every 30 s *inside one recording* gave 17.36 vs 17.99 events/s — 0.63 ± 0.43,
i.e. 0.6 sigma — where the drift model predicts 13.5. An earlier swept
measurement produced a convincing V-shape minimum at −56 ppm with drift
arithmetic agreeing to one decimal; that was **coincidence**. The sweep is a
sawtooth, so ±250 ppm are adjacent in time at the wrap, making "elevated at
both extremes" indistinguishable from "elevated once per 420 s cycle".

**The Teensy Audio Library integration.** `DRIVE_FROM_TONE 1` removes the
graph, `AudioOutputUSBHost` and its occupancy pacing from the build entirely.
Glitches persisted (5.88 -> 4.38/s). So nothing about *what paces the graph*
is causal — which also answers "should SAI own the clock instead of USB": it
would not help, and it would add host-side FIFO drift we do not currently
have, because both clocks derive from the same RT1176 crystal while the device
runs on its own.

**Host-visible packet loss.** `USBAudioOut` now counts the siTD completion
status it used to discard: `xact=0 babble=0 buf=0 short=0` over 200 s while
the audio showed 801 discontinuities in the same window. `short` is the
subtle one — a partial transfer sets no error bit.

**Host starvation.** `pkts/s=1000`, `dropped=0`, `underruns=0`, FIFO in
envelope, in every configuration all day.

**The device thrashing its buffer.** Device-side xscope probes show
`out_underflow` and `out_overflow` each firing **exactly once**, at stream
start, including at ±150 ppm bias.

## The measurement trap — read this before recording anything

Most of what was measured on 2026-08-02 was the measuring rig.

```
loopback (Mac out -> RME in, device absent) : 2.23 events/s, median 10.9 samples
Mac as host -> MC200 -> RME                 : 2.45 events/s
RT1176 host -> MC200 -> RME                 : 4.12 events/s
```

The instrument floor is ~2.2/s with an *identical* magnitude distribution to
what was attributed to the device. Rules that follow:

1. **Run the loopback control first**, every session. It takes four minutes.
2. **Never compare absolute event rates between recordings.** The capture
   chain's floor moved from ~7 to ~17 events/s between sessions.
3. **Match levels** before comparing — detector sensitivity depends on them.
4. The audible click is **~0.2/s**. Every number measured was 4–6/s. They were
   never the same phenomenon; a rate that does not match the observation is a
   signal that the wrong thing is being measured.

`tools/glitchdetect.py` has `selftest` (9/9, validated against known dropouts
including clipped and noisy variants). Its docstring carries these caveats.

## The one live lead

Under the Mac the chain sits at the instrument floor (2.45 vs 2.23/s); under
our host it is 4.12/s. **Our host appears to contribute ~+1.7 events/s.** This
is one run per configuration and has not been repeated. Repeat it before
building anything on it.

## Immediate next step

The device-side instrument is the best tool available — no analogue path, no
capture clock, no microphone permissions — but its fill probe is **wrong**.

1. **Fix the probe.** `lib_xua/src/core/buffer/decouple/decouple.xc` emits
   `space_left`, a raw pointer difference with a special case that flips sign
   as the ring wraps. It is not a fill level. Emit `(wrptr - rdptr)` wrapped
   into the buffer size instead.
2. **Validate it before trusting it.** Set a known bias, confirm the fill
   drifts at the predicted samples/s. A probe that has not been checked
   against a known input is not evidence.
3. **Re-run the locked-bias sweep.** Harness is `/tmp/driftrun.sh` (recreate
   it; it reflashes a locked bias per point rather than correlating two
   timelines, which is what went wrong earlier). Fit drift against bias: the
   slope **must** come out at 1.000 because the trim is calibrated arithmetic.
   The previous run gave 0.222 with R²=0.276 and non-monotonic points — that
   is how you know the probe is broken, not the device.
4. The zero-drift bias is then the device's true rate, and doubles as ground
   truth for validating the feedback decoder.

Also note `out_underflow` is probably useless: both `outUnderflow = 1` sites
are stream-reset points, not runtime dry-outs. The overflow probe is at a
genuine runtime event.

## The actual fix, once measurement is trustworthy

Read the feedback endpoint. `EP 0x82`, 3 bytes, 10.14 fixed point at FS, every
16 ms per `bRefresh`, and drive `setRateBias()` from it. The transport work is
the real cost: this is an isochronous **IN** on a full-speed device behind the
embedded TT, needing start-split plus complete-splits, and `sitd_fill_out()`
is OUT-only today.

Find the feedback endpoint via `bSynchAddress` on the data endpoint, **not**
the usage-type bits — lib_xua leaves the feedback endpoint's own usage bits at
`00` (data), so scanning for a feedback usage type finds nothing. The parser
already captures `feedback_endpoint` and `feedback_refresh` this way.

## Build and run

```bash
# host tests (355 checks)
cd ~/Development/USBHost_t36/test && make

# evkb firmware
cd ~/Development/rt1170/evkb/examples/usb/usb_audio_graph_test && cmake --build build
cd ~/Development/rt1170/evkb && ./tools/rt1170-flash.sh \
    examples/usb/usb_audio_graph_test/build/usb_audio_graph_test.hex

# XMOS firmware -- BOTH defines are required
cd ~/Development/xmos/sw_usb_audio/app_usb_aud_xk_216_mc
cmake -B build -DPARTIAL_TESTED_CONFIGS=ON \
  -DEXTRA_BUILD_FLAGS="-DSTREAM_FORMAT_OUTPUT_1_RESOLUTION_BITS=16;-DXSCOPE"
make -C build 1AMi2o2xxxxxx -j8
xrun --xscope-file /tmp/xs --id 0 bin/1AMi2o2xxxxxx/*.xe
```

`-fxscope` does **not** define `XSCOPE`; without `-DXSCOPE` every probe
compiles to a no-op and the VCD comes out empty.

Address ffmpeg capture devices **by name**, never by index — indices
re-order when an iPhone or similar appears, and a silent recording is the
result. Microphone access is granted to Terminal; capture from this shell
works.

## Uncommitted

`~/Development/xmos/lib_xua` has the instrumentation (two files, additive).
Vendored dependency, so consider extracting a patch file into the repo rather
than leaving edits in the clone. Probe ids are hardcoded 0/1/2 across two
files and would silently mismatch if `xscope_register` were reordered.
