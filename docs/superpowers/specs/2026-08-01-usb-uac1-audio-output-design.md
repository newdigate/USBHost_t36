# USB Audio Class 1.0 output on the i.MX RT1176

Design for adding USB host audio **playback** to this fork of USBHost_t36,
targeting the NXP MIMXRT1170-EVKB. Capture (microphone) input is explicitly out
of scope and gets its own spec later.

Date: 2026-08-01

---

## 1. Goal

Stream stereo 16-bit audio from the RT1176 to a USB Audio Class 1.0 device
attached to the `USB_OTG2` host port, and expose it to the Teensy Audio Library
fork as an ordinary `AudioStream` sink.

Bring-up runs at **48 kHz**; the shipping target is **44.1 kHz**. Section 6
explains why that order is deliberate rather than incidental.

## 2. Constraints

**Licensing: permissive only, no copyleft.** This library is MIT. Reference
material must be MIT/BSD/Apache. Two consequences:

- The NXP MCUXpresso SDK USB host stack is **BSD-3-Clause** and is the sanctioned
  reference. BSD-3-Clause cannot be relicensed as MIT, so any file derived from
  it retains NXP's copyright notice and SPDX header, and `LICENSE.md` gains a
  section listing those files.
- `A-Dunstan/teensy4_usbhost` is **GPLv3** and is excluded. Do not copy from it
  and **do not read its implementation** — reading a copyleft implementation
  before writing your own creates derivative-work risk that is hard to disprove
  afterwards.

**Hardware.** MIMXRT1170-EVKB, `USB_OTG2` (EHCI instance 1, base `0x4042C000`,
IRQ 135). The i.MX RT1176 `.bss` lands in DTCM, which the USB DMA engine cannot
reach, so all DMA-visible structures must be placed in `USBHOST_DMAMEM` — the
same constraint the mass-storage work already established.

**Split transactions are unavoidable.** UAC1 is a full-speed class, and
full-speed devices on the RT1176 root port are serviced through the ChipIdea
*embedded transaction translator*. The evidence is in this repo:
`allocate_interrupt_pipe_bandwidth()` programs `start_mask`/`complete_mask` with
SSPLIT/CSPLIT semantics for any device with `speed < 2`, and root devices are
created by `new_Device(speed, 0, 0)` — a hub address of 0, which matches
`TTCTRL.TTHA`'s reset value. That is why full-speed keyboards already work.

Therefore full-speed isochronous requires **siTD**, not iTD. This is the harder
of the two descriptor formats and it is the main technical risk in this design.

## 3. Current state

The library has **no isochronous support whatsoever**. `USBHost::new_Pipe()`
handles control (0), bulk (2) and interrupt (3); type 1 is added to neither the
async nor the periodic schedule, so the pipe is created but never serviced — and
isochronous transfers do not use queue heads at all in any case. The interrupt
handler carries an explicit `// TODO: skip past iTD, siTD
when/if we support isochronous` at `ehci.cpp:1321`.

Nothing in the library issues `SET_INTERFACE`, which every audio streaming
interface requires.

## 4. Reference material

**NXP MCUXpresso SDK (BSD-3-Clause), already checked out locally:**

| Path | Use |
|---|---|
| `middleware/usb/host/usb_host_ehci.c` | siTD scheduling reference (5618 lines, 139 siTD references) |
| `middleware/usb/host/class/usb_host_audio.c` | UAC1/UAC2 class driver reference |
| `_boards/evkbmimxrt1170/usb_examples/usb_host_audio_speaker` | Runnable example for this exact board |

**Test device** — the headset captured on 2026-08-01, decoded via
`tools/usbcap.py descriptors`:

- UAC1 (`bcdADC` 1.00), 4 interfaces
- Interface 2 = playback, endpoint `0x04`, **adaptive** sync, no feedback endpoint
- alt 6 = 44100 Hz stereo 16-bit, `wMaxPacketSize` 228
- alt 7 = 48000 Hz stereo 16-bit, `wMaxPacketSize` 248
- Interface 1 = microphone, mono, asynchronous (out of scope)

The adaptive sync type is significant: the device's PLL follows the host, so
**no feedback endpoint is required for playback**. An entire subsystem drops out
of scope compared to an asynchronous device.

**Reference trace** — the same headset driven correctly by a PC, which sets the
bar for the RT1176 implementation:

```
sizes    : 192 B x8871
delivered: 48005.9 Hz effective (100.01% of target)
cadence  : avg 1.0000 ms (min 0.9997, max 1.0003)
gaps     : 0 gap(s) >= 2 ms, ~0 frame(s) with no packet
```

## 5. Decisions

| Decision | Choice | Rationale |
|---|---|---|
| Core placement | USB host and audio graph **both on CM7** | No inter-core boundary, no cross-core cache maintenance, one interrupt priority space |
| Clock master | **USB frame clock** drives `AudioStream::update_all()` | The audio PLL and USB frame clock are independent oscillators; a single master eliminates drift handling entirely |
| Simultaneous SAI output | **Out of scope** | Follows from the clock-master choice; revisit only if an application needs codec and USB output at once |
| Latency budget | **10–30 ms** | Ring depth becomes the tuning knob; see section 7 |
| Integration shape | **Sample FIFO + completion callback** | Keeps USBHost_t36 free of any Audio Library dependency |
| Detach behaviour | **Fallback timer** in the adapter | See section 9 |

## 6. Why 48 kHz first

At 48 kHz every frame carries exactly 48 samples — 192 bytes, constant, forever.
At 44.1 kHz a frame carries 44.1 samples, so packets alternate between 176 and
180 bytes on a 9:1 pattern driven by a fractional accumulator.

Bringing up at 48 kHz means the packet-size scheduler does not exist yet, so any
misbehaviour during siTD bring-up is unambiguously a scheduling bug. It also
allows a direct diff against the reference trace above, and matches the QEMU
`usb-audio` model exactly (which is fixed at 48 kHz and rejects any packet that
is not exactly 192 bytes).

44.1 kHz is the shipping target because the Audio Library fork's `AudioStream`
dispatch and the SAI/audio PLL both run at 44.1 kHz. Sending 48 kHz in
production would mean resampling in a system that is natively 44.1.

## 7. Architecture

Five units across two repositories. Each has a stated boundary.

### 7.1 Alternate-setting support — `enumeration.cpp`, ~80 lines

Parse alternate settings during the descriptor walk; add a `SET_INTERFACE`
helper. Deliberately **not** audio-specific — the library lacks this generally,
and a future UVC or CDC driver needs the same capability.

### 7.2 Isochronous transport — new `ehci_iso.cpp`, ~400–500 lines

Owns the siTD pool in `USBHOST_DMAMEM`, periodic frame-list linking, S-mask and
C-mask budgeting for full-speed OUT, bandwidth accounting extending
`uframe_bandwidth[]`, and ring refill. Also completes the `ehci.cpp:1321` TODO so
the interrupt handler walks past iTD and siTD entries in the periodic list.

A new file rather than growing `ehci.cpp` past 2000 lines, and it quarantines the
BSD-3-Clause derivation in one place instead of smearing it through an MIT file.

*Boundary: knows frames and descriptors. Knows nothing about audio.*

### 7.3 UAC1 class driver — new `usb_audio.cpp`, ~500 lines

`USBAudioOut : public USBDriver`. Claims the AudioControl and AudioStreaming
interfaces, parses the UAC1 descriptor set, selects the alternate setting
matching the requested rate and format, issues `SET_INTERFACE`, and handles
feature-unit volume and mute. Owns the sample FIFO and the frame-completion
callback.

Selection must be **by endpoint direction**, not by taking the first audio
streaming interface found — the test headset exposes both a microphone and a
speaker streaming interface.

Public API:

```
begin(rate, channels)   available()       write(samples, count)
onFrameComplete(cb)     volume(level)     mute(bool)
underruns()             starvations()
```

*Boundary: knows audio formats and rates. Knows nothing about `audio_block_t`
or `AudioStream`.*

### 7.4 `AudioOutputUSBHost` — Audio fork, ~100 lines

An `AudioStream` sink whose `update()` drains 128-sample blocks into the FIFO,
and which registers a frame callback that triggers `software_isr`. This unit is
what makes USB the clock master, mirroring the role `AudioOutputI2S`'s DMA ISR
plays today.

*Boundary: knows the FIFO API and `AudioStream`. Nothing about USB.*

### 7.5 `tools/usbcap.py` — already landed

Becomes the verification instrument for 7.2 and 7.3.

## 8. Data flow and clocking

Per USB frame: the EHCI interrupt fires on completion; `ehci_iso.cpp` retires the
finished siTD, refills its payload buffer from the sample FIFO, relinks it N
frames ahead, then invokes the frame callback.

**The callback does not run the graph every frame.** At 44.1 kHz a 128-sample
block spans 2.902 ms, so the graph runs at 344.5 Hz while frames tick at 1000 Hz.
The callback instead checks FIFO depth and triggers `software_isr` only when
there is room for another block. The graph therefore runs on demand at its
natural rate, with every trigger derived from the USB frame clock.

**Packet size scheduling:**

- 48 kHz — a constant 48 samples, 192 bytes. No scheduler.
- 44.1 kHz — fractional accumulator: add 44100 per frame, emit the integer part,
  carry the remainder. Yields 44 or 45 samples (176 or 180 bytes) in a 9:1
  pattern. Fits comfortably inside the headset's 228-byte `wMaxPacketSize`.

**Memory placement:** the siTD pool and the frame payload buffers live in
`USBHOST_DMAMEM`. The sample FIFO stays in ordinary memory; the refill path
copies 176–192 bytes into the DMAMEM payload buffer, keeping the DMA-visible
footprint to a small fixed ring rather than the whole FIFO.

**Latency budget:**

| Stage | Depth |
|---|---|
| siTD ring lookahead | 12 frames = 12 ms |
| Sample FIFO | ~8 ms |
| AudioStream block | 2.9 ms |
| **Total** | **~23 ms** |

Ring depth is the tuning knob for trading margin against latency later.

## 9. Error handling

**Underrun** (FIFO dry at refill): emit a full-size packet of silence and
increment a counter. Silence rather than a short packet, because the endpoint is
adaptive and a truncated packet would nudge the device's PLL. A counter rather
than a log line, because this runs in ISR context.

**Ring starvation** (ISR late, hardware reaches an unrefilled siTD): the device
re-sends stale data. Detect via the active bit and count it. This counter is the
firmware-side twin of `usbcap.py`'s gap detection, and the two should agree.

**Device detach while streaming.** With USB as clock master, detach removes the
thing driving the graph — `update_all()` stops and every node freezes mid-state.
The adapter therefore starts a ~344.5 Hz fallback timer on detach and stops it on
attach. Roughly 30 lines, and it converts a confusing class of bug into a
non-event for applications that also run MIDI or a UI. The driver must also
unlink its siTDs from the periodic list and release the reserved bandwidth.

**Enumeration failures** (not UAC1, no alternate setting matching the requested
rate, `SET_INTERFACE` stalls): fail cleanly and do not claim the device, so
another driver can try.

**Bandwidth allocation failure:** refuse to start and report. At 192 bytes in a
~1500-byte frame this should not occur, but it must fail loudly rather than
corrupt the periodic schedule.

**siTD transaction errors and babble:** count and continue. Isochronous has no
handshake and therefore no retry; stalling the stream over one bad frame would
turn a click into a dropout.

## 10. Testing

The established discipline in this project is QEMU-plus-hardware gates. **This
feature cannot fully follow it**: QEMU's `ehci_state_fetchsitd()` is a stub that
warns "Skipping active siTD" and drops the transfer. QEMU's `usb-audio` device is
also full-speed, 48 kHz only, playback only, and hard-rejects any packet that is
not exactly 192 bytes.

Units split by where they are testable:

| Unit | Gate |
|---|---|
| Alt-setting / `SET_INTERFACE` (7.1) | QEMU — fits the existing pattern |
| UAC1 descriptor parser (7.3) | Host-side fixture, no hardware |
| siTD transport (7.2) | Hardware only, via `usbcap.py` |
| `AudioOutputUSBHost` (7.4) | Fake FIFO, host-side |

The parser fixture is `test/fixtures/headset_uac1_config.bin` — the real 799-byte
configuration descriptor recovered from the capture. It exercises sixteen
alternate settings, a mixer and an extension unit, and runs anywhere.

Since the Saleae is API-automated, M3 onward can be a scripted hardware gate:
flash, capture, run `usbcap.py iso`, assert.

## 11. Milestones

**M0a — stock NXP example, unmodified.** Build `usb_host_audio_speaker` for
`evkbmimxrt1170`, attach the headset. Pure go/no-go on whether full-speed
isochronous works through the embedded TT. Expect **8 kHz** audio, not 48 kHz:
`audio_speaker.c:594` hardcodes the streaming alternate setting to 1, which on
this headset is 8000 Hz stereo. The 48000 Hz search at line 986 is inside the
UAC2 branch only.

*Done when:* audio is audible at any rate.

**M0b — retarget the example.** Override `CONTROLLER_ID` to
`kUSB_ControllerEhci1` (OTG2, matching this library's target rather than the
default OTG1) and change the streaming alternate setting to 7.

*Done when:* `usbcap.py iso` on the EVKB's own port reports 192-byte packets and
zero gaps. This produces the RT1176-side golden trace.

**M1 — enumeration and alternate settings, no audio.** Units 7.1 and the
descriptor-parsing half of 7.3. Select interface 2, alt 7.

*Done when:* logged topology matches `usbcap.py descriptors`, `SET_INTERFACE`
completes without a stall, and the fixture test passes.

**M2 — siTD plumbing.** Unit 7.2, single-shot.

*Done when:* at least one 192-byte OUT to endpoint 4 appears on the analyzer.

**M3 — continuous ring.** Ring of siTDs with ISR completion and refill; 12
frames by default per the section 8 budget, tunable in the 8–16 range.

*Done when:* zero gaps over 60 s, cadence within the reference envelope.

**M4 — audible test tone.** Synthesized 48 kHz sine straight into the FIFO, no
Audio Library linked.

*Done when:* zero gaps, underrun counter reads 0, clean tone for several minutes.

**M5 — Audio Library integration and 44.1 kHz.** Unit 7.4, switch to alt 6,
implement the fractional accumulator.

*Done when:* zero gaps at 44.1 kHz and the `usbcap.py iso` size histogram shows
176 B and 180 B in roughly a 9:1 ratio.

**M6 — robustness.** Volume and mute via feature unit 22, hot-plug, revert to
alt 0 to release bandwidth on stop, detach fallback timer.

## 12. Risks

**M0 gate: PASSED 2026-08-01.** A full-speed UAC1 headset enumerates when
directly attached to the RT1176 `USB_OTG2` root port. Verified on hardware with
this library's own stack via `rt1176-evkb/examples/usb/usb_audio_uac1_test`,
which reported the full topology — `bcdADC=1.00`, control interface 0, streaming
interface 2, feature unit 22, all eight alternate settings — matching
`tools/usbcap.py descriptors` byte for byte, and accepted `SET_INTERFACE` to
alt 7. Hot unplug/replug recovered cleanly. This also verifies **M1** on target.

The reference manual confirms the premise section 2 infers from the split-mask
code:

- §62.3.1.1 — "supports direct connection of a HS/FS/LS device... the
  transaction translator function... implemented within the DMA and protocol
  engine blocks to support connection to full and low speed devices."
- §62.5.4.1 — "Embedded Transaction Translator — Allows direct attachment of FS
  and LS devices" with no companion controller.
- Table 62-56 — a directly attached FS/LS device uses **HubAddr = 0**, "the
  address of the Root Hub where the bus transitions from HS to FS/LS (ie. Split
  target hub is the root hub)", exactly what `new_Device(speed, 0, 0)` does.

**A trap for the siTD work**, from §62.5.4.1.3: because of the embedded TT the
port-enable bit is *always* set after port reset regardless of the chirp result,
so a spec-conformant EHCI driver wrongly concludes "high speed". The real speed
must come from `PORTSC.PSPD`. This library already does that at `ehci.cpp:478`
(`(USBHS_PORTSC1 >> 26) & 3`) — new code must not regress it.

**Unresolved, not blocking.** NXP's `usb_host_audio_speaker` fails to enumerate
this same headset on this same port (`kUSB_HostEventEnumerationFail`), on both
OTG1 and OTG2. Ruled out by testing: connector choice, board power, OTG adapter,
`USBMODE.SDIS`, and PSPD handling (their stack reads it at
`usb_host_ehci.c:4290`). Since our own stack succeeds, this is a fault in the
NXP example, not a hardware limitation. Its only consequence for this design is
that M0b cannot use the SDK example as the golden-trace source — capture from
`usb_audio_uac1_test` instead.

If M0a fails, check the physical causes first — OTG adapter, VBUS, the headset's
100 mA draw — then try a different UAC1 device. Only if several devices fail on
NXP's own stack should the embedded-TT premise be treated as suspect, at which
point this design needs re-scoping rather than adjusting.

**Passing M0 proves the silicon and the SDK can do this.** It does not prove
USBHost_t36 can; that is a different stack with a different scheduling model.
What it buys is the elimination of an unknown, leaving known engineering work.

**Bandwidth is not a risk.** 192 bytes of a ~1500-byte full-speed frame.

**DMA placement will bite if forgotten** — siTD pool and payload buffers both
need `USBHOST_DMAMEM`.

## 13. Out of scope

Microphone capture (asynchronous, mono on the test device, and requiring
IN-direction split isochronous — its own spec). UAC2 and high-speed devices.
Simultaneous SAI and USB output. Feedback endpoints, which the adaptive test
device does not need. Devices behind an external hub, which add a second
transaction-translator layer.
