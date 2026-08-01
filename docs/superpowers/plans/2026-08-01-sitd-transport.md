# siTD Isochronous Transport Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Deliver full-speed isochronous OUT packets from the RT1176 to a UAC1 device attached to the `USB_OTG2` root port, using split-transaction descriptors (siTD), at a steady 1 ms cadence with no missed frames.

**Architecture:** A new `ehci_iso.cpp` owns the siTD pool, periodic frame-list linking and split-transaction budgeting, adapted from NXP's BSD-3-Clause `usb_host_ehci.c`. It exposes a small interface to `usb_audio.cpp` and knows nothing about audio. Verification is layered: struct layout and mask budgeting are pure functions tested on the host; everything downstream is measured on hardware with `tools/usbcap.py`.

**Tech Stack:** C++11, ARM GCC 10, the existing host test harness in `test/`, and a Saleae logic analyser driven through `tools/usbcap.py`.

---

## Prerequisites

**The M0 gate has passed** (spec section 12, 2026-08-01). Full-speed devices
enumerate directly on the RT1176 root port through the embedded transaction
translator; the reference manual confirms `HubAddr = 0` for a directly attached
device. This plan may proceed.

**M0b — golden trace — is still outstanding and blocks Task 6 only.** Capture
`usb_audio_uac1_test` streaming and store the reference figures. Note the SDK
example cannot be used for this: it fails to enumerate the test headset (spec
section 12).

> **The analyser is unreliable on this link — verify from the hardware
> instead, 2026-08-01.**
>
> Probing the EVKB link yields 95–97% `Error packet` frames, while the *same
> rig, leads and ground* captures the same headset against a Mac at **0%
> errors** (`Untitled.csv`, `capture-pre-isoaudio-4-headset-to-mac.csv`).
>
> Eliminated by testing: sample rate (identical failure at 500 MS/s, 41
> samples/bit), breakout hardware, solderless-breadboard connections (rebuilt
> soldered, no change), ground reference, analyser configuration (confirmed a
> single USB LS/FS analyser, D+ and D− on channels 0/1, 3.3 V, Full Speed),
> and a single bad channel (the glitching appears on both lines).
>
> The decoder is not mis-configured: clean packets decode exactly right (SYNC
> then `0x5A` = NAK), and the errors sit on the 12 Mbit/s bit grid — real
> edges the decoder cannot frame, rather than analog mush. Whatever the cause,
> captures from this link are best-effort.
>
> **Consequence for this plan: do not put the analyser on the critical path.**
> The EHCI controller writes completion status back into each siTD — Active
> clears, the error bits report transaction/babble/buffer faults, and
> bytes-to-transfer counts down. That is the controller reporting what it
> actually did, which is stronger evidence than a marginal capture. And a UAC1
> headset that receives audio plays it, so an audible tone proves the whole
> path end to end.
>
> Verification order for Tasks 4–6, strongest first:
>
> 1. siTD writeback: Active cleared, no error bits, bytes-to-transfer at 0
> 2. Firmware starvation and underrun counters
> 3. Audible tone from the headset
> 4. `usbcap.py` cadence and gap analysis — best-effort, not a gate
>
> The starvation counter measures the same property gap detection was meant to
> prove, from inside the firmware, and is trustworthy here where the capture
> is not.
>
> Tasks 1–5 do not depend on this.

**Read before starting:**
- `docs/superpowers/specs/2026-08-01-usb-uac1-audio-output-design.md` sections 2,
  7.2, 8 and 12
- EHCI 1.0 §3.4 (siTD data structure) and §4.12 (split transaction scheduling)
- NXP `middleware/usb/host/usb_host_ehci.c` — `USB_HostEhciLinkSitd`,
  `USB_HostEhciSitdArrayInit`, and the `uframeSmask`/`uframeCmask` computation
  around lines 1540–1730 — plus the `EHCI_HOST_SITD_*` shift constants in
  `usb_host_ehci.h`

## Licensing

`ehci_iso.cpp` is **derived from NXP's BSD-3-Clause code**. It must carry NXP's
copyright notice and `SPDX-License-Identifier: BSD-3-Clause`, not the MIT header
used elsewhere. `LICENSE.md` gains a section listing it. BSD-3-Clause cannot be
relicensed as MIT; keeping the derivation in exactly one file is why the spec
puts it in its own translation unit.

Do not consult `A-Dunstan/teensy4_usbhost` — it is GPLv3.

## Two traps that will cost you a day each

**Port speed.** RM §62.5.4.1.3: because of the embedded TT, the port-enable bit
is *always* set after port reset regardless of the chirp result. A
spec-conformant EHCI driver therefore concludes "high speed" for every device.
Real speed comes from `PORTSC.PSPD`. `ehci.cpp:478` already does this — do not
regress it, and do not add any new "port enabled implies high speed" logic.

**DMA reachability.** On RT1176 plain `.bss` is DTCM, which the EHCI DMA master
cannot reach. Every siTD and every payload buffer must be in `USBHOST_DMAMEM`
and 32-byte aligned. This is the same trap the mass-storage work hit; symptoms
are silent non-transfers rather than faults.

## File Structure

| File | Responsibility |
|---|---|
| `ehci_iso.h` (create) | siTD struct, pool API, mask-budgeting API. Plain types so the pure parts are host-testable. |
| `ehci_iso.cpp` (create) | Pool, periodic-list linking, budgeting, ring refill. BSD-3-Clause. |
| `test/test_sitd.cpp` (create) | Host tests for struct layout and mask budgeting. |
| `test/Makefile` (modify) | Add the second test binary. |
| `ehci.cpp` (modify) | Interrupt handler walks past iTD/siTD in the periodic list — the `ehci.cpp:1321` TODO. |
| `usb_audio.cpp` (modify) | Open the iso pipe once `SET_INTERFACE` completes; feed the ring. |

---

## Task 1: siTD structure and DMAMEM pool

The siTD is seven 32-bit hardware fields plus software bookkeeping, 32-byte
aligned. Getting the layout wrong produces silent non-transfers, so pin it with
static assertions rather than trusting the compiler.

**Files:**
- Create: `ehci_iso.h`
- Create: `ehci_iso.cpp`
- Create: `test/test_sitd.cpp`
- Modify: `test/Makefile`

- [ ] **Step 1: Write the failing layout test**

Create `test/test_sitd.cpp`. It reuses the CHECK macros pattern from
`test_uac1_parse.cpp` — copy them rather than sharing a header, since the two
binaries are independent and the macros are six lines.

```cpp
// Copyright (c) 2026 Nicholas Newdigate
// SPDX-License-Identifier: MIT
#include "ehci_iso.h"
#include <stdio.h>

static int failures = 0, checks = 0;
#define CHECK_EQ(a, b) do { checks++; long _ck_a = (long)(a), _ck_b = (long)(b); \
	if (_ck_a != _ck_b) { printf("FAIL %s:%d: %s == %s (got %ld, want %ld)\n", \
	__FILE__, __LINE__, #a, #b, _ck_a, _ck_b); failures++; } } while (0)

static void test_sitd_layout(void)
{
	// EHCI 1.0 section 3.4: seven consecutive 32-bit hardware fields.
	CHECK_EQ(offsetof(sitd_t, next),        0);
	CHECK_EQ(offsetof(sitd_t, ep_char),     4);
	CHECK_EQ(offsetof(sitd_t, uframe_mask), 8);
	CHECK_EQ(offsetof(sitd_t, status),      12);
	CHECK_EQ(offsetof(sitd_t, buf0),        16);
	CHECK_EQ(offsetof(sitd_t, buf1),        20);
	CHECK_EQ(offsetof(sitd_t, back),        24);
	// Hardware requires 32-byte alignment; the software tail must not push
	// the struct past a 32-byte multiple.
	CHECK_EQ(sizeof(sitd_t) % 32, 0);
}

int main(void)
{
	test_sitd_layout();
	printf("%d checks, %d failures\n", checks, failures);
	return failures ? 1 : 0;
}
```

- [ ] **Step 2: Run to verify it fails**

Run: `cd test && make test_sitd`
Expected: compile error — `ehci_iso.h` does not exist.

- [ ] **Step 3: Write the header**

Create `ehci_iso.h`. Note this header is deliberately free of Arduino and
USBHost_t36 dependencies so the layout and budgeting tests compile on the host.

```cpp
/*
 * Copyright 2026 Nicholas Newdigate
 * Portions derived from the MCUXpresso SDK USB host stack:
 *   Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
 *   Copyright 2016, 2019 - 2020 NXP
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef EHCI_ISO_H_
#define EHCI_ISO_H_
#include <stdint.h>
#include <stddef.h>

// EHCI 1.0 section 3.4. Seven hardware dwords followed by driver bookkeeping.
// Hardware requires 32-byte alignment, so the whole struct is padded to a
// 32-byte multiple.
typedef struct sitd_struct {
	uint32_t next;         // next link pointer + type
	uint32_t ep_char;      // dir, port, hub addr, endpoint, device address
	uint32_t uframe_mask;  // C-mask (bits 15:8), S-mask (bits 7:0)
	uint32_t status;       // status, C-prog-mask, bytes to transfer, IOC
	uint32_t buf0;         // page 0 + current offset
	uint32_t buf1;         // page 1 + transaction position/count
	uint32_t back;         // back pointer
	// --- software bookkeeping, not read by hardware ---
	struct sitd_struct *next_free;
	uint16_t frame;        // frame index this siTD is linked into
	uint16_t reserved;
	uint32_t pad[6];       // pad to 64 bytes (a 32-byte multiple)
} sitd_t;

// Field shifts, matching EHCI 1.0 section 3.4 and NXP's EHCI_HOST_SITD_*.
#define SITD_PTR_TYPE_SITD    0x04u
#define SITD_DIRECTION_SHIFT  31u
#define SITD_PORT_SHIFT       24u
#define SITD_HUB_ADDR_SHIFT   16u
#define SITD_ENDPT_SHIFT      8u
#define SITD_DEV_ADDR_SHIFT   0u
#define SITD_CMASK_SHIFT      8u
#define SITD_SMASK_SHIFT      0u
#define SITD_STATUS_ACTIVE    0x80u
#define SITD_TOTAL_BYTES_SHIFT 16u
#define SITD_IOC_SHIFT        31u

// Split-transaction budgeting for a full-speed isochronous OUT endpoint.
// Returns false if the packet cannot be scheduled. On success writes the
// start-split and complete-split masks.
bool sitd_budget_out(uint16_t max_packet, uint8_t start_uframe,
                     uint8_t *smask, uint8_t *cmask);

#endif
```

- [ ] **Step 4: Write a stub implementation and add the test target**

Create `ehci_iso.cpp` with the same BSD-3-Clause header block, containing only:

```cpp
#include "ehci_iso.h"

bool sitd_budget_out(uint16_t max_packet, uint8_t start_uframe,
                     uint8_t *smask, uint8_t *cmask)
{
	(void)max_packet; (void)start_uframe; (void)smask; (void)cmask;
	return false;
}
```

In `test/Makefile`, add alongside the existing target:

```make
test_sitd: test_sitd.cpp ../ehci_iso.cpp ../ehci_iso.h
	$(CXX) $(CXXFLAGS) -o $@ test_sitd.cpp ../ehci_iso.cpp
```

and extend `run` to execute both binaries, and `clean` to remove both plus their
`.dSYM` bundles.

- [ ] **Step 5: Run to verify it passes**

Run: `cd test && make`
Expected: both binaries build under `-Werror` and report `0 failures`.

- [ ] **Step 6: Commit**

```bash
git add ehci_iso.h ehci_iso.cpp test/test_sitd.cpp test/Makefile
git commit -m "feat: siTD structure with layout pinned by host tests

Seven hardware dwords per EHCI 1.0 section 3.4 plus driver bookkeeping,
padded to a 32-byte multiple. Layout errors here produce silent
non-transfers, so the offsets are asserted rather than assumed."
```

---

## Task 2: Split-transaction budgeting

This is the part most likely to be wrong and the last part that is testable off
hardware, so it gets its own task and real tests.

For a full-speed isochronous **OUT**, the start-splits carry the payload. A
full-speed frame carries at most 188 bytes per microframe, so a packet spanning
N microframes needs N consecutive S-mask bits starting at `start_uframe`. An
isochronous OUT takes **no** complete-splits, so `cmask` is zero — this is the
asymmetry with interrupt endpoints, where `allocate_interrupt_pipe_bandwidth`
in `ehci.cpp` sets both masks.

**Files:**
- Modify: `ehci_iso.cpp`
- Modify: `test/test_sitd.cpp`

- [ ] **Step 1: Write the failing tests**

Add to `test/test_sitd.cpp`, and call from `main`:

```cpp
static void test_budget_out(void)
{
	uint8_t s = 0xAA, c = 0xAA;

	// 192 bytes at 48 kHz spans two microframes (188 + 4).
	CHECK_EQ(sitd_budget_out(192, 0, &s, &c), true);
	CHECK_EQ(s, 0x03);   // uframes 0 and 1
	CHECK_EQ(c, 0x00);   // isochronous OUT takes no complete-splits

	// 180 bytes (44.1 kHz worst case) fits in one microframe.
	CHECK_EQ(sitd_budget_out(180, 0, &s, &c), true);
	CHECK_EQ(s, 0x01);
	CHECK_EQ(c, 0x00);

	// Same packet started later shifts the mask.
	CHECK_EQ(sitd_budget_out(180, 3, &s, &c), true);
	CHECK_EQ(s, 0x08);

	// A full-speed frame is 1023 bytes max for isochronous; beyond that,
	// and beyond the end of the frame, must fail rather than wrap.
	CHECK_EQ(sitd_budget_out(1024, 0, &s, &c), false);
	CHECK_EQ(sitd_budget_out(192, 7, &s, &c), false);  // would need uframe 8
	CHECK_EQ(sitd_budget_out(192, 0, 0, &c), false);   // null out-param
}
```

- [ ] **Step 2: Run to verify it fails**

Run: `cd test && make`
Expected: FAIL on the first `sitd_budget_out(...) == true` — the stub returns false.

- [ ] **Step 3: Implement the budgeting**

Replace the stub in `ehci_iso.cpp`:

```cpp
// A full-speed bus carries at most 188 bytes of isochronous payload per
// microframe. EHCI 1.0 section 4.12.3.
#define FS_BYTES_PER_UFRAME 188u
#define FS_ISO_MAX_PACKET   1023u

bool sitd_budget_out(uint16_t max_packet, uint8_t start_uframe,
                     uint8_t *smask, uint8_t *cmask)
{
	if (!smask || !cmask) return false;
	if (max_packet == 0 || max_packet > FS_ISO_MAX_PACKET) return false;
	if (start_uframe > 7) return false;

	uint32_t uframes = (max_packet + FS_BYTES_PER_UFRAME - 1) / FS_BYTES_PER_UFRAME;
	if (start_uframe + uframes > 8) return false;   // must not wrap the frame

	uint8_t mask = 0;
	for (uint32_t k = 0; k < uframes; k++) mask |= (uint8_t)(1u << (start_uframe + k));

	*smask = mask;
	*cmask = 0;   // isochronous OUT: start-splits carry the data, no CSPLITs
	return true;
}
```

- [ ] **Step 4: Run to verify it passes**

Run: `cd test && make`
Expected: `0 failures` from both binaries.

- [ ] **Step 5: Commit**

```bash
git add ehci_iso.cpp test/test_sitd.cpp
git commit -m "feat: split-transaction budgeting for full-speed isochronous OUT

Start-splits carry the payload at 188 bytes per microframe; an
isochronous OUT takes no complete-splits, unlike the interrupt path in
allocate_interrupt_pipe_bandwidth."
```

---

## Task 3: DMAMEM pool and periodic-list linking

From here verification moves to hardware — there is no way to test frame-list
linking on the host.

**Files:**
- Modify: `ehci_iso.h`, `ehci_iso.cpp`
- Modify: `ehci.cpp`

- [ ] **Step 1: Add the pool**

In `ehci_iso.cpp`, add a fixed pool. `USBHOST_DMAMEM` is defined per-`.cpp` in
this codebase (see `memory.cpp:68`), so define it the same way at the top of
this file rather than expecting it from a header.

```cpp
#if defined(__IMXRT1062__) || defined(__IMXRT1176__)
#define USBHOST_DMAMEM DMAMEM
#else
#define USBHOST_DMAMEM
#endif

// 16 siTDs: 12 frames of ring depth (spec section 8) plus headroom for the
// refill to run ahead of the hardware.
#define SITD_POOL_SIZE 16
static USBHOST_DMAMEM sitd_t sitd_pool[SITD_POOL_SIZE] __attribute__ ((aligned(32)));
static sitd_t *sitd_free_list;
```

Add `sitd_pool_init()`, `sitd_alloc()` and `sitd_free()` with the obvious
free-list behaviour, declared in `ehci_iso.h`.

- [ ] **Step 2: Complete the ehci.cpp:1321 TODO**

The periodic-list walk currently assumes every entry is a QH. Teach it to read
the type bits and skip iTD/siTD entries. Replace the `// TODO: skip past iTD,
siTD when/if we support isochronous` comment with a walk that switches on
`(pointer >> 1) & 3`, treating type 0 (iTD) and type 2 (siTD) as "advance to
the next link pointer" rather than "interpret as QH".

Getting this wrong corrupts the interrupt-endpoint traversal, so re-run the
existing HID gate afterwards, not just the audio path.

- [ ] **Step 3: Link a single siTD into the frame list**

Add `sitd_link(sitd_t *s, uint16_t frame)` writing
`periodictable[frame] = (uint32_t)s | SITD_PTR_TYPE_SITD`, preserving the
existing entry as the siTD's `next` so interrupt QHs already in that frame stay
reachable. Mirror `USB_HostEhciLinkSitd` in NXP's driver.

- [ ] **Step 4: Verify nothing regressed**

Run: `cd test && make` — expect `0 failures`.

Build and run `rt1176-evkb/examples/usb/usb_host_hid_test` on hardware with a
keyboard or mouse attached. Expect the existing HID markers, proving the
periodic-list change did not break interrupt endpoints.

- [ ] **Step 5: Commit**

```bash
git add ehci_iso.h ehci_iso.cpp ehci.cpp
git commit -m "feat: siTD DMAMEM pool and periodic frame list linking

Also completes the ehci.cpp iTD/siTD skip in the periodic walk, which
previously assumed every frame-list entry was a queue head."
```

---

## Task 4: First packet on the wire

> **DONE 2026-08-01 — commit `94b889f`.** Verified on MIMXRT1170-EVKB against a
> Logitech USB Headset (046D:0A8F) on `USB_OTG2`:
>
> ```
> UAC1-TEST: siTD posted, 192 bytes
> UAC1-TEST: siTD active=0 xact_err=0 babble=0 buf_err=0 bytes_left=0
> UAC1-TEST: SITD PASS - controller sent the packet
> ```
>
> `active=0` means the controller walked the periodic list, found the siTD and
> executed it; `bytes_left=0` means all 192 bytes went out; no error bits means
> the transaction completed on the bus.
>
> Confirmed on silicon, having been only inference before:
>
> - an isochronous OUT takes **no** complete-splits (`cmask = 0`) — the least
>   certain assumption in this design
> - T-count `(len + 187) / 188` with TP seeding Begin/Mid/End
> - `HubAddr = 0` addresses the embedded TT for a root-port device
>   (RM Table 62-56)
> - siTD layout, 32-byte alignment and DMAMEM placement — `buf_err` would have
>   flagged a DMA-unreachable payload
>
> Verified from the siTD writeback, not a capture. Steps 3–4 below describe the
> analyser check that was originally planned; it was not used and is not
> required.

The milestone that de-risks everything. One siTD, one 192-byte OUT, visible on
the analyser.

**Files:**
- Modify: `ehci_iso.cpp`, `usb_audio.cpp`

- [ ] **Step 1: Open the iso pipe after SET_INTERFACE**

In `USBAudioOut::control()`, once `active_alt` becomes valid, allocate the iso
pipe using the selected alternate setting's endpoint address and
`max_packet_size`, with `hub_addr = 0` and the device address — per RM
Table 62-56, a directly attached full-speed device uses `HubAddr = 0`.

- [ ] **Step 2: Post one siTD carrying 192 bytes of silence**

Fill a DMAMEM payload buffer with zeros, populate one siTD via
`sitd_budget_out(192, 0, ...)`, set `SITD_STATUS_ACTIVE` and the IOC bit, link
it one frame ahead of the current frame index, and return.

- [ ] **Step 3: Verify on hardware**

Capture with the Saleae on D+/D− of J18 at 100 MS/s while resetting the board
with the headset attached, export the analyzer table, then:

Run: `./tools/usbcap.py summary <capture>.csv`

Expected: the per-endpoint table shows at least one `OUT addr=<n> ep=4` with a
192-byte DATA payload and no handshake — isochronous has none.

Note the split tokens will **not** appear in this capture. The transaction
translator is inside the SoC, so its downstream side carries plain full-speed
traffic. See `tools/README.md`.

- [ ] **Step 4: Commit**

```bash
git add ehci_iso.cpp usb_audio.cpp
git commit -m "feat: post a single isochronous OUT siTD

First packet on the wire: 192 bytes of silence to the headset's alt 7
endpoint, confirmed on the analyser."
```

---

## Task 5: Continuous ring

> **DONE 2026-08-01 — audio confirmed audible.** Verified on MIMXRT1170-EVKB
> against a Logitech USB Headset (046D:0A8F) on `USB_OTG2`:
>
> ```
> UAC1-TEST: streaming started, 1 kHz tone
> UAC1-TEST: HEARTBEAT ... audio=ready alt=7 pkts/s=1000 total=39260
> ```
>
> **`pkts/s=1000`, sustained** — a packet in every 1 ms frame, no empty frames,
> holding steady over tens of thousands of packets. And the 1 kHz tone is
> audible in the headset, which proves the whole path end to end: descriptors,
> split transactions, frame scheduling, timing and payload format.
>
> Two corrections to this plan came out of it:
>
> - **The 12-frame ring was wrong.** `PERIODIC_LIST_SIZE` is 32 and a slot is
>   revisited only every 32 frames, so twelve descriptors would have
>   transmitted in 12 of every 32 frames — a 37% duty cycle. It needs one siTD
>   per slot.
> - **Interrupt-on-complete is off.** With `service()` polling the status word,
>   IOC on 32 descriptors would add ~1000 IRQ/s into an ISR with no siTD
>   handling, for nothing.
>
> Latency is ~32 ms, not the 10–30 ms the spec budgeted: a slot's payload is
> written one full frame-list cycle before it plays. Inherent to the frame list
> size; revisit if it matters.

**Files:**
- Modify: `ehci_iso.cpp`, `usb_audio.cpp`, `usb_audio.h`

- [ ] **Step 1: Build the ring**

Twelve siTDs linked into twelve consecutive frames, each with its own DMAMEM
payload buffer. On completion the ISR retires the finished siTD, refills it from
the sample FIFO, and relinks it twelve frames ahead.

- [ ] **Step 2: Add the FIFO and counters**

Implement `write()`, `available()`, `underruns()` and `starvations()` from the
spec's section 7.3 API. On an empty FIFO emit a full-size packet of silence and
increment `underruns()`; if the hardware reaches an siTD still marked active,
increment `starvations()`.

- [ ] **Step 3: Feed it a synthesised tone**

In the test sketch, fill the FIFO with a 48 kHz sine rather than wiring up the
Audio library — that is the next plan's job.

- [ ] **Step 4: Verify on hardware**

Run: `./tools/usbcap.py iso <capture>.csv --rate 48000`

Expected: `sizes` shows `192 B` only, `gaps` shows 0 over a 60-second capture,
cadence average within a few hundred nanoseconds of 1.0000 ms. Firmware
`underruns()` and `starvations()` must both read 0, and must agree with the
capture — a nonzero gap count with zero starvations means the counters are
wrong, not the ring.

- [ ] **Step 5: Commit**

```bash
git add ehci_iso.cpp usb_audio.cpp usb_audio.h
git commit -m "feat: continuous isochronous ring with underrun accounting"
```

---

## Task 6: Match the golden trace

**Blocked on M0b.** Requires the reference capture.

- [ ] **Step 1: Diff against the reference**

Capture the RT1176 streaming and compare with the golden trace under
`usbcap.py iso`. Payload size histogram, effective delivered rate and gap count
must match. The reference bar from the PC-driven capture is 100.01% of target
with zero gaps and cadence within 0.3 µs.

- [ ] **Step 2: Record the result**

Append the figures to the spec's section 11 as M3 evidence.

---

## Definition of done

- `cd test && make` reports 0 failures from both binaries
- The HID gate still passes, proving the periodic-list change is safe
- `usbcap.py iso` reports zero gaps over 60 seconds at 48 kHz
- Firmware underrun and starvation counters read zero and agree with the capture
- `LICENSE.md` lists `ehci_iso.cpp` as BSD-3-Clause

Once these hold, the Audio library integration and 44.1 kHz packet scheduling
(spec M5) can be planned.
