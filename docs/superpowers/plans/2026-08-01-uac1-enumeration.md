# UAC1 Enumeration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Parse a USB Audio Class 1.0 device's descriptors, select the alternate setting matching a requested rate and format, and activate it with `SET_INTERFACE` — with no audio streaming yet.

**Architecture:** The descriptor parser is deliberately free of Arduino and USBHost_t36 dependencies so it compiles and unit-tests natively on the host against a real captured descriptor. The driver layer on top is a thin `USBDriver` subclass. Streaming (siTD transport) is out of scope for this plan and follows in a separate one, gated on Task 1.

**Tech Stack:** C++11, Arduino/Teensy toolchain for the on-target parts, plain `make` plus a dependency-free assert harness for host tests. Verification on hardware uses `tools/usbcap.py`.

---

## Scope

This plan covers **M0 (hardware gate)** and **M1 (enumeration)** from
[the design spec](../specs/2026-08-01-usb-uac1-audio-output-design.md).

It deliberately stops before siTD transport. The spec's risk section states that
everything from M2 onward assumes full-speed isochronous works through the
RT1176 embedded transaction translator, which is an inference rather than an
observation. Task 1 tests that inference. Writing the siTD plan before Task 1
returns would risk planning work that gets invalidated.

Tasks 2–8 do **not** depend on Task 1's outcome — descriptor parsing and
`SET_INTERFACE` are needed for any audio approach — so Task 1 can run in
parallel with them.

## File Structure

| File | Responsibility |
|---|---|
| `usb_audio_parse.h` (create) | Plain-C++ types and API for UAC1 descriptor parsing. No Arduino, no USBHost_t36. |
| `usb_audio_parse.cpp` (create) | The parser. Host-testable in isolation. |
| `test/Makefile` (create) | Host test build. |
| `test/test_uac1_parse.cpp` (create) | Assert-based tests over the real fixture. |
| `test/fixtures/headset_uac1_config.bin` (exists) | Real 799-byte configuration descriptor captured from the test headset. |
| `usb_audio.h` / `usb_audio.cpp` (create) | `USBAudioOut : public USBDriver`. Claims interfaces, drives `SET_INTERFACE`. |
| `USBHost_t36.h` (modify) | Declare `USBAudioOut`; add the `setInterface` helper declaration. |
| `enumeration.cpp` (modify) | Add the `SET_INTERFACE` control-transfer helper. |

The parser is split from the driver because it is the only part that can be
tested without hardware, and it is where the fiddly logic lives.

---

## Task 1: Hardware gate — prove full-speed isochronous works

**This gates the follow-on siTD plan, not the rest of this one.** No repository
files change.

**Files:**
- Modify (in the SDK tree, not this repo): `~/Development/mcuxsdk-examples/usb_examples/usb_host_audio_speaker/bm/app.h`
- Modify (in the SDK tree): `~/Development/mcuxsdk-examples/usb_examples/usb_host_audio_speaker/bm/audio_speaker.c:594`

- [ ] **Step 1: Build and run NXP's example unmodified**

Build `usb_host_audio_speaker` for board `evkbmimxrt1170`, flash it, attach the
test headset to the board's USB host connector via an OTG adapter, and open the
board UART.

Expected: the terminal prints the audio device information and audio is audible.

**Expect 8 kHz, not 48 kHz.** `audio_speaker.c:594` hardcodes the streaming
alternate setting to 1, which on this headset is 8000 Hz stereo. The 48000 Hz
search at line 986 is inside the `AUDIO_DEVICE_VERSION_02` (UAC2) branch and
does not run for this UAC1 device.

- [ ] **Step 2: Record the outcome**

If audio is audible at any rate, full-speed isochronous works through the
embedded TT and the siTD plan is sound. Write the result into the spec's risk
section.

If it fails, check physical causes first — OTG adapter, VBUS, the headset's
100 mA draw — then try a different UAC1 device. Only if several devices fail
should the embedded-TT premise be treated as suspect, at which point the design
needs re-scoping. **Do not begin the siTD plan.**

- [ ] **Step 3: Retarget to OTG2 and 48 kHz**

Add to the compiler defines: `CONTROLLER_ID=kUSB_ControllerEhci1`. `app.h`
guards it with `#ifndef`, so this overrides cleanly without editing the file.
This selects OTG2, which is the controller this library targets; the default
`kUSB_ControllerEhci0` is OTG1.

In `audio_speaker.c:594`, change the alternate setting argument from `1` to `7`:

```c
if (USB_HostAudioStreamSetInterface(g_audio.classHandle, g_audio.streamIntfHandle, 7,
                                    Audio_ControlCallback, &g_audio) != kStatus_USB_Success)
```

- [ ] **Step 4: Capture the golden trace**

With the Saleae on D+/D- of the host port at 100 MS/s, capture while audio
plays, export the "USB LS and FS" analyzer table to CSV, then:

Run: `./tools/usbcap.py iso <capture>.csv --rate 48000`

Expected: `sizes` shows `192 B`, `gaps` shows 0, cadence average within a few
hundred nanoseconds of 1.0000 ms. Keep this capture — it is the reference the
RT1176 implementation must match.

---

## Task 2: Host test harness

**Files:**
- Create: `test/Makefile`
- Create: `test/test_uac1_parse.cpp`
- Create: `usb_audio_parse.h`

- [ ] **Step 1: Write the header**

Create `usb_audio_parse.h`:

```cpp
// Copyright (c) 2026 Nicholas Newdigate
// SPDX-License-Identifier: MIT
//
// USB Audio Class 1.0 descriptor parsing. Deliberately free of Arduino and
// USBHost_t36 dependencies so it can be unit-tested on the host.
#pragma once
#include <stdint.h>
#include <stddef.h>

#define UAC1_MAX_ALTS 16

struct UAC1AltSetting {
	uint8_t  alternate_setting;
	uint8_t  endpoint_address;
	uint8_t  endpoint_attributes;
	uint16_t max_packet_size;
	uint8_t  channels;
	uint8_t  subframe_size;
	uint8_t  bit_resolution;
	uint32_t sample_rate;
};

struct UAC1Topology {
	uint16_t bcd_adc;
	uint8_t  control_interface;    // 0xFF if none
	uint8_t  streaming_interface;  // 0xFF if none
	uint8_t  feature_unit_id;      // 0 if none
	uint8_t  alt_count;
	UAC1AltSetting alts[UAC1_MAX_ALTS];
};

// Parses a full configuration descriptor. Returns false if the descriptor
// contains no audio streaming interface with an isochronous OUT endpoint.
bool uac1_parse_config(const uint8_t *desc, size_t len, UAC1Topology *out);

// Returns the alternate setting number matching the format, or -1.
int uac1_find_alt(const UAC1Topology *t, uint32_t rate, uint8_t channels, uint8_t bits);
```

- [ ] **Step 2: Write the test harness with one failing test**

Create `test/test_uac1_parse.cpp`:

```cpp
// Copyright (c) 2026 Nicholas Newdigate
// SPDX-License-Identifier: MIT
#include "usb_audio_parse.h"
#include <stdio.h>
#include <stdlib.h>

static int failures = 0;
static int checks = 0;

#define CHECK(cond) do { checks++; if (!(cond)) { \
	printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); failures++; } } while (0)

#define CHECK_EQ(a, b) do { checks++; long _a = (long)(a), _b = (long)(b); \
	if (_a != _b) { printf("FAIL %s:%d: %s == %s (got %ld, want %ld)\n", \
	__FILE__, __LINE__, #a, #b, _a, _b); failures++; } } while (0)

static uint8_t fixture[4096];
static size_t fixture_len;

static void load_fixture(void)
{
	FILE *f = fopen("fixtures/headset_uac1_config.bin", "rb");
	if (!f) { printf("FAIL cannot open fixture\n"); exit(1); }
	fixture_len = fread(fixture, 1, sizeof(fixture), f);
	fclose(f);
}

static void test_fixture_is_a_config_descriptor(void)
{
	CHECK_EQ(fixture_len, 799);
	CHECK_EQ(fixture[0], 9);      // bLength
	CHECK_EQ(fixture[1], 0x02);   // CONFIGURATION
	CHECK_EQ(fixture[2] | (fixture[3] << 8), 799);  // wTotalLength
	CHECK_EQ(fixture[4], 4);      // bNumInterfaces
}

static void test_rejects_garbage(void)
{
	UAC1Topology t;
	uint8_t junk[16] = {0};
	CHECK(!uac1_parse_config(junk, sizeof(junk), &t));
	CHECK(!uac1_parse_config(0, 0, &t));
}

int main(void)
{
	load_fixture();
	test_fixture_is_a_config_descriptor();
	test_rejects_garbage();
	printf("%d checks, %d failures\n", checks, failures);
	return failures ? 1 : 0;
}
```

- [ ] **Step 3: Write the Makefile**

Create `test/Makefile`:

```make
CXX ?= c++
CXXFLAGS ?= -std=c++11 -Wall -Wextra -Werror -g -I..

all: run

test_uac1_parse: test_uac1_parse.cpp ../usb_audio_parse.cpp ../usb_audio_parse.h
	$(CXX) $(CXXFLAGS) -o $@ test_uac1_parse.cpp ../usb_audio_parse.cpp

run: test_uac1_parse
	./test_uac1_parse

clean:
	rm -f test_uac1_parse

.PHONY: all run clean
```

- [ ] **Step 4: Run to verify it fails to build**

Run: `cd test && make`
Expected: link error — `usb_audio_parse.cpp` does not exist yet.

- [ ] **Step 5: Create a stub implementation**

Create `usb_audio_parse.cpp`:

```cpp
// Copyright (c) 2026 Nicholas Newdigate
// SPDX-License-Identifier: MIT
#include "usb_audio_parse.h"
#include <string.h>

bool uac1_parse_config(const uint8_t *d, size_t len, UAC1Topology *out)
{
	(void)d; (void)len; (void)out;
	return false;
}

int uac1_find_alt(const UAC1Topology *t, uint32_t rate, uint8_t channels, uint8_t bits)
{
	(void)t; (void)rate; (void)channels; (void)bits;
	return -1;
}
```

- [ ] **Step 6: Run to verify tests pass**

Run: `cd test && make`
Expected: the final line reads `0 failures` and make exits 0

- [ ] **Step 7: Commit**

```bash
git add usb_audio_parse.h usb_audio_parse.cpp test/Makefile test/test_uac1_parse.cpp test/fixtures/headset_uac1_config.bin
git commit -m "test: host harness for UAC1 descriptor parsing

Adds a dependency-free assert harness and the real 799-byte configuration
descriptor captured from the test headset, so the parser can be developed
against a genuinely messy input with no board attached."
```

---

## Task 3: Identify the control and output streaming interfaces

The device exposes both a microphone and a speaker streaming interface, so
selection must be by endpoint direction rather than by taking the first audio
streaming interface found.

**Files:**
- Modify: `usb_audio_parse.cpp`
- Modify: `test/test_uac1_parse.cpp`

- [ ] **Step 1: Write the failing test**

Add to `test/test_uac1_parse.cpp`, and call it from `main` after
`test_rejects_garbage()`:

```cpp
static void test_identifies_interfaces(void)
{
	UAC1Topology t;
	CHECK(uac1_parse_config(fixture, fixture_len, &t));
	CHECK_EQ(t.bcd_adc, 0x0100);           // UAC 1.00
	CHECK_EQ(t.control_interface, 0);
	CHECK_EQ(t.streaming_interface, 2);    // not 1, which is the microphone
}
```

- [ ] **Step 2: Run to verify it fails**

Run: `cd test && make`
Expected: FAIL on `uac1_parse_config(...)` returning false.

- [ ] **Step 3: Implement interface identification**

Replace the body of `usb_audio_parse.cpp` (keeping the copyright header):

```cpp
#include "usb_audio_parse.h"
#include <string.h>

#define DT_INTERFACE       0x04
#define DT_ENDPOINT        0x05
#define DT_CS_INTERFACE    0x24
#define AC_HEADER          0x01
#define AC_OUTPUT_TERMINAL 0x03
#define AC_FEATURE_UNIT    0x06
#define AS_FORMAT_TYPE     0x02
#define AUDIO_CLASS        0x01
#define SUBCLASS_CONTROL   0x01
#define SUBCLASS_STREAM    0x02

// Finds the audio streaming interface carrying an isochronous OUT endpoint.
// Returns 0xFF if there is none.
static uint8_t find_output_streaming_interface(const uint8_t *d, size_t len)
{
	uint8_t cur = 0xFF;
	bool cur_is_stream = false;
	size_t i = 0;
	while (i + 1 < len && d[i] >= 2 && i + d[i] <= len) {
		uint8_t l = d[i], t = d[i + 1];
		if (t == DT_INTERFACE && l >= 9) {
			cur_is_stream = (d[i+5] == AUDIO_CLASS && d[i+6] == SUBCLASS_STREAM);
			cur = d[i+2];
		} else if (t == DT_ENDPOINT && l >= 7 && cur_is_stream) {
			if ((d[i+2] & 0x80) == 0 && (d[i+3] & 0x03) == 0x01) return cur;
		}
		i += l;
	}
	return 0xFF;
}

bool uac1_parse_config(const uint8_t *d, size_t len, UAC1Topology *out)
{
	if (!d || !out || len < 9) return false;
	memset(out, 0, sizeof(*out));
	out->control_interface = 0xFF;
	out->streaming_interface = 0xFF;

	uint8_t stream_if = find_output_streaming_interface(d, len);
	if (stream_if == 0xFF) return false;
	out->streaming_interface = stream_if;

	bool in_control = false;
	size_t i = 0;
	while (i + 1 < len && d[i] >= 2 && i + d[i] <= len) {
		const uint8_t *b = d + i;
		uint8_t l = b[0], t = b[1];
		if (t == DT_INTERFACE && l >= 9) {
			in_control = (b[5] == AUDIO_CLASS && b[6] == SUBCLASS_CONTROL);
			if (in_control && out->control_interface == 0xFF)
				out->control_interface = b[2];
		} else if (t == DT_CS_INTERFACE && l >= 8 && in_control && b[2] == AC_HEADER) {
			out->bcd_adc = (uint16_t)b[3] | ((uint16_t)b[4] << 8);
		}
		i += l;
	}
	return true;
}

int uac1_find_alt(const UAC1Topology *t, uint32_t rate, uint8_t channels, uint8_t bits)
{
	(void)t; (void)rate; (void)channels; (void)bits;
	return -1;
}
```

- [ ] **Step 4: Run to verify it passes**

Run: `cd test && make`
Expected: the final line reads `0 failures` and make exits 0

- [ ] **Step 5: Commit**

```bash
git add usb_audio_parse.cpp test/test_uac1_parse.cpp
git commit -m "feat: identify UAC1 control and output streaming interfaces

Selects the streaming interface by isochronous OUT endpoint rather than
first-found, because the test headset exposes a microphone streaming
interface ahead of the speaker one."
```

---

## Task 4: Collect alternate settings and formats

**Files:**
- Modify: `usb_audio_parse.cpp`
- Modify: `test/test_uac1_parse.cpp`

- [ ] **Step 1: Write the failing test**

Add to `test/test_uac1_parse.cpp` and call it from `main`:

```cpp
static void test_collects_alt_settings(void)
{
	UAC1Topology t;
	CHECK(uac1_parse_config(fixture, fixture_len, &t));
	CHECK_EQ(t.alt_count, 8);              // alts 0..7 on interface 2

	// alt 0 is the zero-bandwidth setting: no endpoint
	CHECK_EQ(t.alts[0].alternate_setting, 0);
	CHECK_EQ(t.alts[0].endpoint_address, 0);

	// alt 7 is 48 kHz stereo 16-bit
	CHECK_EQ(t.alts[7].alternate_setting, 7);
	CHECK_EQ(t.alts[7].sample_rate, 48000);
	CHECK_EQ(t.alts[7].channels, 2);
	CHECK_EQ(t.alts[7].bit_resolution, 16);
	CHECK_EQ(t.alts[7].subframe_size, 2);
	CHECK_EQ(t.alts[7].endpoint_address, 0x04);
	CHECK_EQ(t.alts[7].max_packet_size, 248);
	// isochronous (bits 1:0 == 01) and adaptive (bits 3:2 == 10) => 0x09
	CHECK_EQ(t.alts[7].endpoint_attributes, 0x09);

	// alt 6 is 44.1 kHz stereo 16-bit
	CHECK_EQ(t.alts[6].sample_rate, 44100);
	CHECK_EQ(t.alts[6].max_packet_size, 228);
}
```

- [ ] **Step 2: Run to verify it fails**

Run: `cd test && make`
Expected: FAIL on `t.alt_count == 8` (got 0).

- [ ] **Step 3: Implement alt-setting collection**

In `usb_audio_parse.cpp`, replace the parse loop inside `uac1_parse_config`
(everything from `bool in_control = false;` to the closing `return true;`) with:

```cpp
	bool in_control = false, in_stream = false;
	UAC1AltSetting *alt = 0;
	size_t i = 0;
	while (i + 1 < len && d[i] >= 2 && i + d[i] <= len) {
		const uint8_t *b = d + i;
		uint8_t l = b[0], t = b[1];
		if (t == DT_INTERFACE && l >= 9) {
			in_control = (b[5] == AUDIO_CLASS && b[6] == SUBCLASS_CONTROL);
			in_stream  = (b[5] == AUDIO_CLASS && b[6] == SUBCLASS_STREAM
			              && b[2] == stream_if);
			alt = 0;
			if (in_control && out->control_interface == 0xFF)
				out->control_interface = b[2];
			if (in_stream && out->alt_count < UAC1_MAX_ALTS) {
				alt = &out->alts[out->alt_count++];
				memset(alt, 0, sizeof(*alt));
				alt->alternate_setting = b[3];
			}
		} else if (t == DT_CS_INTERFACE && l >= 3) {
			if (in_control && b[2] == AC_HEADER && l >= 8) {
				out->bcd_adc = (uint16_t)b[3] | ((uint16_t)b[4] << 8);
			} else if (in_stream && alt && b[2] == AS_FORMAT_TYPE && l >= 11) {
				alt->channels       = b[4];
				alt->subframe_size  = b[5];
				alt->bit_resolution = b[6];
				if (b[7] >= 1)  // bSamFreqType: discrete frequencies
					alt->sample_rate = (uint32_t)b[8]
					                 | ((uint32_t)b[9] << 8)
					                 | ((uint32_t)b[10] << 16);
			}
		} else if (t == DT_ENDPOINT && l >= 7 && in_stream && alt) {
			alt->endpoint_address    = b[2];
			alt->endpoint_attributes = b[3];
			alt->max_packet_size     = (uint16_t)b[4] | ((uint16_t)b[5] << 8);
		}
		i += l;
	}
	return out->alt_count > 0;
```

- [ ] **Step 4: Run to verify it passes**

Run: `cd test && make`
Expected: the final line reads `0 failures` and make exits 0

- [ ] **Step 5: Commit**

```bash
git add usb_audio_parse.cpp test/test_uac1_parse.cpp
git commit -m "feat: collect UAC1 alternate settings, formats and endpoints"
```

---

## Task 5: Resolve the feature unit for volume and mute

The device has three feature units (19, 35 and 22). The right one is whichever
feeds the speaker output terminal — 22 on the test headset. Picking the first
one found would give 19, which controls a microphone.

**Files:**
- Modify: `usb_audio_parse.cpp`
- Modify: `test/test_uac1_parse.cpp`

- [ ] **Step 1: Write the failing test**

Add to `test/test_uac1_parse.cpp` and call it from `main`:

```cpp
static void test_resolves_speaker_feature_unit(void)
{
	UAC1Topology t;
	CHECK(uac1_parse_config(fixture, fixture_len, &t));
	// Output terminal 16 is a Speaker (0x0301) sourced from unit 22.
	// Units 19 and 35 are microphone feature units and must not be chosen.
	CHECK_EQ(t.feature_unit_id, 22);
}
```

- [ ] **Step 2: Run to verify it fails**

Run: `cd test && make`
Expected: FAIL — `t.feature_unit_id` is 0.

- [ ] **Step 3: Implement feature unit resolution**

In `usb_audio_parse.cpp`, add this helper above `uac1_parse_config`:

```cpp
static bool is_speaker_terminal(uint16_t tt)
{
	return tt == 0x0301   // Speaker
	    || tt == 0x0302   // Headphones
	    || tt == 0x0402;  // Headset
}
```

Declare the collection state immediately before the parse loop, next to
`bool in_control = false, in_stream = false;`:

```cpp
	uint8_t fu_ids[UAC1_MAX_ALTS];
	uint8_t fu_count = 0;
	uint8_t speaker_src = 0;
```

Extend the `in_control` branch of the `DT_CS_INTERFACE` case so it reads:

```cpp
			if (in_control && b[2] == AC_HEADER && l >= 8) {
				out->bcd_adc = (uint16_t)b[3] | ((uint16_t)b[4] << 8);
			} else if (in_control && b[2] == AC_OUTPUT_TERMINAL && l >= 9) {
				uint16_t tt = (uint16_t)b[4] | ((uint16_t)b[5] << 8);
				if (is_speaker_terminal(tt)) speaker_src = b[7];  // bSourceID
			} else if (in_control && b[2] == AC_FEATURE_UNIT && l >= 4) {
				if (fu_count < UAC1_MAX_ALTS) fu_ids[fu_count++] = b[3];
			} else if (in_stream && alt && b[2] == AS_FORMAT_TYPE && l >= 11) {
```

Replace the final `return out->alt_count > 0;` with:

```cpp
	for (uint8_t k = 0; k < fu_count; k++) {
		if (fu_ids[k] == speaker_src) { out->feature_unit_id = speaker_src; break; }
	}
	return out->alt_count > 0;
```

- [ ] **Step 4: Run to verify it passes**

Run: `cd test && make`
Expected: the final line reads `0 failures` and make exits 0

- [ ] **Step 5: Commit**

```bash
git add usb_audio_parse.cpp test/test_uac1_parse.cpp
git commit -m "feat: resolve the speaker feature unit via its output terminal

Picking the first feature unit would select a microphone unit on the test
headset. Resolves through the speaker output terminal's bSourceID instead."
```

---

## Task 6: Select an alternate setting by format

**Files:**
- Modify: `usb_audio_parse.cpp`
- Modify: `test/test_uac1_parse.cpp`

- [ ] **Step 1: Write the failing test**

Add both functions to `test/test_uac1_parse.cpp` and call both from `main`:

```cpp
static void test_finds_alt_by_format(void)
{
	UAC1Topology t;
	CHECK(uac1_parse_config(fixture, fixture_len, &t));
	CHECK_EQ(uac1_find_alt(&t, 48000, 2, 16), 7);   // bring-up target
	CHECK_EQ(uac1_find_alt(&t, 44100, 2, 16), 6);   // shipping target
	CHECK_EQ(uac1_find_alt(&t,  8000, 2, 16), 1);
	CHECK_EQ(uac1_find_alt(&t, 96000, 2, 16), -1);  // unsupported rate
	CHECK_EQ(uac1_find_alt(&t, 48000, 1, 16), -1);  // unsupported channel count
	CHECK_EQ(uac1_find_alt(0,   48000, 2, 16), -1);
}

// USBDriver::claim() at type 0 receives `enumbuf + 9` -- the descriptor set
// with the 9-byte configuration header already skipped. Parsing either form
// must give the same answer, or the driver will disagree with these tests.
static void test_parses_without_config_header(void)
{
	UAC1Topology full, offset;
	CHECK(uac1_parse_config(fixture, fixture_len, &full));
	CHECK(uac1_parse_config(fixture + 9, fixture_len - 9, &offset));
	CHECK_EQ(offset.streaming_interface, full.streaming_interface);
	CHECK_EQ(offset.control_interface,   full.control_interface);
	CHECK_EQ(offset.feature_unit_id,     full.feature_unit_id);
	CHECK_EQ(offset.alt_count,           full.alt_count);
	CHECK_EQ(uac1_find_alt(&offset, 48000, 2, 16), 7);
}
```

- [ ] **Step 2: Run to verify it fails**

Run: `cd test && make`
Expected: FAIL — `uac1_find_alt` returns -1 for 48000.

- [ ] **Step 3: Implement the lookup**

In `usb_audio_parse.cpp`, replace the stub `uac1_find_alt` with:

```cpp
int uac1_find_alt(const UAC1Topology *t, uint32_t rate, uint8_t channels, uint8_t bits)
{
	if (!t) return -1;
	for (uint8_t k = 0; k < t->alt_count; k++) {
		const UAC1AltSetting *a = &t->alts[k];
		if (a->sample_rate == rate && a->channels == channels &&
		    a->bit_resolution == bits && a->endpoint_address != 0)
			return (int)a->alternate_setting;
	}
	return -1;
}
```

The `endpoint_address != 0` term excludes the zero-bandwidth alt 0, which has
no endpoint and therefore no format.

- [ ] **Step 4: Run to verify it passes**

Run: `cd test && make`
Expected: the final line reads `0 failures` and make exits 0

- [ ] **Step 5: Commit**

```bash
git add usb_audio_parse.cpp test/test_uac1_parse.cpp
git commit -m "feat: select UAC1 alternate setting by rate, channels and depth"
```

---

## Task 7: Add a SET_INTERFACE helper

The library issues no `SET_INTERFACE` anywhere today. This is a general
capability, not an audio-specific one.

**Files:**
- Modify: `USBHost_t36.h`
- Modify: `enumeration.cpp`

- [ ] **Step 1: Declare the helper**

In `USBHost_t36.h`, inside the `USBHost` class alongside the other static
transfer helpers (near the `queue_Control_Transfer` declaration around line 283),
add:

```cpp
	static bool setInterface(Device_t *dev, setup_t &setup, uint8_t interface,
	                         uint8_t alternate, USBDriver *driver);
```

The caller supplies the `setup_t`. It must stay alive until the transfer
completes, so drivers hold it as a member — the same pattern `USBHub` uses.
A `static` local here would be a bug: two drivers enumerating at once would
overwrite each other's setup packet.

- [ ] **Step 2: Implement it**

In `enumeration.cpp`, append:

```cpp
// USB 2.0 section 9.4.10: SET_INTERFACE
//   bmRequestType = 0x01 (host-to-device, standard, interface recipient)
//   bRequest      = 0x0B
//   wValue        = alternate setting
//   wIndex        = interface number
//   wLength       = 0 (no data stage)
bool USBHost::setInterface(Device_t *dev, setup_t &setup, uint8_t interface,
                           uint8_t alternate, USBDriver *driver)
{
	if (!dev) return false;
	mk_setup(setup, 0x01, 0x0B, alternate, interface, 0);
	return queue_Control_Transfer(dev, &setup, NULL, driver);
}
```

- [ ] **Step 3: Verify it compiles**

```bash
arduino-cli compile -b teensy:avr:mimxrt1060evkb --library . examples/HIDDeviceInfo
```

Expected: compiles with no new warnings, and the "Used library" table names
`/Users/nicholasnewdigate/Development/USBHost_t36`.

**The `--library .` is essential.** The Teensy core ships its own copy of this
library at
`~/Library/Arduino15/packages/teensy/hardware/avr/1.59.0/libraries/USBHost_t36`,
and without `--library` that copy wins — the build then succeeds no matter what
you changed in the working tree, which is worse than no check at all. Always
confirm the "Used library" path points at the working tree before believing a
compile result.

(Compiling for the RT1062 EVKB is sufficient — this code is not chip-specific,
and it keeps the check fast.)

- [ ] **Step 4: Commit**

```bash
git add USBHost_t36.h enumeration.cpp
git commit -m "feat: add SET_INTERFACE control transfer helper

The library issued no SET_INTERFACE anywhere. Every audio streaming
interface requires one, and a future UVC or CDC driver will too."
```

---

## Task 8: USBAudioOut driver claiming the audio interfaces

**Files:**
- Create: `usb_audio.h`
- Create: `usb_audio.cpp`
- Modify: `USBHost_t36.h`

- [ ] **Step 1: Write the driver header**

Create `usb_audio.h`:

```cpp
// Copyright (c) 2026 Nicholas Newdigate
// SPDX-License-Identifier: MIT
#pragma once
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
```

- [ ] **Step 2: Write the driver implementation**

Create `usb_audio.cpp`:

```cpp
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
```

- [ ] **Step 3: Include it from the library header**

In `USBHost_t36.h`, at the end of the file before the final `#endif`, add:

```cpp
#include "usb_audio.h"
```

- [ ] **Step 4: Write an example sketch**

Create `examples/Audio/UAC1Info/UAC1Info.ino`:

```cpp
#include <USBHost_t36.h>

USBHost myusb;
USBHub hub1(myusb);
USBAudioOut audioOut(myusb);

void setup() {
	while (!Serial && millis() < 3000) ;
	Serial.println("UAC1 enumeration test");
	audioOut.format(48000, 2, 16);
	myusb.begin();
}

bool reported = false;

void loop() {
	myusb.Task();
	if (audioOut.ready() && !reported) {
		const UAC1Topology &t = audioOut.topology();
		Serial.printf("UAC %x.%02x  control if=%d  streaming if=%d  feature unit=%d\n",
			t.bcd_adc >> 8, t.bcd_adc & 0xFF, t.control_interface,
			t.streaming_interface, t.feature_unit_id);
		for (uint8_t i = 0; i < t.alt_count; i++) {
			const UAC1AltSetting &a = t.alts[i];
			Serial.printf("  alt %d: ep=0x%02X attr=0x%02X mps=%d ch=%d bits=%d rate=%lu\n",
				a.alternate_setting, a.endpoint_address, a.endpoint_attributes,
				a.max_packet_size, a.channels, a.bit_resolution,
				(unsigned long)a.sample_rate);
		}
		Serial.printf("selected alternate setting %d\n", audioOut.alternateSetting());
		reported = true;
	}
	if (!audioOut.ready()) reported = false;
}
```

- [ ] **Step 5: Verify it compiles**

```bash
arduino-cli compile -b teensy:avr:mimxrt1060evkb --library . examples/Audio/UAC1Info
```

Expected: compiles with no new warnings, and the "Used library" table names the
working tree. See the note in Task 7 Step 3 — without `--library .` this builds
the Teensy core's bundled copy of USBHost_t36 and tells you nothing.

- [ ] **Step 6: Commit**

```bash
git add usb_audio.h usb_audio.cpp USBHost_t36.h examples/Audio/UAC1Info
git commit -m "feat: USBAudioOut driver claiming UAC1 interfaces

Parses the topology, selects the alternate setting matching the requested
format, and activates it with SET_INTERFACE. No streaming yet."
```

---

## Task 9: Verify on hardware

**Files:** none — verification only.

- [ ] **Step 1: Run the example against the test headset**

Flash `examples/Audio/UAC1Info` to the MIMXRT1170-EVKB and attach the headset to
the OTG2 host connector.

Expected serial output:

```
UAC 1.00  control if=0  streaming if=2  feature unit=22
  alt 0: ep=0x00 attr=0x00 mps=0 ch=0 bits=0 rate=0
  alt 1: ep=0x04 attr=0x09 mps=40 ch=2 bits=16 rate=8000
  ...
  alt 7: ep=0x04 attr=0x09 mps=248 ch=2 bits=16 rate=48000
selected alternate setting 7
```

This must match `./tools/usbcap.py descriptors <capture>.csv` exactly. If it
does not, the on-target descriptor walk differs from the host parser and the
discrepancy must be resolved before proceeding.

- [ ] **Step 2: Confirm SET_INTERFACE reaches the wire**

Capture with the Saleae while the board enumerates the headset, then:

Run: `./tools/usbcap.py summary <capture>.csv`

Expected: the SETUP transaction list includes `bmReq=0x01 bReq=0x0B
wValue=0x0007 wIndex=0x0002` — `SET_INTERFACE`, alternate setting 7, interface
2 — and it is followed by an ACK rather than a STALL.

- [ ] **Step 3: Record the result**

Append the observed serial output and the confirmed `SET_INTERFACE` transaction
to the spec's M1 acceptance criteria as evidence.

---

## Definition of done

- `cd test && make` reports 0 failures and exits 0
- `examples/Audio/UAC1Info` prints the topology matching `usbcap.py descriptors`
- `SET_INTERFACE` for interface 2, alternate setting 7 is visible on the analyzer and is ACKed
- Task 1 has returned a verdict on whether full-speed isochronous works through the embedded TT

Once these hold, the siTD transport plan (M2–M3) can be written against a proven
hardware premise.
