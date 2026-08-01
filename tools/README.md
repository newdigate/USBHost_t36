# tools

Host-side helpers for USB audio work on the i.MX RT1176. Not compiled into the
library.

## usbcap.py

Analyses a Saleae Logic 2 **"USB LS and FS"** analyzer export.

In Logic 2, add the *USB LS and FS* analyzer to the D+/D- channels, then export
the analyzer table (not the raw samples) to CSV. Columns should be
`name,type,start_time,duration,"value"`.

### Capture setup

Full speed is 12 Mbit/s, so the Logic 8 needs 100 MS/s to decode it reliably,
and that caps you at **three channels**: D+, D-, and one spare for a firmware
marker GPIO. (50 MS/s over six channels gives only ~4 samples/bit and decodes
unreliably; 25 MS/s over eight will not decode at all.)

Rather than probing the board, put a USB Type-A breakout inline between the host
port and the device -- the RT1176 `USB_OTG2_DP`/`USB_OTG2_DN` pins are dedicated
PHY pins with no GPIO alternative.

Note that **split tokens never appear in these captures**. SSPLIT/CSPLIT live on
the high-speed segment between the host controller and the transaction
translator; the RT1176's TT is inside the SoC, so its downstream side -- which is
what you are probing -- carries plain full-speed traffic. The capture shows the
*result* of the siTD schedule, not the schedule itself.

### Commands

    ./usbcap.py summary     cap.csv
    ./usbcap.py descriptors cap.csv
    ./usbcap.py iso         cap.csv [--endpoint 4] [--rate 48000]

**summary** -- packet-type counts, SOF cadence, per-endpoint token counts and
payload size histograms. Use it to find which endpoint carries the audio.

It reports the analyzer's **decode-error count first**, because a bad probe
does not look like a bad capture -- it looks like a *quiet* one. A capture that
is 99% `Error packet` still yields a plausible-looking handful of decoded
packets, because short handshakes occasionally fall out of noise by chance.
Always read the error line before believing anything below it.

Diagnosing a bad capture:

- **No SOF packets at all.** A full-speed host emits one every 1.000 ms,
  unconditionally. Their absence means the decode is broken, not the bus.
- **Handshakes (NAK/ACK) but no tokens.** A NAK is a *response*; it cannot occur
  without an IN. Seeing thousands of NAKs and zero IN tokens means only the
  shortest packets are surviving.
- **Errors every 1-2 bit times (83-166 ns at full speed), continuing while the
  bus is idle.** An idle bus produces no transitions and should produce no
  errors. This pattern is a floating input picking up noise -- check that BOTH
  D+ and D- are connected, that the channel mapping matches, and that ground is
  tied to the target.

Sample rate is a separate failure and looks different: too few samples per bit
degrades long packets first while short ones still decode. Rule the probe out
before blaming the rate.

**descriptors** -- recovers the configuration descriptor from the enumeration
control transfer and decodes the UAC1 topology: terminals, feature units, every
alternate setting with its format and rate, and endpoint sync type.

**iso** -- the one used to check a host implementation. Reports the payload size
histogram, effective delivered sample rate, 1 ms cadence, and frames where a
packet was expected but never sent. That last number is the interesting one: a
transfer-descriptor ring that refills too late shows up as missed frames.

### Reference trace

A known-good UAC1 playback trace (headset driven by a PC) reads:

    endpoint    : OUT addr=63 ep=4
    packets     : 8871 over 8.870 s
    sizes       : 192 B x8871
    target      : 48000 Hz, 2 ch, 16-bit -> 192 B/frame nominal
    delivered   : 1703232 B -> 48005.9 Hz effective (100.01% of target)
    cadence     : avg 1.0000 ms (min 0.9997, max 1.0003) over 8870 intervals
    gaps        : 0 gap(s) >= 2 ms, ~0 frame(s) with no packet

Constant packet size, no gaps, cadence within 0.3 us of nominal. That is the bar
for the RT1176 host implementation to match.

## Bring-up target

**48 kHz stereo 16-bit first**, 44.1 kHz afterwards.

48 kHz is exactly 192 bytes every frame with no variation. 44.1 kHz is 44.1
samples per frame, which means alternating 176- and 180-byte packets on a 9:1
pattern -- a packet-size scheduler whose bugs are hard to tell apart from
transfer-scheduling bugs on a trace. Bringing up at 48 kHz also allows a direct
diff against the reference trace above.

The test headset supports both: streaming interface 2 alt 7 is 48 kHz stereo and
alt 6 is 44.1 kHz stereo, and its output endpoint is **adaptive**, so it locks to
whatever rate the host sends. No feedback endpoint is required for playback. Its
microphone path is asynchronous and mono, which is the harder case and is why
capture is scheduled after playback.
