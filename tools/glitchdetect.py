#!/usr/bin/env python3
# Copyright (c) 2026 Nicholas Newdigate
# SPDX-License-Identifier: MIT
"""Find phase discontinuities in a recorded steady tone, and correlate them
with the rate-bias sweep from a USBHost_t36 console log.

Why phase and not amplitude: the signal chain under test is a line output
driven into a microphone input, so it is heavily clipped. Clipping destroys
amplitude information but leaves zero-crossing *timing* intact, and a sample
drop or repeat at the far end shows up precisely as a shifted zero crossing.

Usage:
    glitchdetect.py selftest
    glitchdetect.py detect <recording.wav> [--tone 440] [--thresh 0.25]
    glitchdetect.py correlate <recording.wav> <console.log> [--start-epoch N]
"""

import argparse
import re
import sys
import wave

import numpy as np


def read_wav(path):
    """Return (mono float array, sample rate). Takes channel 0 if stereo."""
    with wave.open(path, "rb") as w:
        rate = w.getframerate()
        ch = w.getnchannels()
        width = w.getsampwidth()
        raw = w.readframes(w.getnframes())
    dtype = {1: np.uint8, 2: np.int16, 4: np.int32}.get(width)
    if dtype is None:
        raise SystemExit(f"unsupported sample width: {width} bytes")
    data = np.frombuffer(raw, dtype=dtype).astype(np.float64)
    if width == 1:
        data -= 128.0
    if ch > 1:
        data = data[::ch]
    peak = np.max(np.abs(data)) or 1.0
    return data / peak, rate


def zero_crossings(x, hysteresis=0.25):
    """Rising zero crossings, located to sub-sample precision.

    A Schmitt trigger keeps noise near zero from producing a burst of false
    crossings; the signal must fall below -hysteresis before another rising
    crossing counts. Positions are linearly interpolated between the bracketing
    samples, which matters because a whole-sample quantisation of the crossing
    would itself look like jitter of the size being measured.
    """
    armed = False
    out = []
    prev = x[0]
    for i in range(1, len(x)):
        cur = x[i]
        if not armed:
            if cur < -hysteresis:
                armed = True
        elif prev <= 0.0 < cur:
            frac = -prev / (cur - prev) if cur != prev else 0.0
            out.append(i - 1 + frac)
            armed = False
        prev = cur
    return np.array(out)


def find_glitches(x, rate, tone_hz=440.0, sigma=6.0, min_samples=4.0):
    """Phase discontinuities, as (sample_index, error_in_samples).

    The tone's period is measured from the recording rather than assumed, so a
    converter running slightly off nominal does not read as a constant error.
    A glitch is a single interval that departs from that period by more than
    both an absolute floor and a multiple of the observed jitter.
    """
    zc = zero_crossings(x)
    if len(zc) < 20:
        return [], 0.0
    intervals = np.diff(zc)
    period = np.median(intervals)
    err = intervals - period
    # Median absolute deviation: robust to the very outliers being detected,
    # unlike a standard deviation, which they would inflate into hiding.
    mad = np.median(np.abs(err - np.median(err))) * 1.4826
    thresh = max(min_samples, sigma * mad if mad > 0 else min_samples)
    hits = [(zc[i + 1], err[i]) for i in range(len(err)) if abs(err[i]) > thresh]
    measured_hz = rate / period if period else 0.0
    return hits, measured_hz


def merge(hits, rate, window_s=0.05):
    """Collapse hits closer together than window_s into one event."""
    merged = []
    for pos, err in hits:
        if merged and (pos - merged[-1][0]) < window_s * rate:
            if abs(err) > abs(merged[-1][1]):
                merged[-1] = (merged[-1][0], err)
        else:
            merged.append((pos, err))
    return merged


def cmd_detect(args):
    x, rate = read_wav(args.wav)
    dur = len(x) / rate
    hits, measured = find_glitches(x, rate, args.tone)
    events = merge(hits, rate)
    print(f"file      : {args.wav}")
    print(f"duration  : {dur:.1f} s at {rate} Hz")
    print(f"tone      : {measured:.3f} Hz measured (nominal {args.tone})")
    print(f"glitches  : {len(events)}")
    if events:
        print(f"mean gap  : {dur / len(events):.2f} s")
        print()
        print("      time(s)   phase error (samples)")
        prev = None
        for pos, err in events:
            t = pos / rate
            gap = "" if prev is None else f"  gap {t - prev:6.2f}s"
            print(f"    {t:9.3f}   {err:+8.2f}{gap}")
            prev = t
    return events


BIAS_RE = re.compile(r"BIAS\s+([+-]?\d+)\s*ppm")
UP_RE = re.compile(r"up=(\d+)s")


def cmd_correlate(args):
    """Map each detected glitch onto the bias value in force at the time.

    Alignment uses the console's own `up=` seconds. The recording's start is
    expressed in those same units via --start-uptime, which avoids depending on
    two clocks agreeing.
    """
    events = cmd_detect(args)
    x, rate = read_wav(args.wav)

    # Build (uptime_s, bias_ppm) from the log. BIAS lines carry no timestamp of
    # their own, so each is dated by the most recent heartbeat before it.
    schedule = []
    up = None
    with open(args.log, "r", errors="replace") as f:
        for line in f:
            m = UP_RE.search(line)
            if m:
                up = int(m.group(1))
            m = BIAS_RE.search(line)
            if m and up is not None:
                schedule.append((up, int(m.group(1))))
    if not schedule:
        print("\nno BIAS lines with a preceding heartbeat in the log")
        return

    print(f"\nbias schedule: {len(schedule)} steps, "
          f"{schedule[0][1]:+d} to {schedule[-1][1]:+d} ppm")

    if args.start_uptime is None:
        print("pass --start-uptime <seconds> (board uptime when recording began) "
              "to attribute glitches to steps")
        return

    counts = {}
    for pos, _err in events:
        t = args.start_uptime + pos / rate
        bias = None
        for up_s, b in schedule:
            if up_s <= t:
                bias = b
            else:
                break
        if bias is not None:
            counts[bias] = counts.get(bias, 0) + 1

    # Dwell per step, so a rate can be reported rather than a raw count.
    dwell = {}
    for i, (up_s, b) in enumerate(schedule):
        end = schedule[i + 1][0] if i + 1 < len(schedule) else up_s + args.dwell
        dwell[b] = dwell.get(b, 0) + (end - up_s)

    print("\n  bias(ppm)   glitches   dwell(s)   per minute")
    for b in sorted(set(list(counts) + list(dwell))):
        n = counts.get(b, 0)
        d = dwell.get(b, 0)
        rate_min = (n / d * 60) if d else 0.0
        mark = "  <-- quietest" if d and rate_min == 0 else ""
        print(f"  {b:+8d}   {n:8d}   {d:8d}   {rate_min:9.2f}{mark}")


def cmd_selftest(_args):
    """Validate the detector against signals whose glitches are known.

    Worth doing before trusting it on real audio: a detector that silently
    finds nothing looks exactly like a clean recording.
    """
    rate, tone, dur = 44100, 440.0, 6.0
    n = int(rate * dur)
    rng = np.random.default_rng(12345)
    fails = 0

    def check(name, ok, detail=""):
        nonlocal fails
        if not ok:
            fails += 1
        print(f"  {'PASS' if ok else 'FAIL'}  {name}{'  ' + detail if detail else ''}")

    def tone_with_drops(drop_positions, drop_len=30, clip=None, noise=0.0):
        t = np.arange(n + drop_len * len(drop_positions)) / rate
        sig = np.sin(2 * np.pi * tone * t)
        for p in sorted(drop_positions, reverse=True):
            i = int(p * rate)
            sig = np.concatenate([sig[:i], sig[i + drop_len:]])
        sig = sig[:n]
        if clip is not None:
            sig = np.clip(sig / clip, -1.0, 1.0)
        if noise:
            sig = sig + rng.normal(0, noise, len(sig))
        return sig

    clean = tone_with_drops([])
    hits, measured = find_glitches(clean, rate, tone)
    check("clean tone -> no glitches", len(merge(hits, rate)) == 0,
          f"found {len(merge(hits, rate))}")
    check("clean tone -> frequency recovered", abs(measured - tone) < 0.5,
          f"{measured:.3f} Hz")

    for label, sig, want in [
        ("single drop",      tone_with_drops([2.0]), [2.0]),
        ("three drops",      tone_with_drops([1.0, 3.0, 5.0]), [1.0, 3.0, 5.0]),
        ("clipped 10x",      tone_with_drops([2.5], clip=0.1), [2.5]),
        ("clipped + noise",  tone_with_drops([2.5], clip=0.1, noise=0.02), [2.5]),
        ("short drop (7)",   tone_with_drops([3.0], drop_len=7), [3.0]),
    ]:
        ev = merge(find_glitches(sig, rate, tone)[0], rate)
        times = sorted(p / rate for p, _ in ev)
        ok = len(times) == len(want) and all(
            abs(a - b) < 0.15 for a, b in zip(times, want))
        check(label, ok, f"at {[round(t, 3) for t in times]} want {want}")

    noisy = tone_with_drops([], clip=0.1, noise=0.03)
    ev = merge(find_glitches(noisy, rate, tone)[0], rate)
    check("clipped+noisy, no drops -> no false positives", len(ev) == 0,
          f"found {len(ev)}")

    # An off-nominal converter must not read as a continuous glitch.
    t = np.arange(n) / rate
    off = np.sin(2 * np.pi * (tone * 1.0002) * t)
    ev = merge(find_glitches(off, rate, tone)[0], rate)
    check("tone 200 ppm off nominal -> no glitches", len(ev) == 0,
          f"found {len(ev)}")

    print(f"\n{'all self-tests passed' if not fails else str(fails) + ' FAILED'}")
    return 1 if fails else 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    d = sub.add_parser("detect", help="list phase discontinuities in a recording")
    d.add_argument("wav")
    d.add_argument("--tone", type=float, default=440.0)
    d.set_defaults(func=cmd_detect)

    c = sub.add_parser("correlate", help="attribute glitches to rate-bias steps")
    c.add_argument("wav")
    c.add_argument("log")
    c.add_argument("--tone", type=float, default=440.0)
    c.add_argument("--start-uptime", type=float, default=None,
                   help="board uptime in seconds when the recording started")
    c.add_argument("--dwell", type=int, default=20,
                   help="seconds per bias step, for the final step's dwell")
    c.set_defaults(func=cmd_correlate)

    s = sub.add_parser("selftest", help="validate the detector on known signals")
    s.set_defaults(func=cmd_selftest)

    args = ap.parse_args()
    sys.exit(args.func(args) or 0)


if __name__ == "__main__":
    main()
