#!/usr/bin/env python3
# Copyright (c) 2026 Nicholas Newdigate
# SPDX-License-Identifier: MIT
"""
Analyse a Saleae Logic 2 "USB LS and FS" analyzer CSV export.

Export the analyzer table (not raw samples) to CSV; the expected columns are
    name,type,start_time,duration,"value"
with values of the form "Byte 0xNN", "EOP" and "Reset".

    usbcap.py summary     cap.csv
    usbcap.py descriptors cap.csv
    usbcap.py iso         cap.csv [--endpoint 4] [--rate 48000]

'iso' is the one used to check a host implementation: it reports the payload
size histogram, the 1 ms cadence, and -- most usefully -- frames where a
packet was expected but never sent, which is what a late transfer-descriptor
refill looks like on the wire.
"""

import argparse
import csv
import sys
from collections import Counter, defaultdict

PIDS = {
    0xA5: 'SOF', 0x2D: 'SETUP', 0x69: 'IN', 0xE1: 'OUT',
    0xC3: 'DATA0', 0x4B: 'DATA1', 0x0F: 'MDATA', 0x87: 'PING',
    0xD2: 'ACK', 0x5A: 'NAK', 0x1E: 'STALL', 0x3C: 'PRE/ERR',
    0x96: 'SPLIT',
}

TOKENS = ('IN', 'OUT', 'SETUP')
DATA = ('DATA0', 'DATA1', 'MDATA')

TERMINAL_TYPES = {
    0x0100: 'USB Undefined', 0x0101: 'USB Streaming',
    0x0201: 'Microphone', 0x0301: 'Speaker', 0x0302: 'Headphones',
    0x0402: 'Headset', 0x0603: 'Line Connector',
}
AC_SUBTYPES = {
    1: 'HEADER', 2: 'INPUT_TERMINAL', 3: 'OUTPUT_TERMINAL',
    4: 'MIXER_UNIT', 5: 'SELECTOR_UNIT', 6: 'FEATURE_UNIT',
    7: 'PROCESSING_UNIT', 8: 'EXTENSION_UNIT',
}
SYNC_TYPES = {0: 'none', 1: 'async', 2: 'adaptive', 3: 'sync'}
USAGE_TYPES = {0: 'data', 1: 'feedback', 2: 'implicit-feedback'}
XFER_TYPES = ('control', 'iso', 'bulk', 'interrupt')


class Packet:
    __slots__ = ('t', 'pid', 'body')

    def __init__(self, t, pid, body):
        self.t = t
        self.pid = pid
        self.body = body

    @property
    def addr_ep(self):
        """(address, endpoint) for a token packet, else None."""
        if self.pid not in TOKENS or len(self.body) < 2:
            return None
        v = self.body[0] | (self.body[1] << 8)
        return v & 0x7F, (v >> 7) & 0x0F

    @property
    def payload(self):
        """DATA payload with the trailing CRC16 removed."""
        return self.body[:-2] if len(self.body) >= 2 else b''


def parse(path):
    """Return (packets, resets, errors).

    'errors' counts frames the analyzer could not decode. These matter: a
    capture can look almost empty while actually being 97% undecodable, which
    reads as 'quiet bus' when it really means 'broken probe'. Never report a
    summary without them.
    """
    packets, resets, errors = [], [], 0
    cur, cur_t = bytearray(), None

    with open(path, newline='') as fh:
        reader = csv.reader(fh)
        header = next(reader, None)
        if not header or 'start_time' not in ','.join(header):
            sys.exit("error: %s does not look like a Saleae analyzer export" % path)

        for row in reader:
            if len(row) < 5:
                continue
            value = row[4]

            if value == 'Reset':
                resets.append((float(row[2]), float(row[3])))
                continue

            if value.startswith('Error'):
                errors += 1
                cur, cur_t = bytearray(), None   # abandon any partial packet
                continue

            if value == 'EOP':
                if cur:
                    body = cur[1:] if cur[0] == 0x80 else cur
                    if body:
                        packets.append(Packet(cur_t, PIDS.get(body[0], 'PID?%02X' % body[0]),
                                              bytes(body[1:])))
                cur, cur_t = bytearray(), None
                continue

            if value.startswith('Byte 0x'):
                if not cur:
                    cur_t = float(row[2])
                cur.append(int(value[7:], 16))

    return packets, resets, errors


def cmd_summary(packets, resets, args):
    errors = getattr(args, 'decode_errors', 0)
    if not packets:
        if errors:
            sys.exit("no packets decoded, but %d error frames -- the probe is not "
                     "seeing the bus correctly (check D+/D-/ground)" % errors)
        sys.exit("no packets decoded")

    decoded = len(packets)
    if errors:
        share = 100.0 * errors / max(1, errors + decoded)
        print("!! DECODE ERRORS: %d frames (%.1f%% of all frames) !!" % (errors, share))
        if share > 5.0:
            print("   The capture is mostly undecodable -- treat every figure below")
            print("   as unreliable. Check that BOTH D+ and D- are connected, that the")
            print("   channel mapping matches, and that ground is tied to the target.")
            print("   Errors arriving every 1-2 bit times during an idle bus mean a")
            print("   floating input picking up noise, not a busy bus.")
        print()
    print("resets      : %s" % ", ".join("%.4fs for %.4fs" % r for r in resets) or "none")
    print("span        : %.4f .. %.4f s (%.2f s)"
          % (packets[0].t, packets[-1].t, packets[-1].t - packets[0].t))
    print("packets     : %d" % len(packets))

    print("\npacket types:")
    for pid, n in Counter(p.pid for p in packets).most_common():
        print("  %-8s %d" % (pid, n))

    sofs = [p.t for p in packets if p.pid == 'SOF']
    if len(sofs) > 2:
        deltas = [b - a for a, b in zip(sofs, sofs[1:])]
        # ignore gaps where the capture was idle or the bus was reset
        frame = [d for d in deltas if d < 0.002]
        print("\nSOF         : %d, interval avg %.6f ms (min %.6f, max %.6f)"
              % (len(sofs), 1000 * sum(frame) / len(frame),
                 1000 * min(frame), 1000 * max(frame)))

    print("\nendpoints (token, addr, ep):")
    counts = Counter()
    for p in packets:
        ae = p.addr_ep
        if ae:
            counts[(p.pid,) + ae] += 1
    for (pid, addr, ep), n in counts.most_common(20):
        print("  %-6s addr=%-3d ep=%-2d %d" % (pid, addr, ep, n))

    print("\nDATA payload sizes by token:")
    sizes = defaultdict(Counter)
    last = None
    for p in packets:
        if p.pid in TOKENS:
            last = (p.pid,) + (p.addr_ep or (None, None))
        elif p.pid in DATA and last:
            sizes[last][len(p.payload)] += 1
    for key in sorted(sizes, key=lambda k: -sum(sizes[k].values()))[:10]:
        total = sum(sizes[key].values())
        top = ', '.join("%dB x%d" % (s, n) for s, n in sizes[key].most_common(5))
        print("  %-6s addr=%-3d ep=%-2d total=%-6d %s" % (key + (total, top)))


def _control_transfers(packets):
    """Yield (index, setup_bytes) for each SETUP transaction."""
    for i, p in enumerate(packets):
        if p.pid != 'SETUP':
            continue
        for j in range(i + 1, min(i + 4, len(packets))):
            if packets[j].pid in DATA:
                d = packets[j].payload
                if len(d) == 8:
                    yield i, j, d
                break


def _reassemble(packets, data_start, wlength):
    """Collect IN data-stage payloads following a control SETUP."""
    buf = bytearray()
    for p in packets[data_start + 1:]:
        if p.pid == 'SETUP':
            break
        if p.pid in DATA:
            buf += p.payload
            if len(buf) >= wlength:
                break
    return bytes(buf[:wlength])


def cmd_descriptors(packets, _resets, _args):
    best = None
    for setup_i, data_i, d in _control_transfers(packets):
        bm_request, b_request = d[0], d[1]
        w_value = d[2] | (d[3] << 8)
        w_length = d[6] | (d[7] << 8)
        # GET_DESCRIPTOR(CONFIGURATION), the full read rather than the 9-byte probe
        if bm_request == 0x80 and b_request == 0x06 and (w_value >> 8) == 0x02:
            if best is None or w_length > best[2]:
                best = (data_i, w_value, w_length)

    if not best:
        sys.exit("no GET_DESCRIPTOR(CONFIGURATION) found in capture")

    data_i, _w_value, w_length = best
    cfg = _reassemble(packets, data_i, w_length)
    print("configuration descriptor: %d of %d bytes recovered\n" % (len(cfg), w_length))
    if len(cfg) < w_length:
        print("warning: incomplete -- decode below may truncate\n")
    decode_config(cfg)


def decode_config(d):
    i = 0
    subclass = None
    while i + 1 < len(d):
        length, dtype = d[i], d[i + 1]
        if length == 0:
            break
        b = d[i:i + length]
        i += length

        if dtype == 0x02:
            print("CONFIGURATION: %d interfaces, %d mA, attributes 0x%02X"
                  % (b[4], b[8] * 2, b[7]))
        elif dtype == 0x04:
            subclass = b[6] if b[5] == 0x01 else None
            kind = {0x01: 'audio', 0x03: 'HID'}.get(b[5], '0x%02X' % b[5])
            sub = {1: 'control', 2: 'streaming'}.get(b[6], b[6]) if b[5] == 1 else b[6]
            print("\nINTERFACE %d alt %d: %d endpoint(s), %s/%s"
                  % (b[2], b[3], b[4], kind, sub))
        elif dtype == 0x24:
            _decode_cs_interface(b, subclass)
        elif dtype == 0x25:
            print("    CS_ENDPOINT: attributes 0x%02X (sampling-freq control %s)"
                  % (b[3], "yes" if b[3] & 0x01 else "no"))
        elif dtype == 0x05:
            attr = b[3]
            extra = ""
            if attr & 3 == 1:
                extra = " sync=%s usage=%s" % (SYNC_TYPES[(attr >> 2) & 3],
                                               USAGE_TYPES.get((attr >> 4) & 3, '?'))
            print("    ENDPOINT 0x%02X %s%s maxpacket=%d interval=%d"
                  % (b[2], XFER_TYPES[attr & 3], extra, b[4] | (b[5] << 8), b[6]))
        elif dtype == 0x21:
            print("    HID descriptor")


def _decode_cs_interface(b, subclass):
    subtype = b[2]
    if subclass == 1:      # AudioControl
        if subtype == 1:
            print("    AC HEADER: UAC %d.%02d, %d streaming interface(s) %s"
                  % (b[4], b[3], b[7], list(b[8:8 + b[7]])))
        elif subtype == 2:
            tt = b[4] | (b[5] << 8)
            print("    INPUT_TERMINAL id=%d type=0x%04X (%s) channels=%d"
                  % (b[3], tt, TERMINAL_TYPES.get(tt, 'unknown'), b[7]))
        elif subtype == 3:
            tt = b[4] | (b[5] << 8)
            print("    OUTPUT_TERMINAL id=%d type=0x%04X (%s) source=%d"
                  % (b[3], tt, TERMINAL_TYPES.get(tt, 'unknown'), b[7]))
        elif subtype == 6:
            size = b[5] or 1
            n = max(0, (len(b) - 7) // size)
            ctrls = [hex(int.from_bytes(b[6 + k * size:6 + (k + 1) * size], 'little'))
                     for k in range(n)]
            print("    FEATURE_UNIT id=%d source=%d controls=%s" % (b[3], b[4], ctrls))
        else:
            print("    AC %s" % AC_SUBTYPES.get(subtype, subtype))
    elif subclass == 2:    # AudioStreaming
        if subtype == 1:
            print("    AS_GENERAL: terminal=%d delay=%d format=0x%04X"
                  % (b[3], b[4], b[5] | (b[6] << 8)))
        elif subtype == 2:
            n = b[7]
            if n == 0:
                lo = b[8] | (b[9] << 8) | (b[10] << 16)
                hi = b[11] | (b[12] << 8) | (b[13] << 16)
                rates = "continuous %d..%d Hz" % (lo, hi)
            else:
                rates = ", ".join(str(b[8 + 3 * k] | (b[9 + 3 * k] << 8) | (b[10 + 3 * k] << 16))
                                  for k in range(n))
            print("    FORMAT_TYPE I: %d ch, %d-bit in %d byte(s), rates: %s"
                  % (b[4], b[6], b[5], rates))


def cmd_iso(packets, _resets, args):
    # Isochronous data transactions have no handshake, so identify candidate
    # endpoints by token counts and let --endpoint override.
    sizes = defaultdict(Counter)
    times = defaultdict(list)
    last = None
    for p in packets:
        if p.pid in TOKENS:
            last = (p.pid,) + (p.addr_ep or (None, None))
        elif p.pid in DATA and last and last[0] in ('IN', 'OUT'):
            sizes[last][len(p.payload)] += 1
            times[last].append(p.t)

    if not sizes:
        sys.exit("no data transactions found")

    if args.endpoint is not None:
        cands = [k for k in sizes if k[2] == args.endpoint]
        if not cands:
            sys.exit("endpoint %d carried no data in this capture" % args.endpoint)
        key = max(cands, key=lambda k: sum(sizes[k].values()))
    else:
        key = max(sizes, key=lambda k: sum(sizes[k].values()))

    pid, addr, ep = key
    ts = times[key]
    expected = args.rate * args.channels * args.width // 1000

    print("endpoint    : %s addr=%d ep=%d" % (pid, addr, ep))
    print("packets     : %d over %.3f s" % (len(ts), ts[-1] - ts[0] if len(ts) > 1 else 0))
    print("sizes       : %s"
          % ", ".join("%d B x%d" % (s, n) for s, n in sizes[key].most_common(6)))
    print("target      : %d Hz, %d ch, %d-bit -> %d B/frame nominal"
          % (args.rate, args.channels, args.width * 8, expected))

    total_bytes = sum(s * n for s, n in sizes[key].items())
    if len(ts) > 1:
        span = ts[-1] - ts[0]
        effective = total_bytes / (args.channels * args.width) / span
        print("delivered   : %d B -> %.1f Hz effective (%.2f%% of target)"
              % (total_bytes, effective, 100.0 * effective / args.rate))

    if len(ts) > 2:
        deltas = [b - a for a, b in zip(ts, ts[1:])]
        frame = [d for d in deltas if d < 0.002]
        if frame:
            print("cadence     : avg %.4f ms (min %.4f, max %.4f) over %d intervals"
                  % (1000 * sum(frame) / len(frame), 1000 * min(frame),
                     1000 * max(frame), len(frame)))
        gaps = [(a, b - a) for a, b in zip(ts, ts[1:]) if b - a >= 0.002]
        missed = sum(int(round(g / 0.001)) - 1 for _, g in gaps)
        print("gaps        : %d gap(s) >= 2 ms, ~%d frame(s) with no packet"
              % (len(gaps), missed))
        for t, g in gaps[:10]:
            print("    at %.5f s: %.3f ms (~%d frames)" % (t, 1000 * g, round(g / 0.001) - 1))
        if len(gaps) > 10:
            print("    ... %d more" % (len(gaps) - 10))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)

    for name, fn in (('summary', cmd_summary), ('descriptors', cmd_descriptors)):
        p = sub.add_parser(name)
        p.add_argument('capture')
        p.set_defaults(func=fn)

    p = sub.add_parser('iso')
    p.add_argument('capture')
    p.add_argument('--endpoint', type=int, default=None,
                   help='endpoint number (default: busiest data endpoint)')
    p.add_argument('--rate', type=int, default=48000, help='expected sample rate (default 48000)')
    p.add_argument('--channels', type=int, default=2, help='channel count (default 2)')
    p.add_argument('--width', type=int, default=2, help='bytes per sample (default 2)')
    p.set_defaults(func=cmd_iso)

    args = ap.parse_args()
    packets, resets, errors = parse(args.capture)
    args.decode_errors = errors
    args.func(packets, resets, args)


if __name__ == '__main__':
    main()
