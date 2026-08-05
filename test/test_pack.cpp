// Copyright (c) 2026 Nicholas Newdigate
// SPDX-License-Identifier: MIT
#include "usb_audio_parse.h"
#include <stdio.h>

static int failures = 0, checks = 0;
#define CHECK_EQ(a, b) do { checks++; long _ck_a = (long)(a), _ck_b = (long)(b); \
	if (_ck_a != _ck_b) { printf("FAIL %s:%d: %s == %s (got %ld, want %ld)\n", \
	__FILE__, __LINE__, #a, #b, _ck_a, _ck_b); failures++; } } while (0)

static void test_pack(void)
{
	int16_t src[4] = {0x1234, (int16_t)0xFEDC, 0x7FFF, (int16_t)0x8000};
	uint8_t dst[64];

	// 2 stereo frames into 8ch 24-in-4, left-justified per UAC: the pad
	// bytes sit at the LOW end of each subslot, the sign is simply the
	// sample's own high byte.
	uint32_t n = uac_pack16(dst, src, 2, 2, 8, 4);
	CHECK_EQ(n, 64);
	CHECK_EQ(dst[0], 0x00); CHECK_EQ(dst[1], 0x00); CHECK_EQ(dst[2], 0x34); CHECK_EQ(dst[3], 0x12);
	CHECK_EQ(dst[4], 0x00); CHECK_EQ(dst[5], 0x00); CHECK_EQ(dst[6], 0xDC); CHECK_EQ(dst[7], 0xFE);
	for (int i = 8; i < 32; i++) CHECK_EQ(dst[i], 0);   // channels 3..8 zero
	CHECK_EQ(dst[32], 0x00); CHECK_EQ(dst[33], 0x00); CHECK_EQ(dst[34], 0xFF); CHECK_EQ(dst[35], 0x7F);
	CHECK_EQ(dst[36], 0x00); CHECK_EQ(dst[37], 0x00); CHECK_EQ(dst[38], 0x00); CHECK_EQ(dst[39], 0x80);
	for (int i = 40; i < 64; i++) CHECK_EQ(dst[i], 0);

	// 24-in-3: both channels pinned, including the negative one.
	n = uac_pack16(dst, src, 1, 2, 2, 3);
	CHECK_EQ(n, 6);
	CHECK_EQ(dst[0], 0x00); CHECK_EQ(dst[1], 0x34); CHECK_EQ(dst[2], 0x12);
	CHECK_EQ(dst[3], 0x00); CHECK_EQ(dst[4], 0xDC); CHECK_EQ(dst[5], 0xFE);

	// native 16-in-2
	n = uac_pack16(dst, src, 1, 2, 2, 2);
	CHECK_EQ(n, 4);
	CHECK_EQ(dst[0], 0x34); CHECK_EQ(dst[1], 0x12);
	CHECK_EQ(dst[2], 0xDC); CHECK_EQ(dst[3], 0xFE);

	// guard branches
	CHECK_EQ(uac_pack16(dst, src, 1, 2, 2, 5), 0);      // unsupported subslot
	CHECK_EQ(uac_pack16(dst, src, 1, 2, 2, 1), 0);
	CHECK_EQ(uac_pack16(NULL, src, 1, 2, 2, 4), 0);
	CHECK_EQ(uac_pack16(dst, NULL, 1, 2, 2, 4), 0);
	CHECK_EQ(uac_pack16(dst, src, 1, 4, 2, 4), 0);      // ch_live > ch_total
}

// --- cooperative-mode pattern ------------------------------------------
//
// The strongest check here is a MIRROR of the device's checker (decouple.xc,
// uacvCheckSample): sync on the seed, then advance-and-compare every sample.
// If the mirror locks once and counts zero errors over a multi-call packed
// stream, the wire contract holds end to end. Byte-level pins alone would
// pass a generator that is self-consistent but disagrees with the device --
// the exact class of defect that made the 16-bit UAC1 witness look like a
// host that never tried.

struct DeviceMirror {
	uint32_t lfsr = 0;
	bool synced = false;
	int run = 0;
	int syncs = 0, errs = 0;

	uint32_t next(void) {
		uint32_t lsb = lfsr & 1u;
		lfsr >>= 1;
		if (lsb) lfsr ^= UACV_PATTERN_TAPS;
		return lfsr << 8;
	}
	void sample(uint32_t s) {
		if (!synced) {
			if ((s & 0xFFFFFF00u) == ((UACV_PATTERN_SEED << 8) & 0xFFFFFF00u)) {
				lfsr = UACV_PATTERN_SEED;
				synced = true;
				run = 0;
				syncs++;
			}
			return;
		}
		if ((s & 0xFFFFFF00u) == (next() & 0xFFFFFF00u)) { run = 0; return; }
		errs++;
		if (++run >= 8) { synced = false; run = 0; }
	}
	// Feed a packed buffer the way the device unpacks it: consecutive
	// little-endian subslots, each one sample.
	void feed(const uint8_t *buf, uint32_t bytes, uint8_t subslot) {
		for (uint32_t i = 0; i + subslot <= bytes; i += subslot) {
			uint32_t s = 0;
			// 3-byte subslots left-justify into the top of the word with
			// the low byte clear, exactly as lib_xua's 3-byte path does.
			for (uint8_t b = 0; b < subslot; b++)
				s |= (uint32_t)buf[i + b] << (8 * (b + 4 - subslot));
			sample(s);
		}
	}
};

static void test_pattern(void)
{
	uint8_t dst[8 * 4 * 8];
	uint32_t lfsr = 0;
	bool primed = false;

	// Seed first, byte layout pinned: the device locks on 0x00ACE100, which
	// in a 4-byte subslot is little-endian {00, E1, AC, 00}.
	uint32_t n = uacv_pack_pattern(dst, 1, 2, 4, &lfsr, &primed);
	CHECK_EQ(n, 8);
	CHECK_EQ(primed, true);
	CHECK_EQ(dst[0], 0x00); CHECK_EQ(dst[1], 0xE1); CHECK_EQ(dst[2], 0xAC); CHECK_EQ(dst[3], 0x00);
	// Second sample (channel 2 of the same frame) is already the NEXT value
	// -- one fresh value per sample, not per frame. 0xACE1 advances to
	// 0xB4BC852C, emitted as (lfsr << 8) = 0xBC852C00 -> {00, 2C, 85, BC}.
	CHECK_EQ(dst[4], 0x00); CHECK_EQ(dst[5], 0x2C); CHECK_EQ(dst[6], 0x85); CHECK_EQ(dst[7], 0xBC);

	// Device-mirror over a multi-call stream: 8 channels, varying frame
	// counts per call like the servo's 5/6-frame microframes. One lock,
	// zero errors, and the mirror must actually have consumed samples.
	lfsr = 0; primed = false;
	DeviceMirror m;
	static const uint32_t frame_counts[] = {5, 6, 5, 5, 6, 8, 1, 6};
	for (unsigned k = 0; k < sizeof(frame_counts) / sizeof(frame_counts[0]); k++) {
		n = uacv_pack_pattern(dst, frame_counts[k], 8, 4, &lfsr, &primed);
		CHECK_EQ(n, frame_counts[k] * 8 * 4);
		m.feed(dst, n, 4);
	}
	CHECK_EQ(m.syncs, 1);
	CHECK_EQ(m.errs, 0);
	CHECK_EQ(m.synced, true);

	// 3-byte subslots carry the same sequence: fresh generator, same mirror.
	lfsr = 0; primed = false;
	DeviceMirror m3;
	for (int k = 0; k < 4; k++) {
		n = uacv_pack_pattern(dst, 6, 8, 3, &lfsr, &primed);
		CHECK_EQ(n, 6 * 8 * 3);
		m3.feed(dst, n, 3);
	}
	CHECK_EQ(m3.syncs, 1);
	CHECK_EQ(m3.errs, 0);

	// A dropped microframe IS detected: skip one packed buffer and the
	// mirror must record errors -- this is the defect R7 exists to catch,
	// so a test suite where it cannot fire proves nothing.
	lfsr = 0; primed = false;
	DeviceMirror md;
	n = uacv_pack_pattern(dst, 6, 8, 4, &lfsr, &primed);
	md.feed(dst, n, 4);
	n = uacv_pack_pattern(dst, 6, 8, 4, &lfsr, &primed);   // lost on the wire
	n = uacv_pack_pattern(dst, 6, 8, 4, &lfsr, &primed);
	md.feed(dst, n, 4);
	CHECK_EQ(md.syncs, 1);
	if (md.errs == 0) { checks++; failures++;
		printf("FAIL %s:%d: dropped packet went undetected\n", __FILE__, __LINE__); }

	// Guards: a 16-bit subslot cannot carry the sequence and must refuse,
	// not truncate -- truncation is precisely the silent non-pattern payload
	// the fallback counter exists to make visible.
	lfsr = 0; primed = false;
	CHECK_EQ(uacv_pack_pattern(dst, 1, 2, 2, &lfsr, &primed), 0);
	CHECK_EQ(primed, false);                     // a refusal must not burn the seed
	CHECK_EQ(uacv_pack_pattern(dst, 1, 2, 5, &lfsr, &primed), 0);
	CHECK_EQ(uacv_pack_pattern(NULL, 1, 2, 4, &lfsr, &primed), 0);
	CHECK_EQ(uacv_pack_pattern(dst, 1, 2, 4, NULL, &primed), 0);
	CHECK_EQ(uacv_pack_pattern(dst, 1, 2, 4, &lfsr, NULL), 0);
}

static void test_unpack(void)
{
	// One MC200 capture frame: 8 channels of 24-in-4. Hand-built rather
	// than round-tripped, so the byte order is asserted against the layout
	// and not merely against uac_pack16 agreeing with itself.
	uint8_t src[64];
	for (int c = 0; c < 8; c++) {
		src[c*4 + 0] = 0xAA;                    // pad, must be ignored
		src[c*4 + 1] = (uint8_t)(0x10 + c);     // low 8 of the 24, dropped
		src[c*4 + 2] = (uint8_t)(0x20 + c);     // -> int16 low byte
		src[c*4 + 3] = (uint8_t)(0x30 + c);     // -> int16 high byte
	}
	int16_t dst[16];
	CHECK_EQ(uac_unpack16(dst, src, 1, 8, 8, 4), 8);
	for (int c = 0; c < 8; c++)
		CHECK_EQ((uint16_t)dst[c], (uint16_t)(0x3020 + c * 0x0101));

	// Taking fewer channels than the device sends must SKIP the rest, not
	// read them contiguously: channel 1 of frame 0 is at byte 4, and the
	// second frame starts at byte 32 -- not byte 8.
	uint8_t two[64];
	for (int f = 0; f < 2; f++)
		for (int c = 0; c < 8; c++) {
			two[f*32 + c*4 + 0] = 0; two[f*32 + c*4 + 1] = 0;
			two[f*32 + c*4 + 2] = (uint8_t)(f * 16 + c);
			two[f*32 + c*4 + 3] = 0;
		}
	int16_t st[4];
	CHECK_EQ(uac_unpack16(st, two, 2, 2, 8, 4), 4);
	CHECK_EQ(st[0], 0); CHECK_EQ(st[1], 1);     // frame 0, channels 0..1
	CHECK_EQ(st[2], 16); CHECK_EQ(st[3], 17);   // frame 1 -- 32 bytes on

	// 16-in-2 is verbatim little-endian; the negative value pins the sign.
	uint8_t s2[4] = {0xDC, 0xFE, 0x34, 0x12};
	int16_t d2[2];
	CHECK_EQ(uac_unpack16(d2, s2, 1, 2, 2, 2), 2);
	CHECK_EQ((uint16_t)d2[0], 0xFEDC);
	CHECK_EQ((uint16_t)d2[1], 0x1234);

	// 24-in-3: the pad is at the low end, so the top two bytes are 1 and 2.
	uint8_t s3[3] = {0x99, 0xDC, 0xFE};
	int16_t d3[1];
	CHECK_EQ(uac_unpack16(d3, s3, 1, 1, 1, 3), 1);
	CHECK_EQ((uint16_t)d3[0], 0xFEDC);

	// Round trip through uac_pack16 for every subslot the device may use.
	// Exact in this direction (pack zeroes the bits unpack drops); the
	// reverse is not, which is why it is not asserted.
	const int16_t want[6] = {0x1234, (int16_t)0xFEDC, 0x7FFF,
	                         (int16_t)0x8000, 0x0001, (int16_t)0xFFFF};
	for (uint8_t subslot = 2; subslot <= 4; subslot++) {
		uint8_t packed[8 * 4 * 3];
		int16_t back[6];
		CHECK_EQ(uac_pack16(packed, want, 3, 2, 2, subslot), 3u * 2 * subslot);
		CHECK_EQ(uac_unpack16(back, packed, 3, 2, 2, subslot), 6);
		for (int i = 0; i < 6; i++) CHECK_EQ(back[i], want[i]);
	}

	// Guards. ch_want > ch_total refuses rather than zero-filling: a device
	// sending fewer channels than negotiated is a broken negotiation, and
	// silence in the extra channels would make it look like working audio.
	int16_t d[8];
	uint8_t s[64] = {0};
	CHECK_EQ(uac_unpack16(d, s, 1, 3, 2, 4), 0);
	CHECK_EQ(uac_unpack16(d, s, 1, 2, 0, 4), 0);
	CHECK_EQ(uac_unpack16(d, s, 1, 2, 2, 1), 0);
	CHECK_EQ(uac_unpack16(d, s, 1, 2, 2, 5), 0);
	CHECK_EQ(uac_unpack16(NULL, s, 1, 2, 2, 4), 0);
	CHECK_EQ(uac_unpack16(d, NULL, 1, 2, 2, 4), 0);
	CHECK_EQ(uac_unpack16(d, s, 0, 2, 2, 4), 0);   // no frames, no samples
}

int main(void)
{
	test_pack();
	test_unpack();
	test_pattern();
	if (failures == 0) { printf("test_pack: all %d checks passed\n", checks); return 0; }
	printf("test_pack: %d/%d FAILED\n", failures, checks);
	return 1;
}
