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

int main(void)
{
	test_pack();
	if (failures == 0) { printf("test_pack: all %d checks passed\n", checks); return 0; }
	printf("test_pack: %d/%d FAILED\n", failures, checks);
	return 1;
}
