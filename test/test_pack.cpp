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

	// 2 stereo frames into 8ch 24-in-4: sample<<8 little-endian, 6 zeros.
	uint32_t n = uac_pack16(dst, src, 2, 2, 8, 4);
	CHECK_EQ(n, 64);
	CHECK_EQ(dst[0], 0x00); CHECK_EQ(dst[1], 0x34); CHECK_EQ(dst[2], 0x12); CHECK_EQ(dst[3], 0x00);
	CHECK_EQ(dst[4], 0x00); CHECK_EQ(dst[5], 0xDC); CHECK_EQ(dst[6], 0xFE); CHECK_EQ(dst[7], 0xFF);
	for (int i = 8; i < 32; i++) CHECK_EQ(dst[i], 0);   // channels 3..8 zero
	CHECK_EQ(dst[33], 0xFF); CHECK_EQ(dst[34], 0x7F);   // frame 2 left
	CHECK_EQ(dst[37], 0x00); CHECK_EQ(dst[38], 0x80);   // frame 2 right

	// 24-in-3 and native 16-in-2
	n = uac_pack16(dst, src, 1, 2, 2, 3);
	CHECK_EQ(n, 6);
	CHECK_EQ(dst[0], 0x00); CHECK_EQ(dst[1], 0x34); CHECK_EQ(dst[2], 0x12);
	n = uac_pack16(dst, src, 1, 2, 2, 2);
	CHECK_EQ(n, 4);
	CHECK_EQ(dst[0], 0x34); CHECK_EQ(dst[1], 0x12);

	CHECK_EQ(uac_pack16(dst, src, 1, 2, 2, 5), 0);      // unsupported subslot
}

int main(void)
{
	test_pack();
	if (failures == 0) { printf("test_pack: all %d checks passed\n", checks); return 0; }
	printf("test_pack: %d/%d FAILED\n", failures, checks);
	return 1;
}
