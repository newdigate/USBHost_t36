// Copyright (c) 2026 Nicholas Newdigate
// SPDX-License-Identifier: MIT
#include "usb_audio2_parse.h"
#include <stdio.h>

static int failures = 0, checks = 0;
#define CHECK_EQ(a, b) do { checks++; long _ck_a = (long)(a), _ck_b = (long)(b); \
	if (_ck_a != _ck_b) { printf("FAIL %s:%d: %s == %s (got %ld, want %ld)\n", \
	__FILE__, __LINE__, #a, #b, _ck_a, _ck_b); failures++; } } while (0)

static void test_clock_cur_setup(void)
{
	// UAC2 5.2.5.1.1: SET CUR of CS_SAM_FREQ_CONTROL on clock entity 5,
	// AC interface 0; rate payload travels separately (4-byte LE).
	uint8_t s[8];
	uac2_clock_cur_setup(s, 0, 5);
	CHECK_EQ(s[0], 0x21);                  // class request, interface, H->D
	CHECK_EQ(s[1], 0x01);                  // CUR
	CHECK_EQ(s[2], 0x00); CHECK_EQ(s[3], 0x01);   // wValue: CS_SAM_FREQ<<8
	CHECK_EQ(s[4], 0x00); CHECK_EQ(s[5], 0x05);   // wIndex: (clockID<<8)|itf
	CHECK_EQ(s[6], 0x04); CHECK_EQ(s[7], 0x00);   // wLength 4

	// Non-zero AC interface lands in wIndex's low byte.
	uac2_clock_cur_setup(s, 3, 9);
	CHECK_EQ(s[4], 0x03);
	CHECK_EQ(s[5], 0x09);
}

int main(void)
{
	test_clock_cur_setup();
	if (failures == 0) { printf("test_uac2_parse: all %d checks passed\n", checks); return 0; }
	printf("test_uac2_parse: %d/%d FAILED\n", failures, checks);
	return 1;
}
