// Copyright (c) 2026 Nicholas Newdigate
// SPDX-License-Identifier: MIT
#include "ehci_iso.h"
#include <stdio.h>
#include <stddef.h>

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
	// Hardware requires each siTD to be 32-byte aligned, and an array of
	// them to have a 32-byte-multiple stride. aligned(32) gives both; assert
	// both, because padding the size alone would still leave a pool's base
	// address unaligned and the failure is silent.
	CHECK_EQ(sizeof(sitd_t) % 32, 0);
	CHECK_EQ(alignof(sitd_t), 32);

	// A pool of these must stay aligned end to end.
	static sitd_t pool[4];
	CHECK_EQ(((size_t)(void *)pool) % 32, 0);
	CHECK_EQ((char *)&pool[1] - (char *)&pool[0], (long)sizeof(sitd_t));
}

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

	// Bounds: iso is capped at 1023 bytes on full speed, and a packet must
	// not run past the end of the frame.
	CHECK_EQ(sitd_budget_out(1024, 0, &s, &c), false);
	CHECK_EQ(sitd_budget_out(192, 7, &s, &c), false);  // would need uframe 8
	CHECK_EQ(sitd_budget_out(0, 0, &s, &c), false);    // zero-length
	CHECK_EQ(sitd_budget_out(192, 0, 0, &c), false);   // null out-param
	CHECK_EQ(sitd_budget_out(192, 0, &s, 0), false);   // null out-param
}

int main(void)
{
	test_sitd_layout();
	test_budget_out();
	printf("%d checks, %d failures\n", checks, failures);
	return failures ? 1 : 0;
}
