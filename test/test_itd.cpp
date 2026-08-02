// Copyright (c) 2026 Nicholas Newdigate
// SPDX-License-Identifier: MIT
#include "ehci_iso.h"
#include <stdio.h>
#include <stddef.h>

static int failures = 0, checks = 0;
#define CHECK_EQ(a, b) do { checks++; long _ck_a = (long)(a), _ck_b = (long)(b); \
	if (_ck_a != _ck_b) { printf("FAIL %s:%d: %s == %s (got %ld, want %ld)\n", \
	__FILE__, __LINE__, #a, #b, _ck_a, _ck_b); failures++; } } while (0)

static void test_itd_layout(void)
{
	// EHCI 1.0 section 3.3: next link, eight transaction words, seven
	// buffer page pointers -- 16 consecutive dwords, 32-byte aligned.
	CHECK_EQ(offsetof(itd_t, next), 0);
	CHECK_EQ(offsetof(itd_t, transaction), 4);
	CHECK_EQ(offsetof(itd_t, bufptr), 36);
	CHECK_EQ(sizeof(((itd_t *)0)->transaction), 32);
	CHECK_EQ(sizeof(((itd_t *)0)->bufptr), 28);
	CHECK_EQ(sizeof(itd_t) % 32, 0);
	CHECK_EQ(alignof(itd_t), 32);

	static itd_t pool[4];
	CHECK_EQ(((size_t)(void *)pool) % 32, 0);
	CHECK_EQ((char *)&pool[1] - (char *)&pool[0], (long)sizeof(itd_t));
}

int main(void)
{
	test_itd_layout();
	printf("%d checks, %d failures\n", checks, failures);
	return failures ? 1 : 0;
}
