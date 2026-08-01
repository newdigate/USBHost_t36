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
	// Hardware requires 32-byte alignment; the software tail must not push
	// the struct past a 32-byte multiple.
	CHECK_EQ(sizeof(sitd_t) % 32, 0);
}

int main(void)
{
	test_sitd_layout();
	printf("%d checks, %d failures\n", checks, failures);
	return failures ? 1 : 0;
}
