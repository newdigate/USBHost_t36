// Copyright (c) 2026 Nicholas Newdigate
// SPDX-License-Identifier: MIT
#include "ehci_iso.h"
#include <stdio.h>
#include <stddef.h>
#include <stdint.h>

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

static void test_itd_pool(void)
{
	itd_pool_init();
	itd_t *seen[64];
	int n = 0;
	itd_t *node;
	while (n < 64 && (node = itd_alloc()) != NULL) {
		CHECK_EQ(((uintptr_t)node) % 32, 0);
		for (int i = 0; i < n; i++) CHECK_EQ(node == seen[i], false);
		seen[n++] = node;
	}
	CHECK_EQ(n, 40);                       // spec section 4: 40-node pool
	CHECK_EQ((void *)itd_alloc(), (void *)0);
	itd_free(seen[0]);
	CHECK_EQ((void *)itd_alloc(), (void *)seen[0]);
}

static uint8_t itd_buf[8192] __attribute__ ((aligned(32)));

static void test_fill_out(void)
{
	itd_pool_init();
	itd_t *n = itd_alloc();
	CHECK_EQ(n != NULL, true);

	// Eight mixed-size transactions; lengths chosen to exercise the
	// offset math.
	uint16_t lens[8] = {176, 208, 176, 176, 208, 176, 176, 208};
	CHECK_EQ(itd_fill_out(n, 3, 1, itd_buf, lens, 208, false), true);

	// bufptr low bits: ep/addr in [0], max packet + direction(OUT) in
	// [1], Multi=1 in [2] (EHCI Table 3-6).
	CHECK_EQ(n->bufptr[0] & 0x7F, 3);              // device address
	CHECK_EQ((n->bufptr[0] >> 8) & 0x0F, 1);       // endpoint
	CHECK_EQ(n->bufptr[1] & 0x7FF, 208);           // max packet size
	CHECK_EQ((n->bufptr[1] >> 11) & 1, 0);         // direction OUT (EHCI: I/O bit lives in bufptr[1])
	CHECK_EQ(n->bufptr[2] & 3, 1);                 // Multi = 1
	CHECK_EQ((n->bufptr[2] >> 2) & 0x3FF, 0);      // bufptr[2] low bits: Multi only
	// page pointers are consecutive 4K pages of one contiguous buffer
	CHECK_EQ(n->bufptr[0] & 0xFFFFF000u, ((uint32_t)(uintptr_t)itd_buf) & 0xFFFFF000u);
	CHECK_EQ(n->bufptr[1] & 0xFFFFF000u, ((((uint32_t)(uintptr_t)itd_buf) & 0xFFFFF000u) + 4096u));

	// transaction 0: active, len 176, PG 0, offset = buf offset in page
	uint32_t t0 = n->transaction[0];
	CHECK_EQ(t0 & ITD_TXN_ACTIVE, ITD_TXN_ACTIVE);
	CHECK_EQ((t0 >> ITD_TXN_LENGTH_SHIFT) & 0xFFF, 176);
	CHECK_EQ((t0 >> ITD_TXN_PG_SHIFT) & 7, 0);
	CHECK_EQ(t0 & ITD_TXN_OFFSET_MASK, ((uint32_t)(uintptr_t)itd_buf) & 0xFFF);
	CHECK_EQ(t0 & ITD_TXN_IOC, 0);

	// cumulative offsets: transaction 1 starts 176 bytes in
	uint32_t addr1 = (((uint32_t)(uintptr_t)itd_buf) & 0xFFF) + 176;
	CHECK_EQ((n->transaction[1] >> ITD_TXN_PG_SHIFT) & 7, addr1 >> 12);
	CHECK_EQ(n->transaction[1] & ITD_TXN_OFFSET_MASK, addr1 & 0xFFF);

	// a zero-length microframe stays inactive
	uint16_t lens2[8] = {176, 0, 176, 176, 176, 176, 176, 176};
	CHECK_EQ(itd_fill_out(n, 3, 1, itd_buf, lens2, 208, true), true);
	CHECK_EQ(n->transaction[1], 0);
	// ioc lands on the LAST active transaction
	CHECK_EQ(n->transaction[7] & ITD_TXN_IOC, ITD_TXN_IOC);

	// rejections: len > max packet, mps > 1024, nulls, endpoint/address
	// field widths
	uint16_t bad[8] = {209, 0, 0, 0, 0, 0, 0, 0};
	CHECK_EQ(itd_fill_out(n, 3, 1, itd_buf, bad, 208, false), false);
	CHECK_EQ(itd_fill_out(n, 3, 1, itd_buf, lens, 1025, false), false);
	CHECK_EQ(itd_fill_out(n, 3, 1, NULL, lens, 208, false), false);
	CHECK_EQ(itd_fill_out(NULL, 3, 1, itd_buf, lens, 208, false), false);
	CHECK_EQ(itd_fill_out(n, 3, 16, itd_buf, lens, 208, false), false);
	CHECK_EQ(itd_fill_out(n, 128, 1, itd_buf, lens, 208, false), false);

	// The rejection contract is "descriptor untouched": a fill that fails
	// PAST valid entries must not have scribbled on the node first. (A
	// mutant writing transactions progressively passes every check above.)
	CHECK_EQ(itd_fill_out(n, 3, 1, itd_buf, lens, 208, false), true);
	uint32_t snap_txn[8], snap_buf[7];
	for (int i = 0; i < 8; i++) snap_txn[i] = n->transaction[i];
	for (int i = 0; i < 7; i++) snap_buf[i] = n->bufptr[i];
	uint16_t midbad[8] = {176, 176, 5000, 176, 176, 176, 176, 176};
	CHECK_EQ(itd_fill_out(n, 3, 1, itd_buf, midbad, 208, false), false);
	for (int i = 0; i < 8; i++) CHECK_EQ(n->transaction[i], snap_txn[i]);
	for (int i = 0; i < 7; i++) CHECK_EQ(n->bufptr[i], snap_buf[i]);

	// IOC must land on the last ACTIVE transaction, not literally slot 7.
	uint16_t lead5[8] = {176, 176, 176, 176, 176, 0, 0, 0};
	CHECK_EQ(itd_fill_out(n, 3, 1, itd_buf, lead5, 208, true), true);
	CHECK_EQ(n->transaction[4] & ITD_TXN_IOC, ITD_TXN_IOC);
	CHECK_EQ(n->transaction[4] & ITD_TXN_ACTIVE, ITD_TXN_ACTIVE);
	CHECK_EQ(n->transaction[5], 0);
	CHECK_EQ(n->transaction[7], 0);

	// Force a genuine page crossing: start 100 bytes before a 4K boundary
	// inside the buffer, so transaction 1 must select PG 1.
	uint8_t *near_end = itd_buf +
		((4096u - ((uintptr_t)itd_buf & 0xFFFu) - 100u) & 0xFFFu);
	uint16_t cross[8] = {176, 176, 0, 0, 0, 0, 0, 0};
	CHECK_EQ(itd_fill_out(n, 3, 1, near_end, cross, 208, false), true);
	CHECK_EQ((n->transaction[0] >> ITD_TXN_PG_SHIFT) & 7, 0);
	CHECK_EQ((n->transaction[1] >> ITD_TXN_PG_SHIFT) & 7, 1);
	CHECK_EQ(n->transaction[1] & ITD_TXN_OFFSET_MASK, (((uintptr_t)near_end & 0xFFFu) + 176u) & 0xFFFu);
}

static void test_txn_status(void)
{
	itd_pool_init();
	itd_t *n = itd_alloc();
	uint16_t lens[8] = {176, 176, 176, 176, 176, 176, 176, 176};
	CHECK_EQ(itd_fill_out(n, 3, 1, itd_buf, lens, 208, false), true);

	itd_txn_status_t st;
	itd_get_txn_status(n, 0, &st);
	CHECK_EQ(st.active, true);
	CHECK_EQ(st.err_xact || st.err_babble || st.err_buffer, false);
	CHECK_EQ(st.length, 176);

	// controller retires transaction 2 with a babble error
	n->transaction[2] = (n->transaction[2] & ~ITD_TXN_ACTIVE) | ITD_TXN_ERR_BABBLE;
	itd_get_txn_status(n, 2, &st);
	CHECK_EQ(st.active, false);
	CHECK_EQ(st.err_babble, true);

	itd_get_txn_status(NULL, 0, &st);      // nulls tolerated
	CHECK_EQ(st.active, false);
	itd_get_txn_status(n, 8, &st);         // txn index out of range
	CHECK_EQ(st.active, false);
}

int main(void)
{
	test_itd_layout();
	test_itd_pool();
	test_fill_out();
	test_txn_status();
	printf("%d checks, %d failures\n", checks, failures);
	return failures ? 1 : 0;
}
