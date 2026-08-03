// Copyright (c) 2026 Nicholas Newdigate
// SPDX-License-Identifier: MIT
#include "ehci_iso.h"
#include <stdio.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

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

// File-scope static so its address shares high bits with the itd/sitd pools
// (both static, hence BSS) on this 64-bit host -- see test_sitd.cpp's
// test_skip_iso comment for why sitd_skip_iso's pointer reconstruction
// depends on that.
static uint32_t slot;

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

// Refilling a descriptor that is still linked overwrites it in place, so the
// fill must leave every hardware dword it owns holding fresh state -- a field
// left stale is one the controller reads from the PREVIOUS packet. Poison the
// node first so "still stale" is visible rather than coincidentally right,
// and check the whole hardware area, not the fields a particular fill happens
// to care about. This is what guards the ACTIVE-last write ordering: reorder
// the stores and a dropped field shows up here.
static void test_fill_out_refill_leaves_nothing_stale(void)
{
	itd_pool_init();
	itd_t *n = itd_alloc();
	CHECK_EQ(n != NULL, true);

	memset(n, 0xA5, sizeof(*n));
	uint16_t lens[8] = {176, 0, 176, 176, 0, 176, 176, 176};
	CHECK_EQ(itd_fill_out(n, 3, 1, itd_buf, lens, 208, true), true);

	uint32_t page0 = ((uint32_t)(uintptr_t)itd_buf) & 0xFFFFF000u;
	uint32_t off = ((uint32_t)(uintptr_t)itd_buf) & 0xFFFu;
	for (int i = 0; i < 7; i++) {
		uint32_t want = page0 + (uint32_t)i * 4096u;
		if (i == 0) want |= (1u << 8) | 3u;
		if (i == 1) want |= 208u;
		if (i == 2) want |= 1u;
		CHECK_EQ(n->bufptr[i], want);
	}
	// Inactive microframes must be cleared to zero, not left poisoned:
	// a stale ACTIVE bit here is a phantom transaction.
	CHECK_EQ(n->transaction[1], 0u);
	CHECK_EQ(n->transaction[4], 0u);
	uint32_t cursor = off;
	for (int k = 0; k < 8; k++) {
		if (lens[k] == 0) continue;
		uint32_t want = ITD_TXN_ACTIVE
		              | ((uint32_t)lens[k] << ITD_TXN_LENGTH_SHIFT)
		              | ((cursor >> 12) << ITD_TXN_PG_SHIFT)
		              | (cursor & ITD_TXN_OFFSET_MASK);
		if (k == 7) want |= ITD_TXN_IOC;      // last active transaction
		CHECK_EQ(n->transaction[k], want);
		cursor += lens[k];
	}
	// `next` belongs to itd_link/itd_unlink, not to the fill, so it keeps
	// its poison -- the boundary is deliberate.
	CHECK_EQ(n->next, 0xA5A5A5A5u);
}

static void test_fill_in_refill_leaves_nothing_stale(void)
{
	static uint8_t buf[8] __attribute__ ((aligned(4)));
	itd_t n;
	memset(&n, 0xA5, sizeof(n));

	CHECK_EQ(itd_fill_in(&n, 9, 2, buf, 8, 4, false), true);

	uint32_t page0 = ((uint32_t)(uintptr_t)buf) & 0xFFFFF000u;
	uint32_t off = ((uint32_t)(uintptr_t)buf) & 0xFFFu;
	CHECK_EQ(n.transaction[0],
	         ITD_TXN_ACTIVE | (8u << ITD_TXN_LENGTH_SHIFT) | off);
	for (int k = 1; k < 8; k++) CHECK_EQ(n.transaction[k], 0u);
	CHECK_EQ(n.bufptr[0], page0 | (2u << 8) | 9u);
	CHECK_EQ(n.bufptr[1], (page0 + 4096u) | ITD_BUFPTR1_DIR_IN | 4u);
	CHECK_EQ(n.bufptr[2], (page0 + 8192u) | 1u);
	for (int i = 3; i < 7; i++)
		CHECK_EQ(n.bufptr[i], page0 + (uint32_t)i * 4096u);
	CHECK_EQ(n.next, 0xA5A5A5A5u);
}

// The untouched-on-rejection contract, checked across the WHOLE struct the
// way test_fill_in_rejects does. test_fill_out()'s snapshot covers the
// transaction/bufptr arrays only, which a mutant scribbling on bookkeeping
// would slip past.
static void test_fill_out_untouched_on_rejection(void)
{
	itd_t n, before;
	uint16_t lens[8] = {176, 176, 0, 0, 0, 0, 0, 0};

	memset(&n, 0xA5, sizeof(n));
	memcpy(&before, &n, sizeof(n));

	CHECK_EQ(itd_fill_out(NULL, 3, 1, itd_buf, lens, 208, false), false);
	CHECK_EQ(itd_fill_out(&n, 3, 1, NULL, lens, 208, false), false);
	CHECK_EQ(itd_fill_out(&n, 3, 1, itd_buf, NULL, 208, false), false);
	CHECK_EQ(itd_fill_out(&n, 3, 1, itd_buf, lens, 0, false), false);
	CHECK_EQ(itd_fill_out(&n, 3, 1, itd_buf, lens, 1025, false), false);
	CHECK_EQ(itd_fill_out(&n, 3, 16, itd_buf, lens, 208, false), false);
	CHECK_EQ(itd_fill_out(&n, 128, 1, itd_buf, lens, 208, false), false);
	uint16_t toolong[8] = {209, 0, 0, 0, 0, 0, 0, 0};
	CHECK_EQ(itd_fill_out(&n, 3, 1, itd_buf, toolong, 208, false), false);
	uint16_t allzero[8] = {0, 0, 0, 0, 0, 0, 0, 0};
	CHECK_EQ(itd_fill_out(&n, 3, 1, itd_buf, allzero, 208, false), false);
	CHECK_EQ(memcmp(&n, &before, sizeof(n)), 0);
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

static void test_link_unlink(void)
{
	itd_pool_init();
	itd_t *a = itd_alloc();
	itd_t *b = itd_alloc();
	CHECK_EQ(a != NULL && b != NULL, true);

	slot = 0x01u;                          // empty frame slot (terminate)
	itd_link(&slot, a, 7);
	CHECK_EQ(slot & 0x06u, 0x00u);         // type bits: iTD
	CHECK_EQ(slot & 0xFFFFFFE0u, (uint32_t)(uintptr_t)a & 0xFFFFFFE0u);
	CHECK_EQ(a->next, 0x01u);
	CHECK_EQ(a->frame, 7);

	// A second iTD goes in front; the first stays reachable behind it.
	itd_link(&slot, b, 7);
	CHECK_EQ((slot & 0xFFFFFFE0u), (uint32_t)(uintptr_t)b & 0xFFFFFFE0u);
	CHECK_EQ((b->next & 0xFFFFFFE0u), (uint32_t)(uintptr_t)a & 0xFFFFFFE0u);

	// skip_iso walks past both and lands where a QH would attach.
	CHECK_EQ((void *)sitd_skip_iso(&slot), (void *)&a->next);

	// Unlink tail then head; slot returns to terminate.
	CHECK_EQ(itd_unlink(&slot, a), true);
	CHECK_EQ((slot & 0xFFFFFFE0u), (uint32_t)(uintptr_t)b & 0xFFFFFFE0u);
	CHECK_EQ(b->next, 0x01u);
	CHECK_EQ(itd_unlink(&slot, b), true);
	CHECK_EQ(slot, 0x01u);
	CHECK_EQ(itd_unlink(&slot, a), false);

	// Null arguments are ignored, not dereferenced.
	itd_link(NULL, a, 0);
	itd_link(&slot, NULL, 0);
	CHECK_EQ(slot, 0x01u);
	CHECK_EQ(itd_unlink(NULL, a), false);
	CHECK_EQ(itd_unlink(&slot, NULL), false);

	// Mixed run: an siTD in front of an iTD unlinks independently.
	sitd_pool_init();
	sitd_t *s = sitd_alloc();
	itd_link(&slot, a, 9);
	sitd_link(&slot, s, 9);
	CHECK_EQ((void *)sitd_skip_iso(&slot), (void *)&a->next);
	CHECK_EQ(itd_unlink(&slot, a), true);        // unlink the iTD behind the siTD
	CHECK_EQ((slot & 0xFFFFFFE0u), (uint32_t)(uintptr_t)s & 0xFFFFFFE0u);
	CHECK_EQ(s->next, 0x01u);
	CHECK_EQ(sitd_unlink(&slot, s), true);
	CHECK_EQ(slot, 0x01u);
}

// Regression for the traversal-guard bound: a frame's iso run can legally
// hold nodes from BOTH pools at once, so a walker bounded by only one pool's
// size could exhaust its guard on a legal list and spuriously report "not
// found" (see ISO_RUN_GUARD in ehci_iso.cpp). Build the combined worst case
// -- every node from both pools chained into one slot -- and unlink the one
// buried deepest, which needs more traversal steps than either pool alone.
static void test_mixed_run_guard(void)
{
	itd_pool_init();
	sitd_pool_init();
	slot = 0x01u;

	// Drain the iTD pool first, linking each in turn, so these end up
	// deepest in the chain: the first one linked (itd_nodes[0]) is pushed
	// furthest from the head by every link that follows it.
	itd_t *itd_nodes[64];
	int n_itd = 0;
	itd_t *in;
	while (n_itd < 64 && (in = itd_alloc()) != NULL) {
		itd_link(&slot, in, 5);
		itd_nodes[n_itd++] = in;
	}

	// Then the whole siTD pool on top, so the run's head is all siTDs and
	// itd_nodes[0] sits behind all of them too.
	sitd_t *sitd_nodes[64];
	int n_sitd = 0;
	sitd_t *sn;
	while (n_sitd < 64 && (sn = sitd_alloc()) != NULL) {
		sitd_link(&slot, sn, 5);
		sitd_nodes[n_sitd++] = sn;
	}
	CHECK_EQ(n_itd > 0 && n_sitd > 0, true);

	// itd_nodes[0] is now (n_sitd + n_itd - 1) hops behind the head -- with
	// both pools at their documented size of 40 that is 79 hops, well past
	// either pool's size alone (the bug this guards against). A walker
	// still bounded by one pool's size would give up early and report false.
	CHECK_EQ(itd_unlink(&slot, itd_nodes[0]), true);
	CHECK_EQ(itd_nodes[0]->next, 0x01u);
	if (n_itd > 1) CHECK_EQ(itd_nodes[1]->next, 0x01u);
}

static void test_fill_in(void)
{
	static uint8_t buf[8] __attribute__ ((aligned(4)));
	itd_t n;
	// Pool-recycled nodes arrive dirty (an OUT node is freed with up to 8
	// live transactions), so the fill must be proven to clear every field it
	// does not program -- a fresh-zeroed node cannot detect a missing zeroing
	// loop. Poison with 0xA5 to make this explicit.
	memset(&n, 0xA5, sizeof(n));

	CHECK_EQ(itd_fill_in(&n, 9, 2, buf, 8, 4, false), true);

	// Exactly one transaction armed, in microframe 0: active, expected
	// length 8 (the buffer room; the device sends at most max_packet and
	// the controller writes the received count back), PG 0, offset =
	// buffer's low 12 bits, no IOC.
	uint32_t off = (uint32_t)(uintptr_t)buf & 0xFFFu;
	CHECK_EQ(n.transaction[0],
	         ITD_TXN_ACTIVE | (8u << ITD_TXN_LENGTH_SHIFT) | off);
	for (int k = 1; k < 8; k++) CHECK_EQ(n.transaction[k], 0u);

	// Page chain: page i = page 0 + i*4K; ep/dev in bufptr[0], DIRECTION
	// + max packet in bufptr[1], Multi=1 in bufptr[2].
	uint32_t page0 = (uint32_t)(uintptr_t)buf & 0xFFFFF000u;
	CHECK_EQ(n.bufptr[0], page0 | (2u << 8) | 9u);
	CHECK_EQ(n.bufptr[1], (page0 + 4096u) | ITD_BUFPTR1_DIR_IN | 4u);
	CHECK_EQ(n.bufptr[2], (page0 + 8192u) | 1u);
	for (int i = 3; i < 7; i++)
		CHECK_EQ(n.bufptr[i], page0 + (uint32_t)i * 4096u);

	// The direction bit is the whole difference between IN and OUT fills:
	// an OUT fill of the same node must leave bit 11 clear.
	uint16_t lens[8] = { 4, 0, 0, 0, 0, 0, 0, 0 };
	CHECK_EQ(itd_fill_out(&n, 9, 2, buf, lens, 4, false), true);
	CHECK_EQ(n.bufptr[1] & ITD_BUFPTR1_DIR_IN, 0u);

	// IOC lands on the single transaction when asked.
	CHECK_EQ(itd_fill_in(&n, 9, 2, buf, 8, 4, true), true);
	CHECK_EQ(n.transaction[0] & ITD_TXN_IOC, ITD_TXN_IOC);

	// Status read-back surfaces a completed IN's written-back length: the
	// controller clears ACTIVE and replaces the length field with the
	// received count (4 bytes of feedback inside an 8-byte expectation).
	n.transaction[0] = (4u << ITD_TXN_LENGTH_SHIFT);
	itd_txn_status_t st;
	itd_get_txn_status(&n, 0, &st);
	CHECK_EQ(st.active, false);
	CHECK_EQ(st.length, 4);
	CHECK_EQ(st.err_xact || st.err_babble || st.err_buffer, false);
}

static void test_fill_in_rejects(void)
{
	static uint8_t buf[8];
	itd_t n, before;

	// Every rejection leaves the descriptor untouched: fill with a
	// recognisable pattern and compare the whole struct afterwards.
	memset(&n, 0xA5, sizeof(n));
	memcpy(&before, &n, sizeof(n));

	CHECK_EQ(itd_fill_in(0, 9, 2, buf, 8, 4, false), false);
	CHECK_EQ(itd_fill_in(&n, 9, 2, 0, 8, 4, false), false);
	CHECK_EQ(itd_fill_in(&n, 9, 2, buf, 0, 4, false), false);     // len 0
	CHECK_EQ(itd_fill_in(&n, 9, 2, buf, 3073, 4, false), false);  // > uframe max
	CHECK_EQ(itd_fill_in(&n, 9, 2, buf, 8, 0, false), false);     // mps 0
	CHECK_EQ(itd_fill_in(&n, 9, 2, buf, 8, 1025, false), false);  // mps > 1024
	CHECK_EQ(itd_fill_in(&n, 9, 16, buf, 8, 4, false), false);    // endpoint width
	CHECK_EQ(itd_fill_in(&n, 128, 2, buf, 8, 4, false), false);   // address width
	CHECK_EQ(memcmp(&n, &before, sizeof(n)), 0);

	// Boundary acceptances, so the guards reject only what they claim:
	memset(&n, 0, sizeof(n));
	CHECK_EQ(itd_fill_in(&n, 127, 15, buf, 8, 1024, false), true);
	CHECK_EQ(itd_fill_in(&n, 9, 2, buf, 3072, 1024, false), true);
	CHECK_EQ(itd_fill_in(&n, 9, 2, buf, 1, 1, false), true);
}

int main(void)
{
	test_itd_layout();
	test_itd_pool();
	test_fill_out();
	test_txn_status();
	test_link_unlink();
	test_mixed_run_guard();
	test_fill_in();
	test_fill_in_rejects();
	test_fill_out_refill_leaves_nothing_stale();
	test_fill_in_refill_leaves_nothing_stale();
	test_fill_out_untouched_on_rejection();
	printf("%d checks, %d failures\n", checks, failures);
	return failures ? 1 : 0;
}
