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

// Periodic frame list entries: bit 0 = terminate, bits 2:1 = type
// (0=iTD, 1=QH, 2=siTD, 3=FSTN), address = value & 0xFFFFFFE0.
#define LINK_TERMINATE 0x01u
#define LINK_TYPE_ITD  0x00u
#define LINK_TYPE_QH   0x02u
#define LINK_TYPE_SITD 0x04u

// --- 64-bit host vs. 32-bit target pointer widths ---
//
// sitd_skip_iso() reads a 32-bit hardware link word and follows it by
// reconstructing a pointer from its address bits (31:5). On the real
// target, uintptr_t is 32 bits, so those bits *are* the whole address and
// no reconstruction is needed. On this 64-bit host, a bare 32-bit value
// cannot hold a full pointer, so sitd_skip_iso() recovers the missing high
// bits from the pointer it just read *through* (see ehci_iso.cpp), on the
// assumption that the linked descriptor lives in the same region of
// address space as the link field pointing at it. That assumption holds
// unconditionally on target (flat 32-bit address space, no high bits to
// disagree). To make it hold here too, every node in a chain below --
// including the "frame_*" variables that stand in for periodic-table
// entries -- is `static`, so they all land in the process's contiguous
// BSS region and share the same high 32 bits. Plain stack locals would
// NOT work: this host's stack lives in a completely different region of
// the address space than BSS (verified: static addresses here are
// ~0x1xxxxxxxx while stack addresses are ~0x7ffxxxxxxxxx), so the
// high-bits splice would silently reconstruct a wild pointer. So this is
// a real exercise of the reconstruction logic, not a trivial pass.
static uint32_t frame_a, frame_b, frame_c, frame_d, frame_e;
static uint32_t sitd_node[8] __attribute__ ((aligned(32)));
static uint32_t sitd_a[8]    __attribute__ ((aligned(32)));
static uint32_t sitd_b[8]    __attribute__ ((aligned(32)));
static uint32_t itd_node[8]  __attribute__ ((aligned(32)));
static uint32_t frame_f;
static uint32_t cyc[8]       __attribute__ ((aligned(32)));
static uint32_t slot;

static void test_skip_iso(void)
{
	// A frame whose head is already a queue head: nothing to skip. The
	// address bits are never dereferenced for a QH entry (the type check
	// short-circuits first), so they can be left zero.
	frame_a = LINK_TYPE_QH;
	CHECK_EQ((void *)sitd_skip_iso(&frame_a), (void *)&frame_a);

	// An empty frame (terminate bit set): nothing to skip.
	frame_b = LINK_TERMINATE;
	CHECK_EQ((void *)sitd_skip_iso(&frame_b), (void *)&frame_b);

	// One siTD at the head, then a queue head: must return the address of
	// the siTD's own next field, which is where the QH chain begins.
	sitd_node[0] = LINK_TERMINATE;
	frame_c = ((uint32_t)(uintptr_t)sitd_node) | LINK_TYPE_SITD;
	CHECK_EQ((void *)sitd_skip_iso(&frame_c), (void *)&sitd_node[0]);

	// Two isochronous descriptors chained, then the QH chain.
	sitd_b[0] = LINK_TERMINATE;
	sitd_a[0] = ((uint32_t)(uintptr_t)sitd_b) | LINK_TYPE_SITD;
	frame_d = ((uint32_t)(uintptr_t)sitd_a) | LINK_TYPE_SITD;
	CHECK_EQ((void *)sitd_skip_iso(&frame_d), (void *)&sitd_b[0]);

	// An iTD is skipped the same way as an siTD.
	itd_node[0] = LINK_TERMINATE;
	frame_e = ((uint32_t)(uintptr_t)itd_node) | LINK_TYPE_ITD;
	CHECK_EQ((void *)sitd_skip_iso(&frame_e), (void *)&itd_node[0]);

	// Null input must not crash.
	CHECK_EQ((void *)sitd_skip_iso(0), (void *)0);

	// A cyclic list must terminate. sitd_skip_iso() runs in interrupt
	// context, so an unbounded walk over a corrupted list would wedge the
	// system rather than merely misbehave. cyc[0] links to itself, which no
	// correct scheduler would ever build -- the point is that corruption
	// must degrade, not hang. If the bound regresses this test never
	// returns, which is itself the failure signal.
	cyc[0] = ((uint32_t)(uintptr_t)cyc) | LINK_TYPE_SITD;
	frame_f = ((uint32_t)(uintptr_t)cyc) | LINK_TYPE_SITD;
	CHECK_EQ(sitd_skip_iso(&frame_f) != NULL, true);
}

static void test_sitd_pool(void)
{
	sitd_pool_init();

	// Drain the pool. Every allocation must be non-null, distinct, and
	// 32-byte aligned (hardware requirement -- see test_sitd_layout above).
	sitd_t *seen[256];
	int n = 0;
	sitd_t *s;
	while (n < (int)(sizeof(seen) / sizeof(seen[0])) && (s = sitd_alloc()) != NULL) {
		CHECK_EQ(((uintptr_t)s) % 32, 0);
		for (int i = 0; i < n; i++) CHECK_EQ(s == seen[i], false);
		seen[n++] = s;
	}
	CHECK_EQ(n > 0, true);

	// Pool exhausted: allocating further returns null rather than
	// fabricating a descriptor outside the DMA-reachable pool.
	CHECK_EQ((void *)sitd_alloc(), (void *)0);

	// Freeing gives back exactly one unit of capacity, and the freed siTD
	// can be reallocated (LIFO free list, so it is the very next one out).
	sitd_free(seen[0]);
	sitd_t *reused = sitd_alloc();
	CHECK_EQ(reused != NULL, true);
	CHECK_EQ((void *)reused, (void *)seen[0]);
	CHECK_EQ((void *)sitd_alloc(), (void *)0);   // exhausted again
}

static void test_link_unlink(void)
{
	sitd_pool_init();
	sitd_t *a = sitd_alloc();
	sitd_t *b = sitd_alloc();
	CHECK_EQ(a != NULL && b != NULL, true);

	// An empty frame slot. Linking must put the siTD at the head and carry
	// the previous contents into its next link.
	slot = LINK_TERMINATE;
	sitd_link(&slot, a, 3);
	CHECK_EQ((slot & 0x06u), LINK_TYPE_SITD);
	CHECK_EQ((slot & 0xFFFFFFE0u), (uint32_t)(uintptr_t)a & 0xFFFFFFE0u);
	CHECK_EQ(a->next, LINK_TERMINATE);
	CHECK_EQ(a->frame, 3);

	// A second siTD goes in front of the first, and the first stays reachable.
	sitd_link(&slot, b, 3);
	CHECK_EQ((slot & 0xFFFFFFE0u), (uint32_t)(uintptr_t)b & 0xFFFFFFE0u);
	CHECK_EQ((b->next & 0xFFFFFFE0u), (uint32_t)(uintptr_t)a & 0xFFFFFFE0u);

	// skip_iso must walk past both and land on a's next field, which is
	// where a queue head would attach.
	CHECK_EQ((void *)sitd_skip_iso(&slot), (void *)&a->next);

	// Unlink the tail: b stays at the head, its next becomes terminate.
	CHECK_EQ(sitd_unlink(&slot, a), true);
	CHECK_EQ((slot & 0xFFFFFFE0u), (uint32_t)(uintptr_t)b & 0xFFFFFFE0u);
	CHECK_EQ(b->next, LINK_TERMINATE);

	// Unlink the head: the slot returns to what it was.
	CHECK_EQ(sitd_unlink(&slot, b), true);
	CHECK_EQ(slot, LINK_TERMINATE);

	// Unlinking something absent reports failure rather than corrupting.
	CHECK_EQ(sitd_unlink(&slot, a), false);
	CHECK_EQ(slot, LINK_TERMINATE);

	// Null arguments are ignored, not dereferenced.
	sitd_link(NULL, a, 0);
	sitd_link(&slot, NULL, 0);
	CHECK_EQ(slot, LINK_TERMINATE);
	CHECK_EQ(sitd_unlink(NULL, a), false);
	CHECK_EQ(sitd_unlink(&slot, NULL), false);
}

int main(void)
{
	test_sitd_layout();
	test_budget_out();
	test_skip_iso();
	test_sitd_pool();
	test_link_unlink();
	printf("%d checks, %d failures\n", checks, failures);
	return failures ? 1 : 0;
}
