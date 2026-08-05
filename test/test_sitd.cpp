// Copyright (c) 2026 Nicholas Newdigate
// SPDX-License-Identifier: MIT
#include "ehci_iso.h"
#include <stdio.h>
#include <stddef.h>
#include <string.h>

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

static uint32_t fill_buf[64] __attribute__ ((aligned(32)));

static void test_fill_out(void)
{
	sitd_pool_init();
	sitd_t *n = sitd_alloc();
	CHECK_EQ(n != NULL, true);

	// 192 bytes at 48 kHz to endpoint 4 on device 3, directly attached
	// (hub 0, port 0), starting at microframe 0, interrupt on complete.
	CHECK_EQ(sitd_fill_out(n, 3, 4, 0, 0, fill_buf, 192, 0, true), true);

	// Endpoint characteristics: OUT (bit 31 clear), port 0, hub 0, ep 4, addr 3.
	CHECK_EQ(n->ep_char >> 31, 0);                    // direction OUT
	CHECK_EQ((n->ep_char >> 8) & 0x0F, 4);            // endpoint
	CHECK_EQ(n->ep_char & 0x7F, 3);                   // device address
	CHECK_EQ((n->ep_char >> 16) & 0x7F, 0);           // hub addr, root port
	CHECK_EQ((n->ep_char >> 24) & 0x7F, 0);           // port

	// 192 bytes spans two microframes, so S-mask covers uframes 0 and 1 and
	// C-mask is zero -- an isochronous OUT takes no complete-splits.
	CHECK_EQ(n->uframe_mask & 0xFF, 0x03);
	CHECK_EQ((n->uframe_mask >> 8) & 0xFF, 0x00);

	// Status: active, 192 bytes to transfer, IOC set.
	CHECK_EQ(n->status & 0x80, 0x80);                 // Active
	CHECK_EQ((n->status >> 16) & 0x3FF, 192);         // bytes to transfer
	CHECK_EQ(n->status >> 31, 1);                     // IOC

	// Buffer pointers: buf0 is the payload, buf1 is the NEXT 4K page plus
	// the split bookkeeping. 192 bytes needs two start-splits, so T-count 2
	// and TP=Begin(1) to seed the Begin/Mid/End sequence.
	CHECK_EQ(n->buf0, (uint32_t)(uintptr_t)fill_buf);
	CHECK_EQ(n->buf1 & 0xFFFFF000u,
	         (((uint32_t)(uintptr_t)fill_buf) + 4096u) & 0xFFFFF000u);
	CHECK_EQ(n->buf1 & 0x07, 2);                      // T-count
	CHECK_EQ((n->buf1 >> 3) & 0x03, 1);               // TP = Begin

	CHECK_EQ(n->back, 1);                             // back pointer terminated

	// A 180-byte packet fits one start-split: T-count 1, TP = All(0).
	CHECK_EQ(sitd_fill_out(n, 3, 4, 0, 0, fill_buf, 180, 0, false), true);
	CHECK_EQ(n->buf1 & 0x07, 1);
	CHECK_EQ((n->buf1 >> 3) & 0x03, 0);
	CHECK_EQ(n->status >> 31, 0);                     // IOC not requested

	// Rejections leave the descriptor alone rather than half-built.
	CHECK_EQ(sitd_fill_out(n, 3, 4, 0, 0, fill_buf, 1024, 0, true), false);
	CHECK_EQ(sitd_fill_out(n, 3, 4, 0, 0, NULL, 192, 0, true), false);
	CHECK_EQ(sitd_fill_out(NULL, 3, 4, 0, 0, fill_buf, 192, 0, true), false);

	// Status readback of a freshly filled, not-yet-run descriptor.
	sitd_status_t st;
	CHECK_EQ(sitd_fill_out(n, 3, 4, 0, 0, fill_buf, 192, 0, true), true);
	sitd_get_status(n, &st);
	CHECK_EQ(st.active, true);
	CHECK_EQ(st.bytes_left, 192);
	CHECK_EQ(st.err_transaction || st.err_babble || st.err_buffer, false);

	// Simulate the controller completing it: Active cleared, count drained.
	n->status &= ~0x80u;
	n->status &= ~(0x3FFu << 16);
	sitd_get_status(n, &st);
	CHECK_EQ(st.active, false);
	CHECK_EQ(st.bytes_left, 0);

	// Field-width guards, mirroring itd_fill_out: endpoint is 4 bits,
	// device address 7 -- oversized values would bleed into ep_char
	// neighbours.
	CHECK_EQ(sitd_fill_out(n, 3, 16, 0, 0, fill_buf, 192, 0, false), false);
	CHECK_EQ(sitd_fill_out(n, 128, 4, 0, 0, fill_buf, 192, 0, false), false);
	CHECK_EQ(sitd_fill_in(n, 3, 16, 0, 0, fill_buf, 8, 0, false), false);
	CHECK_EQ(sitd_fill_in(n, 128, 2, 0, 0, fill_buf, 8, 0, false), false);
}

// An siTD is refilled in place while still linked, so every hardware dword
// the fill owns must be rewritten -- a field left stale is one the controller
// reads from the previous packet. Filling a poisoned node must therefore
// produce byte-identical hardware state to filling a zeroed one; any field
// the fill forgets shows up as 0xA5A5A5A5 against 0. `next` is excluded on
// purpose: it belongs to sitd_link/sitd_unlink, not to the fill.
static void test_fill_refill_leaves_nothing_stale(void)
{
	sitd_t clean, poisoned;

	memset(&clean, 0, sizeof(clean));
	memset(&poisoned, 0xA5, sizeof(poisoned));
	CHECK_EQ(sitd_fill_out(&clean, 3, 4, 0, 0, fill_buf, 192, 0, true), true);
	CHECK_EQ(sitd_fill_out(&poisoned, 3, 4, 0, 0, fill_buf, 192, 0, true), true);
	CHECK_EQ(clean.ep_char,      poisoned.ep_char);
	CHECK_EQ(clean.uframe_mask,  poisoned.uframe_mask);
	CHECK_EQ(clean.status,       poisoned.status);
	CHECK_EQ(clean.buf0,         poisoned.buf0);
	CHECK_EQ(clean.buf1,         poisoned.buf1);
	CHECK_EQ(clean.back,         poisoned.back);
	CHECK_EQ(poisoned.next, 0xA5A5A5A5u);   // the fill does not own the link

	memset(&clean, 0, sizeof(clean));
	memset(&poisoned, 0xA5, sizeof(poisoned));
	CHECK_EQ(sitd_fill_in(&clean, 3, 2, 0, 0, fill_buf, 8, 0, false), true);
	CHECK_EQ(sitd_fill_in(&poisoned, 3, 2, 0, 0, fill_buf, 8, 0, false), true);
	CHECK_EQ(clean.ep_char,      poisoned.ep_char);
	CHECK_EQ(clean.uframe_mask,  poisoned.uframe_mask);
	CHECK_EQ(clean.status,       poisoned.status);
	CHECK_EQ(clean.buf0,         poisoned.buf0);
	CHECK_EQ(clean.buf1,         poisoned.buf1);
	CHECK_EQ(clean.back,         poisoned.back);
	CHECK_EQ(poisoned.next, 0xA5A5A5A5u);
}

// Untouched-on-rejection across the whole struct, matching the iTD fills'
// contract: a fill that refuses must not have scribbled on a live descriptor.
static void test_fill_untouched_on_rejection(void)
{
	sitd_t n, before;

	memset(&n, 0xA5, sizeof(n));
	memcpy(&before, &n, sizeof(n));

	CHECK_EQ(sitd_fill_out(NULL, 3, 4, 0, 0, fill_buf, 192, 0, false), false);
	CHECK_EQ(sitd_fill_out(&n, 3, 4, 0, 0, NULL, 192, 0, false), false);
	CHECK_EQ(sitd_fill_out(&n, 3, 16, 0, 0, fill_buf, 192, 0, false), false);
	CHECK_EQ(sitd_fill_out(&n, 128, 4, 0, 0, fill_buf, 192, 0, false), false);
	// A budget that cannot be scheduled: too many bytes to fit the frame.
	CHECK_EQ(sitd_fill_out(&n, 3, 4, 0, 0, fill_buf, 2000, 0, false), false);
	CHECK_EQ(memcmp(&n, &before, sizeof(n)), 0);

	CHECK_EQ(sitd_fill_in(NULL, 3, 2, 0, 0, fill_buf, 8, 0, false), false);
	CHECK_EQ(sitd_fill_in(&n, 3, 2, 0, 0, NULL, 8, 0, false), false);
	CHECK_EQ(sitd_fill_in(&n, 3, 16, 0, 0, fill_buf, 8, 0, false), false);
	CHECK_EQ(sitd_fill_in(&n, 128, 2, 0, 0, fill_buf, 8, 0, false), false);
	CHECK_EQ(memcmp(&n, &before, sizeof(n)), 0);
}

static void test_budget_in(void)
{
	uint8_t s = 0xAA, c = 0xAA;

	// Full-speed isochronous IN behind the (embedded) TT: one start-split in
	// microframe S carrying the IN token, then complete-splits to collect the
	// data, one per 188-byte chunk, starting two microframes later -- the FS
	// transaction itself runs during S+1. This is the NXP EHCI host stack's
	// scheduling (usb_host_ehci.c, USB_HostBandwidthHsHostAllocateIso:
	// "there are two micro-frame interval between start-split and
	// complete-split"; CS count = ceil(maxPacket/188)).
	//
	// The 3-byte UAC1 feedback packet is the real user: one CS, S+2.
	CHECK_EQ(sitd_budget_in(3, 0, &s, &c), true);
	CHECK_EQ(s, 0x01);
	CHECK_EQ(c, 0x04);

	// Later start shifts both masks.
	CHECK_EQ(sitd_budget_in(3, 5, &s, &c), true);
	CHECK_EQ(s, 0x20);
	CHECK_EQ(c, 0x80);

	// S=6 would need a complete-split in microframe 8: rejected, because for
	// isochronous the NXP allocator treats a clipped CS window as an
	// allocation failure rather than silently shrinking it.
	CHECK_EQ(sitd_budget_in(3, 6, &s, &c), false);

	// 188 bytes is still one chunk; 189 needs a second complete-split.
	CHECK_EQ(sitd_budget_in(188, 0, &s, &c), true);
	CHECK_EQ(c, 0x04);
	CHECK_EQ(sitd_budget_in(189, 0, &s, &c), true);
	CHECK_EQ(c, 0x0C);

	// Largest legal FS iso packet: 1023 bytes = 6 chunks, CS in uframes
	// 2..7 -- exactly fits from S=0 and nowhere later.
	CHECK_EQ(sitd_budget_in(1023, 0, &s, &c), true);
	CHECK_EQ(s, 0x01);
	CHECK_EQ(c, 0xFC);
	CHECK_EQ(sitd_budget_in(1023, 1, &s, &c), false);

	// Bounds and null checks, mirroring sitd_budget_out.
	CHECK_EQ(sitd_budget_in(1024, 0, &s, &c), false);
	CHECK_EQ(sitd_budget_in(0, 0, &s, &c), false);
	CHECK_EQ(sitd_budget_in(3, 8, &s, &c), false);
	CHECK_EQ(sitd_budget_in(3, 0, 0, &c), false);
	CHECK_EQ(sitd_budget_in(3, 0, &s, 0), false);
}

static void test_fill_in(void)
{
	sitd_pool_init();
	sitd_t *n = sitd_alloc();
	CHECK_EQ(n != NULL, true);

	// The UAC1 feedback read: 8 bytes of room for a 3-byte packet from
	// endpoint 2 IN on device 3, directly attached, microframe 0, polled.
	CHECK_EQ(sitd_fill_in(n, 3, 2, 0, 0, fill_buf, 8, 0, false), true);

	// Endpoint characteristics: IN (bit 31 set), ep 2, addr 3, root port.
	CHECK_EQ(n->ep_char >> 31, 1);
	CHECK_EQ((n->ep_char >> 8) & 0x0F, 2);
	CHECK_EQ(n->ep_char & 0x7F, 3);
	CHECK_EQ((n->ep_char >> 16) & 0x7F, 0);
	CHECK_EQ((n->ep_char >> 24) & 0x7F, 0);

	// Masks per the budget: SS at uframe 0, CS at uframe 2.
	CHECK_EQ(n->uframe_mask & 0xFF, 0x01);
	CHECK_EQ((n->uframe_mask >> 8) & 0xFF, 0x04);

	// Status: active, room for 8 bytes, no IOC.
	CHECK_EQ(n->status & 0x80, 0x80);
	CHECK_EQ((n->status >> 16) & 0x3FF, 8);
	CHECK_EQ(n->status >> 31, 0);

	// Buffer pointers. TP and T-count are OUT-only fields (EHCI 1.0 Table
	// 3-12: "used only for OUT"), so buf1 carries just the next-page
	// pointer, with the low bits zero.
	CHECK_EQ(n->buf0, (uint32_t)(uintptr_t)fill_buf);
	CHECK_EQ(n->buf1 & 0xFFFFF000u,
	         (((uint32_t)(uintptr_t)fill_buf) + 4096u) & 0xFFFFF000u);
	CHECK_EQ(n->buf1 & 0xFFFu, 0);

	CHECK_EQ(n->back, 1);

	// Completion: controller clears Active and decrements total-bytes by
	// what arrived. 3 of 8 bytes -> bytes_left 5, and the driver computes
	// received = armed - bytes_left.
	n->status &= ~0x80u;
	n->status = (n->status & ~(0x3FFu << 16)) | (5u << 16);
	sitd_status_t st;
	sitd_get_status(n, &st);
	CHECK_EQ(st.active, false);
	CHECK_EQ(st.bytes_left, 5);

	// Rejections, mirroring sitd_fill_out.
	CHECK_EQ(sitd_fill_in(n, 3, 2, 0, 0, fill_buf, 8, 6, false), false);
	CHECK_EQ(sitd_fill_in(n, 3, 2, 0, 0, NULL, 8, 0, false), false);
	CHECK_EQ(sitd_fill_in(NULL, 3, 2, 0, 0, fill_buf, 8, 0, false), false);
	CHECK_EQ(sitd_fill_in(n, 3, 2, 0, 0, fill_buf, 0, 0, false), false);
}

static void test_disarm(void)
{
	// A recycled node carries its previous life's ACTIVE status: alloc does
	// not zero. Disarm must read back inactive so the node can be LINKED
	// before it is armed (the priming margin in usb_audio.cpp).
	sitd_pool_init();
	sitd_t *n = sitd_alloc();
	CHECK_EQ(sitd_fill_out(n, 3, 2, 0, 0, fill_buf, 96, 0, false), true);
	sitd_disarm(n);
	sitd_status_t st;
	sitd_get_status(n, &st);
	CHECK_EQ(st.active, false);
	sitd_disarm(NULL);   // must not crash
}

int main(void)
{
	test_sitd_layout();
	test_budget_out();
	test_skip_iso();
	test_sitd_pool();
	test_link_unlink();
	test_fill_out();
	test_budget_in();
	test_fill_in();
	test_fill_refill_leaves_nothing_stale();
	test_fill_untouched_on_rejection();
	test_disarm();
	printf("%d checks, %d failures\n", checks, failures);
	return failures ? 1 : 0;
}
