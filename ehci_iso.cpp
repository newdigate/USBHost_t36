/*
 * Copyright 2026 Nicholas Newdigate
 * Portions derived from the MCUXpresso SDK USB host stack:
 *   Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
 *   Copyright 2016, 2019 - 2020 NXP
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "ehci_iso.h"

// A full-speed bus carries at most 188 bytes of isochronous payload per
// microframe. EHCI 1.0 section 4.12.3.
#define FS_BYTES_PER_UFRAME 188u
#define FS_ISO_MAX_PACKET   1023u

bool sitd_budget_out(uint16_t max_packet, uint8_t start_uframe,
                     uint8_t *smask, uint8_t *cmask)
{
	if (!smask || !cmask) return false;
	if (max_packet == 0 || max_packet > FS_ISO_MAX_PACKET) return false;
	if (start_uframe > 7) return false;

	uint32_t uframes = (max_packet + FS_BYTES_PER_UFRAME - 1) / FS_BYTES_PER_UFRAME;
	if (start_uframe + uframes > 8) return false;   // must not wrap the frame

	uint8_t mask = 0;
	for (uint32_t k = 0; k < uframes; k++) mask |= (uint8_t)(1u << (start_uframe + k));

	*smask = mask;
	*cmask = 0;   // isochronous OUT: start-splits carry the data, no CSPLITs
	return true;
}

bool sitd_budget_in(uint16_t max_packet, uint8_t start_uframe,
                    uint8_t *smask, uint8_t *cmask)
{
	if (!smask || !cmask) return false;
	if (max_packet == 0 || max_packet > FS_ISO_MAX_PACKET) return false;
	if (start_uframe > 7) return false;

	// One complete-split per 188-byte chunk, starting two microframes after
	// the start-split: the TT runs the FS IN during start_uframe+1 and the
	// data is collected from start_uframe+2 on (NXP usb_host_ehci.c,
	// USB_HostBandwidthHsHostAllocateIso: "there are two micro-frame
	// interval between start-split and complete-split"). A window past
	// microframe 7 is a scheduling failure -- for isochronous the NXP
	// allocator rejects rather than clips, because a clipped window is a
	// packet that can silently never be collected.
	uint32_t chunks = (max_packet + FS_BYTES_PER_UFRAME - 1) / FS_BYTES_PER_UFRAME;
	if (start_uframe + 2u + chunks > 8) return false;

	uint8_t mask = 0;
	for (uint32_t k = 0; k < chunks; k++)
		mask |= (uint8_t)(1u << (start_uframe + 2 + k));

	*smask = (uint8_t)(1u << start_uframe);
	*cmask = mask;
	return true;
}

// Periodic frame list link-pointer encoding, EHCI 1.0 section 3.1: bit 0 is
// Terminate, bits 2:1 are the type (0=iTD, 1=QH, 2=siTD, 3=FSTN), and bits
// 31:5 are the (32-byte aligned) address of the linked structure.
#define LINK_TERMINATE 0x01u
#define LINK_TYPE_MASK 0x06u
#define LINK_TYPE_ITD  0x00u
#define LINK_TYPE_QH   0x02u
#define LINK_TYPE_SITD 0x04u
#define LINK_ADDR_MASK 0xFFFFFFE0u

// One siTD per periodic frame list slot (32 on this target) for the audio
// OUT ring -- a slot is only revisited every PERIODIC_LIST_SIZE frames, so to
// put a packet on the bus every 1 ms every slot needs its own live
// descriptor. Plus headroom for the UAC1 feedback endpoint reader (two
// descriptors, 16 frames apart) and a test descriptor: without the headroom,
// beginStreaming() would drain the pool to exactly zero and the feedback
// pipe could never be armed. Also the bound on how many isochronous
// descriptors one frame can hold (see sitd_skip_iso).
#define SITD_POOL_SIZE 40
// The HS ring plus a full-rate feedback reader: 32 periodic OUT slots and
// 32 feedback IN slots, one per frame. Full-rate polling is not a luxury --
// subsampling the device's 1000/s reports aliases their dither duty into a
// slowly wandering rate estimate the FIFO then integrates (measured at
// 250 polls/s: >10 s fill drift predicted from the sampled reports with
// correlation +0.91, and no filter horizon fixes it; see the evkb repo's
// transcript_uacv_servo_isolation.txt). Arming a stream drains this pool
// to exactly zero by design.
#define ITD_POOL_SIZE  64

// Any iso descriptor from either pool can legally occupy a frame's iso
// run, so every bounded walker over that run must tolerate the combined
// worst case, not one pool's.
#define ISO_RUN_GUARD (SITD_POOL_SIZE + ITD_POOL_SIZE)

volatile uint32_t *sitd_skip_iso(volatile uint32_t *frame_link)
{
	if (!frame_link) return frame_link;

	// Bounded because this runs in interrupt context: a corrupted or cyclic
	// link list would otherwise spin here forever and wedge the system. A
	// frame can legitimately hold at most ISO_RUN_GUARD isochronous
	// descriptors (both pools), so anything beyond that is corruption.
	// Bailing out returns a link field that is still safe for the caller to
	// insert at -- worse scheduling, but not a hang.
	for (unsigned guard = 0; guard <= ISO_RUN_GUARD; guard++) {
		uint32_t link = *frame_link;
		if ((link & LINK_TERMINATE) || ((link & LINK_TYPE_MASK) == LINK_TYPE_QH)) {
			return frame_link;
		}

		uint32_t addr = link & LINK_ADDR_MASK;

		// Reconstruct the pointer being followed. On the real target
		// uintptr_t is 32 bits, so this is exactly (volatile uint32_t *)addr
		// -- the high-bits splice below folds away to nothing at compile
		// time. On a wider host it recovers the missing high bits by
		// assuming the linked descriptor lives in the same region of
		// address space as the link field we just read through, which is
		// unconditionally true on target (one flat 32-bit address space)
		// and is arranged to be true in the host tests (see
		// test/test_sitd.cpp: every node in a test chain is `static`, so
		// they share high bits).
		uintptr_t same_region = (uintptr_t)frame_link & ~(uintptr_t)0xFFFFFFFFu;
		frame_link = (volatile uint32_t *)(same_region | (uintptr_t)addr);
	}
	return frame_link;   /* guard tripped: list is cyclic or corrupt */
}

// USBHOST_DMAMEM is defined per-.cpp in this codebase, not in a shared
// header (see memory.cpp:67, ehci.cpp:66, hid.cpp:41, enumeration.cpp:53) --
// follow that convention here too. RT1176: plain .bss is DTCM, which the
// EHCI DMA master cannot reach; DMAMEM places data in DMA-reachable OCRAM.
// On Teensy 4.x (__IMXRT1062__) .bss is already OCRAM, so, matching every
// other USBHOST_DMAMEM site in this codebase, the guard is RT1176-only. On
// the host build (this file compiled for test/test_sitd) USBHOST_DMAMEM is
// empty, so the pool below is plain static storage -- that is what makes it
// testable here at all; on RT1176 it is the difference between the EHCI
// controller being able to see these siTDs and silently not.
// DMAMEM itself comes from the Teensy core, so pull that in only on target.
// The host build must stay free of Arduino headers -- that is what lets this
// file be compiled into test/test_sitd and exercised without hardware.
#if defined(__IMXRT1176__)
#include <Arduino.h>
#define USBHOST_DMAMEM DMAMEM
#else
#define USBHOST_DMAMEM
#endif

// 12 frames of ring depth (EHCI 1.0 section 8) plus headroom so a refill can
// run ahead of the hardware.
static USBHOST_DMAMEM sitd_t sitd_pool[SITD_POOL_SIZE] __attribute__ ((aligned(32)));
static sitd_t *sitd_free_list;

void sitd_pool_init(void)
{
	sitd_free_list = NULL;
	for (int i = SITD_POOL_SIZE - 1; i >= 0; i--) {
		sitd_pool[i].next_free = sitd_free_list;
		sitd_free_list = &sitd_pool[i];
	}
}

sitd_t *sitd_alloc(void)
{
	sitd_t *node = sitd_free_list;
	if (node) sitd_free_list = node->next_free;
	return node;
}

void sitd_free(sitd_t *node)
{
	if (!node) return;
	node->next_free = sitd_free_list;
	sitd_free_list = node;
}

static USBHOST_DMAMEM itd_t itd_pool[ITD_POOL_SIZE] __attribute__ ((aligned(32)));
static itd_t *itd_free_list;

void itd_pool_init(void)
{
	itd_free_list = NULL;
	for (int i = ITD_POOL_SIZE - 1; i >= 0; i--) {
		itd_pool[i].next_free = itd_free_list;
		itd_free_list = &itd_pool[i];
	}
}

itd_t *itd_alloc(void)
{
	itd_t *node = itd_free_list;
	if (node) itd_free_list = node->next_free;
	return node;
}


void itd_free(itd_t *node)
{
	if (!node) return;
	node->next_free = itd_free_list;
	itd_free_list = node;
}

// Publish a descriptor's buffer state before the bit that hands it to the
// controller.
//
// These descriptors are refilled IN PLACE while still linked into the
// periodic schedule, so the controller can read one at any moment. Writing
// the ACTIVE bit before the page pointers it depends on leaves a window --
// short, but real -- where the hardware owns a descriptor pointing at the
// previous packet's buffer. Statement order alone does not close it: the
// fields are plain uint32_t, so both the compiler and the M7's store buffer
// are free to reorder the writes. This is the barrier that makes the
// ordering the code is written in the ordering the controller observes.
//
// __sync_synchronize() rather than CMSIS __DMB() because this file is also
// compiled for the host test build; it emits `dmb ish` for Cortex-M7 (whose
// TRM specifies all DMB options behave as full-system) and a plain fence on
// the host, where it costs nothing and means nothing.
static inline void iso_publish_barrier(void)
{
	__sync_synchronize();
}

void itd_disarm(itd_t *node)
{
	if (!node) return;
	for (int k = 0; k < 8; k++) node->transaction[k] = 0;
	iso_publish_barrier();
}

void sitd_disarm(sitd_t *node)
{
	if (!node) return;
	node->status = 0;
	iso_publish_barrier();
}

bool itd_fill_out(itd_t *node, uint8_t dev_addr, uint8_t endpoint,
                  const void *buf, const uint16_t len[8], uint16_t max_packet,
                  bool ioc_last)
{
	if (!node || !buf || !len) return false;
	if (max_packet == 0 || max_packet > 1024) return false;
	if (endpoint > 15 || dev_addr > 127) return false;

	uint32_t base = (uint32_t)(uintptr_t)buf;
	uint32_t page0 = base & 0xFFFFF000u;

	int last_active = -1;
	uint32_t off = base & 0xFFFu;
	uint32_t txn[8];
	for (int k = 0; k < 8; k++) {
		if (len[k] == 0) { txn[k] = 0; continue; }
		if (len[k] > max_packet) return false;
		uint32_t pg = off >> 12;
		// structural bound for the 7 bufptr entries; unreachable while
		// max_packet <= 1024 keeps 8 transactions inside ~3 pages
		if (pg > 6) return false;   // contiguous buffer outran the pages
		txn[k] = ITD_TXN_ACTIVE
		       | ((uint32_t)len[k] << ITD_TXN_LENGTH_SHIFT)
		       | (pg << ITD_TXN_PG_SHIFT)
		       | (off & ITD_TXN_OFFSET_MASK);
		last_active = k;
		off += len[k];
	}
	if (last_active < 0) return false;
	if (ioc_last) txn[last_active] |= ITD_TXN_IOC;

	// Buffer state first, ACTIVE last (see iso_publish_barrier): the
	// transaction words carry the ACTIVE bits, so they are the commit.
	//
	// Contiguous payload means consecutive physical pages: page i is
	// page0 + i*4K. Low bits per EHCI Table 3-6.
	for (int i = 0; i < 7; i++) node->bufptr[i] = page0 + (uint32_t)i * 4096u;
	node->bufptr[0] |= ((uint32_t)endpoint << 8) | dev_addr;
	// Direction is bufptr[1] bit 11 -- stays 0 = OUT because bufptr[1]
	// only ever receives max_packet, bits 10:0.
	node->bufptr[1] |= max_packet;
	node->bufptr[2] |= 1u;              // Multi = 1

	iso_publish_barrier();
	for (int k = 0; k < 8; k++) node->transaction[k] = txn[k];
	return true;
}

bool itd_fill_in(itd_t *node, uint8_t dev_addr, uint8_t endpoint,
                 void *buf, uint16_t len, uint16_t max_packet, bool ioc)
{
	if (!node || !buf) return false;
	// 3072 is the EHCI ceiling for one microframe (Multi=3 of 1024); the
	// 12-bit length field could encode more, and a larger value would be
	// a lie the controller might act on.
	if (len == 0 || len > 3072) return false;
	if (max_packet == 0 || max_packet > 1024) return false;
	if (endpoint > 15 || dev_addr > 127) return false;

	uint32_t base = (uint32_t)(uintptr_t)buf;
	uint32_t page0 = base & 0xFFFFF000u;
	uint32_t off = base & 0xFFFu;

	// Buffer state first, ACTIVE last (see iso_publish_barrier).
	//
	// Contiguous payload means consecutive physical pages, same as the
	// OUT fill. Low bits per EHCI Table 3-6.
	for (int i = 0; i < 7; i++) node->bufptr[i] = page0 + (uint32_t)i * 4096u;
	node->bufptr[0] |= ((uint32_t)endpoint << 8) | dev_addr;
	node->bufptr[1] |= ITD_BUFPTR1_DIR_IN | max_packet;
	node->bufptr[2] |= 1u;              // Multi = 1
	// Clearing the unused microframes is part of the buffer state, not the
	// commit: a stale ACTIVE bit left in one of them is a phantom
	// transaction, so they must be dead before transaction[0] goes live.
	for (int k = 1; k < 8; k++) node->transaction[k] = 0;

	iso_publish_barrier();
	// One transaction, microframe 0. PG is 0 by construction (off < 4K);
	// a read crossing into the next page rolls to bufptr[1] in hardware.
	node->transaction[0] = ITD_TXN_ACTIVE
	                     | ((uint32_t)len << ITD_TXN_LENGTH_SHIFT)
	                     | (off & ITD_TXN_OFFSET_MASK)
	                     | (ioc ? ITD_TXN_IOC : 0u);
	return true;
}

void itd_get_txn_status(const itd_t *node, unsigned txn, itd_txn_status_t *out)
{
	if (!out) return;
	if (!node || txn > 7) {
		out->active = out->err_xact = out->err_babble = out->err_buffer = false;
		out->length = 0;
		return;
	}
	uint32_t t = node->transaction[txn];
	out->active     = (t & ITD_TXN_ACTIVE) != 0u;
	out->err_buffer = (t & ITD_TXN_ERR_BUFFER) != 0u;
	out->err_babble = (t & ITD_TXN_ERR_BABBLE) != 0u;
	out->err_xact   = (t & ITD_TXN_ERR_XACT) != 0u;
	out->length     = (uint16_t)((t >> ITD_TXN_LENGTH_SHIFT) & 0xFFFu);
}

void itd_link(volatile uint32_t *frame_slot, itd_t *node, uint16_t frame)
{
	if (!frame_slot || !node) return;
	node->next = *frame_slot;
	node->frame = frame;
	*frame_slot = ((uint32_t)(uintptr_t)node & LINK_ADDR_MASK) | LINK_TYPE_ITD;
}

bool itd_unlink(volatile uint32_t *frame_slot, itd_t *node)
{
	if (!frame_slot || !node) return false;
	uint32_t target = ((uint32_t)(uintptr_t)node & LINK_ADDR_MASK) | LINK_TYPE_ITD;
	volatile uint32_t *link = frame_slot;
	// Bounded for the same reason sitd_unlink is: interrupt context must
	// degrade on a corrupt list, not hang. The traversal steps through
	// whatever it finds -- iTD or siTD -- since both put their next link
	// at offset 0 (itd_t::next / sitd_t::next), so the reconstructed
	// uint32_t* is valid either way without knowing which struct it is.
	for (unsigned guard = 0; guard <= ISO_RUN_GUARD; guard++) {
		uint32_t cur = *link;
		if (cur & LINK_TERMINATE) return false;
		if ((cur & LINK_TYPE_MASK) == LINK_TYPE_QH) return false;
		if ((cur & (LINK_ADDR_MASK | LINK_TYPE_MASK)) == target) {
			*link = node->next;
			node->next = LINK_TERMINATE;
			return true;
		}
		uintptr_t same_region = (uintptr_t)link & ~(uintptr_t)0xFFFFFFFFu;
		link = (volatile uint32_t *)(same_region | (uintptr_t)(cur & LINK_ADDR_MASK));
	}
	return false;
}

void sitd_link(volatile uint32_t *frame_slot, sitd_t *node, uint16_t frame)
{
	if (!frame_slot || !node) return;

	// Whatever was at the head becomes our next link, so any interrupt queue
	// heads already scheduled in this frame stay reachable behind us.
	node->next = *frame_slot;
	node->frame = frame;

	// The address must be 32-byte aligned for the type bits to be free; the
	// pool guarantees that (aligned(32) on sitd_t and on the pool array).
	*frame_slot = ((uint32_t)(uintptr_t)node & LINK_ADDR_MASK) | LINK_TYPE_SITD;
}

bool sitd_unlink(volatile uint32_t *frame_slot, sitd_t *node)
{
	if (!frame_slot || !node) return false;

	uint32_t target = ((uint32_t)(uintptr_t)node & LINK_ADDR_MASK) | LINK_TYPE_SITD;
	volatile uint32_t *link = frame_slot;

	// Bounded for the same reason sitd_skip_iso is: this runs in interrupt
	// context and must degrade rather than hang on a corrupt list.
	for (unsigned guard = 0; guard <= ISO_RUN_GUARD; guard++) {
		uint32_t cur = *link;
		if (cur & LINK_TERMINATE) return false;
		if ((cur & LINK_TYPE_MASK) == LINK_TYPE_QH) return false;  // past the iso run

		if ((cur & (LINK_ADDR_MASK | LINK_TYPE_MASK)) == target) {
			*link = node->next;      // splice it out
			node->next = LINK_TERMINATE;
			return true;
		}

		uintptr_t same_region = (uintptr_t)link & ~(uintptr_t)0xFFFFFFFFu;
		link = (volatile uint32_t *)(same_region | (uintptr_t)(cur & LINK_ADDR_MASK));
	}
	return false;
}

// siTD status/result field bits, EHCI 1.0 Table 3-11.
#define SITD_STATUS_ERR_XACT   0x02u   // transaction error
#define SITD_STATUS_ERR_BABBLE 0x10u
#define SITD_STATUS_ERR_BUFFER 0x20u
#define SITD_TP_SHIFT          3u
#define SITD_TCOUNT_SHIFT      0u
#define SITD_TP_ALL            0u      // whole payload in one start-split
#define SITD_TP_BEGIN          1u      // first of several; HC sequences Mid/End

bool sitd_fill_out(sitd_t *node, uint8_t dev_addr, uint8_t endpoint,
                   uint8_t hub_addr, uint8_t port, const void *buf,
                   uint16_t len, uint8_t start_uframe, bool ioc)
{
	if (!node || !buf) return false;
	if (endpoint > 15 || dev_addr > 127) return false;

	uint8_t smask = 0, cmask = 0;
	if (!sitd_budget_out(len, start_uframe, &smask, &cmask)) return false;

	uint32_t addr = (uint32_t)(uintptr_t)buf;

	// Direction 0 = OUT. hub_addr/port are 0 for a device on the root port.
	node->ep_char = ((uint32_t)0u << SITD_DIRECTION_SHIFT)
	              | ((uint32_t)port     << SITD_PORT_SHIFT)
	              | ((uint32_t)hub_addr << SITD_HUB_ADDR_SHIFT)
	              | ((uint32_t)endpoint << SITD_ENDPT_SHIFT)
	              | ((uint32_t)dev_addr << SITD_DEV_ADDR_SHIFT);

	node->uframe_mask = ((uint32_t)cmask << SITD_CMASK_SHIFT)
	                  | ((uint32_t)smask << SITD_SMASK_SHIFT);

	node->buf0 = addr;

	// Buffer pointer 1 holds the NEXT 4K page (a payload may straddle one
	// boundary) plus the split bookkeeping. T-count is the number of OUT
	// start-splits at 188 bytes each; TP seeds the Begin/Mid/End sequence
	// the controller then walks, or says All when one split suffices.
	uint32_t tcount = ((uint32_t)len + FS_BYTES_PER_UFRAME - 1u) / FS_BYTES_PER_UFRAME;
	uint32_t tp = (tcount > 1u) ? SITD_TP_BEGIN : SITD_TP_ALL;
	node->buf1 = ((addr + 4096u) & 0xFFFFF000u)
	           | (tp << SITD_TP_SHIFT)
	           | (tcount << SITD_TCOUNT_SHIFT);

	node->back = LINK_TERMINATE;

	// ACTIVE lives in `status`, so status is the commit and goes last (see
	// iso_publish_barrier): an siTD is refilled in place while linked, and
	// the controller must never see it armed against the previous packet's
	// buffer pointers.
	iso_publish_barrier();
	node->status = ((uint32_t)len << SITD_TOTAL_BYTES_SHIFT)
	             | SITD_STATUS_ACTIVE
	             | (ioc ? (1uL << SITD_IOC_SHIFT) : 0u);
	return true;
}

bool sitd_fill_in(sitd_t *node, uint8_t dev_addr, uint8_t endpoint,
                  uint8_t hub_addr, uint8_t port, void *buf,
                  uint16_t len, uint8_t start_uframe, bool ioc)
{
	if (!node || !buf) return false;
	if (endpoint > 15 || dev_addr > 127) return false;

	uint8_t smask = 0, cmask = 0;
	if (!sitd_budget_in(len, start_uframe, &smask, &cmask)) return false;

	uint32_t addr = (uint32_t)(uintptr_t)buf;

	// Direction 1 = IN.
	node->ep_char = ((uint32_t)1u << SITD_DIRECTION_SHIFT)
	              | ((uint32_t)port     << SITD_PORT_SHIFT)
	              | ((uint32_t)hub_addr << SITD_HUB_ADDR_SHIFT)
	              | ((uint32_t)endpoint << SITD_ENDPT_SHIFT)
	              | ((uint32_t)dev_addr << SITD_DEV_ADDR_SHIFT);

	node->uframe_mask = ((uint32_t)cmask << SITD_CMASK_SHIFT)
	                  | ((uint32_t)smask << SITD_SMASK_SHIFT);

	node->buf0 = addr;

	// TP and T-count are OUT-only (EHCI 1.0 Table 3-12); for IN, buffer
	// pointer 1 is just the next 4K page in case the payload straddles one.
	node->buf1 = (addr + 4096u) & 0xFFFFF000u;

	node->back = LINK_TERMINATE;

	// Commit last, as in sitd_fill_out. Total bytes is the room offered;
	// the controller decrements it by what actually arrives, so
	// received = len - bytes_left at completion.
	iso_publish_barrier();
	node->status = ((uint32_t)len << SITD_TOTAL_BYTES_SHIFT)
	             | SITD_STATUS_ACTIVE
	             | (ioc ? (1uL << SITD_IOC_SHIFT) : 0u);
	return true;
}

void sitd_get_status(const sitd_t *node, sitd_status_t *out)
{
	if (!out) return;
	if (!node) {
		out->active = false;
		out->err_transaction = out->err_babble = out->err_buffer = false;
		out->bytes_left = 0;
		return;
	}
	uint32_t s = node->status;
	out->active          = (s & SITD_STATUS_ACTIVE) != 0u;
	out->err_transaction = (s & SITD_STATUS_ERR_XACT) != 0u;
	out->err_babble      = (s & SITD_STATUS_ERR_BABBLE) != 0u;
	out->err_buffer      = (s & SITD_STATUS_ERR_BUFFER) != 0u;
	out->bytes_left      = (uint16_t)((s >> SITD_TOTAL_BYTES_SHIFT) & 0x3FFu);
}
