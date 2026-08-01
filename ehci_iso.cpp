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

// Periodic frame list link-pointer encoding, EHCI 1.0 section 3.1: bit 0 is
// Terminate, bits 2:1 are the type (0=iTD, 1=QH, 2=siTD, 3=FSTN), and bits
// 31:5 are the (32-byte aligned) address of the linked structure.
#define LINK_TERMINATE 0x01u
#define LINK_TYPE_MASK 0x06u
#define LINK_TYPE_QH   0x02u
#define LINK_ADDR_MASK 0xFFFFFFE0u

volatile uint32_t *sitd_skip_iso(volatile uint32_t *frame_link)
{
	if (!frame_link) return frame_link;

	for (;;) {
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
#if defined(__IMXRT1176__)
#define USBHOST_DMAMEM DMAMEM
#else
#define USBHOST_DMAMEM
#endif

// 12 frames of ring depth (EHCI 1.0 section 8) plus headroom so a refill can
// run ahead of the hardware.
#define SITD_POOL_SIZE 16

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
