/*
 * Copyright 2026 Nicholas Newdigate
 * Portions derived from the MCUXpresso SDK USB host stack:
 *   Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
 *   Copyright 2016, 2019 - 2020 NXP
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef EHCI_ISO_H_
#define EHCI_ISO_H_
#include <stdint.h>
#include <stddef.h>

// sizeof(sitd_struct) without the trailing pad[] below. A standalone struct
// with the identical field sequence (rather than sizeof(sitd_t) computed
// from within its own definition, which isn't possible), used only to size
// the padding at compile time. sizeof(void*) stands in for
// sizeof(sitd_struct*): every real ABI this header is built for (Itanium,
// AAPCS) gives all object pointers the same size and alignment, and this
// struct is never instantiated for anything but the sizeof below.
//
// This matters because the byte count needed to reach a 32-byte multiple
// is NOT the same on a 64-bit host (8-byte next_free, extra alignment
// padding before it) as on the 32-bit ARM target this code actually ships
// on (4-byte next_free). A fixed-size pad[N] tuned only against a host
// build would compile clean here and still be silently misaligned on
// device -- exactly the "looks fine, transfers nothing" failure mode this
// whole struct exists to avoid. Computing the padding from sizeof() keeps
// it correct on both.
namespace sitd_detail {
struct prefix_t {
	uint32_t next, ep_char, uframe_mask, status, buf0, buf1, back;
	void *next_free_placeholder;
	uint16_t frame, reserved;
};
}

// EHCI 1.0 section 3.4. Seven hardware dwords followed by driver bookkeeping.
// Hardware requires 32-byte alignment, so the whole struct is padded to a
// 32-byte multiple.
typedef struct sitd_struct {
	uint32_t next;         // next link pointer + type
	uint32_t ep_char;      // dir, port, hub addr, endpoint, device address
	uint32_t uframe_mask;  // C-mask (bits 15:8), S-mask (bits 7:0)
	uint32_t status;       // status, C-prog-mask, bytes to transfer, IOC
	uint32_t buf0;         // page 0 + current offset
	uint32_t buf1;         // page 1 + transaction position/count
	uint32_t back;         // back pointer
	// --- software bookkeeping, not read by hardware ---
	struct sitd_struct *next_free;
	uint16_t frame;        // frame index this siTD is linked into
	uint16_t reserved;
	uint8_t pad[(32u - (sizeof(sitd_detail::prefix_t) % 32u)) % 32u];
} sitd_t;

// Field shifts, matching EHCI 1.0 section 3.4 and NXP's EHCI_HOST_SITD_*.
#define SITD_PTR_TYPE_SITD     0x04u
#define SITD_DIRECTION_SHIFT   31u
#define SITD_PORT_SHIFT        24u
#define SITD_HUB_ADDR_SHIFT    16u
#define SITD_ENDPT_SHIFT       8u
#define SITD_DEV_ADDR_SHIFT    0u
#define SITD_CMASK_SHIFT       8u
#define SITD_SMASK_SHIFT       0u
#define SITD_STATUS_ACTIVE     0x80u
#define SITD_TOTAL_BYTES_SHIFT 16u
#define SITD_IOC_SHIFT         31u

// Split-transaction budgeting for a full-speed isochronous OUT endpoint.
// Returns false if the packet cannot be scheduled. On success writes the
// start-split and complete-split masks.
bool sitd_budget_out(uint16_t max_packet, uint8_t start_uframe,
                     uint8_t *smask, uint8_t *cmask);

#endif
