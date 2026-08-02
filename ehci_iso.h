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

// EHCI 1.0 section 3.4. Seven hardware dwords followed by driver bookkeeping.
//
// aligned(32) does two jobs at once: the hardware requires each siTD to be
// 32-byte aligned, and the attribute also rounds sizeof() up to a multiple of
// 32 so an array of these has the right stride. Both matter -- padding the
// size alone would still leave a pool's base address unaligned. It is also
// pointer-size portable by construction, which a hand-computed pad[N] is not:
// next_free is 8 bytes on a 64-bit host and 4 on the 32-bit ARM target, so a
// fixed pad tuned against a host build would compile clean and be silently
// wrong on device -- the "looks fine, transfers nothing" failure this struct
// exists to prevent. Matches the Transfer_t pools in USBHost_t36.h.
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
} __attribute__ ((aligned(32))) sitd_t;

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

// EHCI 1.0 section 3.3. High-speed isochronous: one iTD per frame carries up
// to eight microframe transactions -- no splits, the TT is not involved.
// aligned(32) is the hardware alignment requirement and makes the pool
// stride a multiple of 32, same reasoning as sitd_t.
typedef struct itd_struct {
	uint32_t next;            // next link pointer + type
	uint32_t transaction[8];  // per-microframe status/control
	uint32_t bufptr[7];       // page pointers; low bits carry ep/mps/dir/mult
	// --- software bookkeeping, not read by hardware ---
	struct itd_struct *next_free;
	uint16_t frame;           // frame index this iTD is linked into
	uint16_t reserved;
} __attribute__ ((aligned(32))) itd_t;

// Transaction word fields, EHCI 1.0 Table 3-3.
#define ITD_TXN_ACTIVE       0x80000000u
#define ITD_TXN_ERR_BUFFER   0x40000000u
#define ITD_TXN_ERR_BABBLE   0x20000000u
#define ITD_TXN_ERR_XACT     0x10000000u
#define ITD_TXN_LENGTH_SHIFT 16u        // bits 27:16
#define ITD_TXN_IOC          0x00008000u
#define ITD_TXN_PG_SHIFT     12u        // bits 14:12
#define ITD_TXN_OFFSET_MASK  0x00000FFFu

// Split-transaction budgeting for a full-speed isochronous OUT endpoint.
// Returns false if the packet cannot be scheduled. On success writes the
// start-split and complete-split masks.
bool sitd_budget_out(uint16_t max_packet, uint8_t start_uframe,
                     uint8_t *smask, uint8_t *cmask);

// Same for a full-speed isochronous IN: one start-split carrying the IN
// token in microframe `start_uframe`, then complete-splits to collect the
// data -- one per 188-byte chunk, beginning two microframes after the
// start-split (the FS transaction itself occupies the microframe between).
// This is the NXP EHCI host stack's scheduling for this controller
// (usb_host_ehci.c, USB_HostBandwidthHsHostAllocateIso). A complete-split
// window that would run past microframe 7 is a scheduling failure, not a
// clip: for isochronous there is no later retry to absorb the loss.
bool sitd_budget_in(uint16_t max_packet, uint8_t start_uframe,
                    uint8_t *smask, uint8_t *cmask);

// Walk past any isochronous descriptors (iTD/siTD) at the head of a periodic
// frame list entry, and return the address of the link field where the first
// queue head is, or should be, attached.
//
// EHCI requires isochronous descriptors to precede interrupt queue heads
// within a frame. add_qh_to_periodic_schedule() must therefore start its
// queue-head traversal after them, or it will read driver fields out of
// hardware-descriptor memory.
//
// Safe because both structures carry their next-link at offset 0: Pipe_struct
// begins with qh.horizontal_link, sitd_t begins with next. So a link can be
// followed without knowing which it points at.
volatile uint32_t *sitd_skip_iso(volatile uint32_t *frame_link);

// Fixed-capacity pool of siTDs, backed by DMA-reachable memory (see
// USBHOST_DMAMEM in ehci_iso.cpp). sitd_pool_init() (re)initializes the pool
// and must be called once before use. sitd_alloc() returns a pointer to a
// free siTD, or NULL if the pool is exhausted. sitd_free() returns an siTD
// to the pool; it must not still be linked into the periodic schedule.
void sitd_pool_init(void);
sitd_t *sitd_alloc(void);
void sitd_free(sitd_t *node);

// Fixed-capacity pool of iTDs, same conventions as the siTD pool above:
// DMA-reachable backing, (re)init before use, alloc returns NULL when
// exhausted, free requires the node be unlinked from the schedule.
void itd_pool_init(void);
itd_t *itd_alloc(void);
void itd_free(itd_t *node);

// Fill an iTD for one frame of high-speed isochronous OUT: up to eight
// microframe transactions from one contiguous DMA-reachable buffer, len[k]
// bytes in microframe k (0 = that microframe carries nothing and its
// transaction stays inactive). Page pointers are derived from the buffer
// being contiguous: page i is page 0 plus i*4K. ioc_last raises IOC on the
// last active transaction. Returns false (descriptor untouched or unarmed)
// on nulls, max_packet 0 or >1024, any len[k] > max_packet, all-zero
// lengths, or a buffer whose span would need more than seven pages.
bool itd_fill_out(itd_t *node, uint8_t dev_addr, uint8_t endpoint,
                  const void *buf, const uint16_t len[8], uint16_t max_packet,
                  bool ioc_last);

// Link an siTD into one periodic frame list slot, at the head of that frame's
// list. `frame_slot` is &periodictable[frame]; the caller owns the table.
//
// Head insertion is not a convenience -- EHCI requires isochronous descriptors
// to precede interrupt queue heads within a frame, and sitd_skip_iso() (and
// therefore add_qh_to_periodic_schedule) depends on that ordering holding.
//
// Whatever the slot pointed at becomes this siTD's next link, so interrupt
// queue heads already scheduled in this frame stay reachable.
// Fill an siTD for one full-speed isochronous OUT transaction.
//
// `hub_addr`/`port` identify the transaction translator. For a device attached
// directly to the root port both are 0 -- RM Table 62-56: "directly attached
// [where HubAddr = 0 and HubAddr is the address of the Root Hub where the bus
// transitions from HS to FS/LS]".
//
// `buf` must be DMA-reachable (USBHOST_DMAMEM on RT1176) and is not copied.
// Returns false if the transfer cannot be scheduled, in which case the siTD is
// left inactive rather than half-built.
bool sitd_fill_out(sitd_t *node, uint8_t dev_addr, uint8_t endpoint,
                   uint8_t hub_addr, uint8_t port, const void *buf,
                   uint16_t len, uint8_t start_uframe, bool ioc);

// Fill an siTD for one full-speed isochronous IN transaction. `len` is the
// room available in `buf` (which must be DMA-reachable); the device may send
// less, and the caller computes received = len - status.bytes_left after
// completion. TP/T-count are OUT-only fields (EHCI 1.0 Table 3-12), so buf1
// carries only the next-page pointer. The UAC1 feedback endpoint (3 bytes
// every 2^bRefresh ms) is the intended user.
bool sitd_fill_in(sitd_t *node, uint8_t dev_addr, uint8_t endpoint,
                  uint8_t hub_addr, uint8_t port, void *buf,
                  uint16_t len, uint8_t start_uframe, bool ioc);

// Completion status, read back from the descriptor after the controller has
// processed it. `active` false with no error flags and bytes_left 0 means the
// packet went out. This is the primary verification path: the controller
// reporting what it did beats inferring it from a logic capture.
typedef struct {
	bool     active;        // still queued -- hardware has not run it yet
	bool     err_transaction;
	bool     err_babble;
	bool     err_buffer;
	uint16_t bytes_left;    // bytes-to-transfer remaining; 0 means all sent
} sitd_status_t;

void sitd_get_status(const sitd_t *node, sitd_status_t *out);

void sitd_link(volatile uint32_t *frame_slot, sitd_t *node, uint16_t frame);

// Remove an siTD from a frame slot's list. Returns true if it was found and
// unlinked. Walks only the isochronous run at the head, since that is the
// only place an siTD can legally be.
bool sitd_unlink(volatile uint32_t *frame_slot, sitd_t *node);

#endif
