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
