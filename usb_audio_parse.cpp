// Copyright (c) 2026 Nicholas Newdigate
// SPDX-License-Identifier: MIT
#include "usb_audio_parse.h"
#include <string.h>

bool uac1_parse_config(const uint8_t *desc, size_t len, UAC1Topology *out)
{
	(void)desc; (void)len; (void)out;
	return false;
}

int uac1_find_alt(const UAC1Topology *t, uint32_t rate, uint8_t channels, uint8_t bits)
{
	(void)t; (void)rate; (void)channels; (void)bits;
	return -1;
}
