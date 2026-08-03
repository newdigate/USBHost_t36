// Copyright (c) 2026 Nicholas Newdigate
// SPDX-License-Identifier: MIT
#include "usb_audio2_parse.h"

void uac2_clock_cur_setup(uint8_t setup[8], uint8_t ac_interface, uint8_t clock_id)
{
	setup[0] = 0x21;              // class, interface recipient, host-to-device
	setup[1] = 0x01;              // CUR
	setup[2] = 0x00;
	setup[3] = 0x01;              // CS_SAM_FREQ_CONTROL << 8
	setup[4] = ac_interface;
	setup[5] = clock_id;
	setup[6] = 0x04;              // wLength: 4-byte rate follows
	setup[7] = 0x00;
}
