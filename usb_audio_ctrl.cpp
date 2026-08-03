// Copyright (c) 2026 Nicholas Newdigate
// SPDX-License-Identifier: MIT
#include "usb_audio_ctrl.h"

uac_ctrl_action_t uac_ctrl_poll(bool busy, uint32_t started_ms, uint32_t now_ms,
                                uint32_t timeout_ms, uint8_t attempts,
                                uint8_t max_attempts)
{
	if (!busy) return UAC_CTRL_WAIT;
	// Unsigned subtraction: correct across the millis() wrap, where
	// now_ms < started_ms but the true elapsed time is small.
	uint32_t elapsed = now_ms - started_ms;
	if (elapsed < timeout_ms) return UAC_CTRL_WAIT;
	return (attempts < max_attempts) ? UAC_CTRL_RETRY : UAC_CTRL_GIVE_UP;
}
