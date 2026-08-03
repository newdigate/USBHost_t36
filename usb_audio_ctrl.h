// Copyright (c) 2026 Nicholas Newdigate
// SPDX-License-Identifier: MIT
//
// Watchdog policy for the audio driver's post-claim control sequence.
//
// Why a watchdog exists at all: a USB driver in this stack is told when one
// of its control transfers COMPLETES CLEANLY and at no other time.
// followup_Transfer() (ehci.cpp) calls the pipe's callback only when the qTD
// token shows neither active nor error bits, and an errored transfer is
// routed instead to the pipe's error_callback_function -- which for a control
// pipe is permanently the core's enumeration_error() (enumeration.cpp), a
// function that retries enumeration and never notifies the driver that owns
// the request. So a stalled, errored or simply never-completing request
// leaves the driver's sequence state frozen with nothing to un-freeze it.
//
// Measured on the RT1176 EVKB, 2026-08-03: a device that re-enumerated while
// its clock SET_CUR was outstanding left the driver at CTRL_SET_CLOCK for 71
// seconds and counting -- alternate setting never valid, only a host reset
// recovering it.
//
// Pure arithmetic, no Arduino or USBHost_t36 dependency, so the policy is
// unit-tested on the host (test/test_ctrl.cpp) rather than only on a bench
// that has to be provoked into re-enumerating at the wrong moment.
#ifndef USB_AUDIO_CTRL_H_
#define USB_AUDIO_CTRL_H_
#include <stdint.h>

typedef enum {
	UAC_CTRL_WAIT = 0,   // nothing outstanding, or still inside its window
	UAC_CTRL_RETRY,      // deadline passed and attempts remain: reissue
	UAC_CTRL_GIVE_UP     // deadline passed and attempts are spent
} uac_ctrl_action_t;

// One watchdog decision.
//
// `busy` is whether a request is outstanding; when false the result is always
// WAIT, so a stale started_ms left behind by a sequence that finished normally
// can never restart a working stream. Elapsed time is computed with unsigned
// subtraction, which is correct across the ~49.7 day millis() wrap. The
// deadline is reached AT timeout_ms. `attempts` is how many have already been
// spent, so attempts >= max_attempts gives up.
uac_ctrl_action_t uac_ctrl_poll(bool busy, uint32_t started_ms, uint32_t now_ms,
                                uint32_t timeout_ms, uint8_t attempts,
                                uint8_t max_attempts);

#endif // USB_AUDIO_CTRL_H_
