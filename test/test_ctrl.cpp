// Copyright (c) 2026 Nicholas Newdigate
// SPDX-License-Identifier: MIT
#include "usb_audio_ctrl.h"
#include <stdio.h>

static int failures = 0, checks = 0;
#define CHECK_EQ(a, b) do { checks++; long _ck_a = (long)(a), _ck_b = (long)(b); \
	if (_ck_a != _ck_b) { printf("FAIL %s:%d: %s == %s (got %ld, want %ld)\n", \
	__FILE__, __LINE__, #a, #b, _ck_a, _ck_b); failures++; } } while (0)

#define TIMEOUT 500u
#define MAXTRY  3u

static void test_idle_never_fires(void)
{
	// Nothing outstanding: the watchdog must stay silent no matter how old
	// the timestamp is. A configuration sequence that finished normally
	// leaves a stale started_ms behind, and re-firing on it would restart a
	// device that is streaming happily.
	CHECK_EQ(uac_ctrl_poll(false, 0, 100000, TIMEOUT, 0, MAXTRY), UAC_CTRL_WAIT);
	CHECK_EQ(uac_ctrl_poll(false, 0, 0, TIMEOUT, 0, MAXTRY), UAC_CTRL_WAIT);
	CHECK_EQ(uac_ctrl_poll(false, 12345, 12345 + TIMEOUT * 10, TIMEOUT, 2, MAXTRY),
	         UAC_CTRL_WAIT);
}

static void test_deadline_boundary(void)
{
	// Outstanding and inside the window: keep waiting. The deadline is
	// reached AT timeout_ms, not after it -- pinned on both sides so an
	// off-by-one in either direction fails.
	CHECK_EQ(uac_ctrl_poll(true, 1000, 1000, TIMEOUT, 0, MAXTRY), UAC_CTRL_WAIT);
	CHECK_EQ(uac_ctrl_poll(true, 1000, 1000 + TIMEOUT - 1, TIMEOUT, 0, MAXTRY),
	         UAC_CTRL_WAIT);
	CHECK_EQ(uac_ctrl_poll(true, 1000, 1000 + TIMEOUT, TIMEOUT, 0, MAXTRY),
	         UAC_CTRL_RETRY);
	CHECK_EQ(uac_ctrl_poll(true, 1000, 1000 + TIMEOUT + 1, TIMEOUT, 0, MAXTRY),
	         UAC_CTRL_RETRY);
}

static void test_attempts_exhaust(void)
{
	// Each expiry consumes an attempt; the caller passes how many have been
	// spent. Retry while any remain, then give up rather than hammering a
	// device that is not answering.
	CHECK_EQ(uac_ctrl_poll(true, 0, TIMEOUT, TIMEOUT, 0, MAXTRY), UAC_CTRL_RETRY);
	CHECK_EQ(uac_ctrl_poll(true, 0, TIMEOUT, TIMEOUT, 1, MAXTRY), UAC_CTRL_RETRY);
	CHECK_EQ(uac_ctrl_poll(true, 0, TIMEOUT, TIMEOUT, 2, MAXTRY), UAC_CTRL_RETRY);
	CHECK_EQ(uac_ctrl_poll(true, 0, TIMEOUT, TIMEOUT, 3, MAXTRY), UAC_CTRL_GIVE_UP);
	CHECK_EQ(uac_ctrl_poll(true, 0, TIMEOUT, TIMEOUT, 200, MAXTRY), UAC_CTRL_GIVE_UP);

	// A zero retry budget gives up on the first expiry.
	CHECK_EQ(uac_ctrl_poll(true, 0, TIMEOUT, TIMEOUT, 0, 0), UAC_CTRL_GIVE_UP);
}

static void test_millis_wraparound(void)
{
	// millis() wraps every ~49.7 days. Unsigned subtraction gives the true
	// elapsed time across the wrap; a signed or naive `now > started + t`
	// comparison would either fire instantly or never fire again for weeks.
	uint32_t near_end = 0xFFFFFF00u;

	// 0x100 ms before the wrap, 0xFF ms elapsed: still inside a 500 ms window.
	CHECK_EQ(uac_ctrl_poll(true, near_end, 0xFFFFFFFFu, TIMEOUT, 0, MAXTRY),
	         UAC_CTRL_WAIT);
	// Past the wrap, elapsed = 256 + now: 256 + 100 = 356 ms, still inside.
	CHECK_EQ(uac_ctrl_poll(true, near_end, 100u, TIMEOUT, 0, MAXTRY),
	         UAC_CTRL_WAIT);
	// One tick short of the deadline: 256 + 243 = 499 ms.
	CHECK_EQ(uac_ctrl_poll(true, near_end, 243u, TIMEOUT, 0, MAXTRY),
	         UAC_CTRL_WAIT);
	// 256 + 244 = 500 ms elapsed exactly: the deadline, reached across
	// the wrap rather than in spite of it.
	CHECK_EQ(uac_ctrl_poll(true, near_end, 244u, TIMEOUT, 0, MAXTRY),
	         UAC_CTRL_RETRY);
	// Well past.
	CHECK_EQ(uac_ctrl_poll(true, near_end, 5000u, TIMEOUT, 0, MAXTRY),
	         UAC_CTRL_RETRY);
	// And the exact wrap point itself: started at the last tick.
	CHECK_EQ(uac_ctrl_poll(true, 0xFFFFFFFFu, 0u, TIMEOUT, 0, MAXTRY),
	         UAC_CTRL_WAIT);
	CHECK_EQ(uac_ctrl_poll(true, 0xFFFFFFFFu, TIMEOUT - 1u, TIMEOUT, 0, MAXTRY),
	         UAC_CTRL_RETRY);
}

static void test_zero_timeout(void)
{
	// A zero window means "expired the moment it was issued" -- degenerate,
	// but it must resolve to a decision rather than to arithmetic that
	// wraps back into WAIT.
	CHECK_EQ(uac_ctrl_poll(true, 1000, 1000, 0, 0, MAXTRY), UAC_CTRL_RETRY);
	CHECK_EQ(uac_ctrl_poll(true, 1000, 1000, 0, MAXTRY, MAXTRY), UAC_CTRL_GIVE_UP);
}

int main(void)
{
	test_idle_never_fires();
	test_deadline_boundary();
	test_attempts_exhaust();
	test_millis_wraparound();
	test_zero_timeout();
	if (failures == 0) { printf("test_ctrl: all %d checks passed\n", checks); return 0; }
	printf("test_ctrl: %d/%d FAILED\n", failures, checks);
	return 1;
}
