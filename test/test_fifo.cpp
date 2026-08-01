// Copyright (c) 2026 Nicholas Newdigate
// SPDX-License-Identifier: MIT
#include "usb_audio_fifo.h"
#include <stdio.h>

static int failures = 0, checks = 0;
#define CHECK_EQ(a, b) do { checks++; long _ck_a = (long)(a), _ck_b = (long)(b); \
	if (_ck_a != _ck_b) { printf("FAIL %s:%d: %s == %s (got %ld, want %ld)\n", \
	__FILE__, __LINE__, #a, #b, _ck_a, _ck_b); failures++; } } while (0)

static usb_audio_fifo_t f;
static int16_t src[8192], dst[8192];

static void test_basic(void)
{
	usb_audio_fifo_reset(&f);
	CHECK_EQ(usb_audio_fifo_used(&f), 0);
	CHECK_EQ(usb_audio_fifo_free(&f), USB_AUDIO_FIFO_SAMPLES - 1);

	for (int i = 0; i < 100; i++) src[i] = (int16_t)(i * 37);
	CHECK_EQ(usb_audio_fifo_write(&f, src, 100), 100);
	CHECK_EQ(usb_audio_fifo_used(&f), 100);

	CHECK_EQ(usb_audio_fifo_read(&f, dst, 100), true);
	CHECK_EQ(usb_audio_fifo_used(&f), 0);
	for (int i = 0; i < 100; i++) CHECK_EQ(dst[i], (int16_t)(i * 37));
}

static void test_partial_read_refused(void)
{
	// A short frame is worse than a silent one: it shifts every later sample
	// and the device cannot resync. So an under-filled read must take nothing.
	usb_audio_fifo_reset(&f);
	CHECK_EQ(usb_audio_fifo_write(&f, src, 50), 50);
	CHECK_EQ(usb_audio_fifo_read(&f, dst, 100), false);
	CHECK_EQ(usb_audio_fifo_used(&f), 50);      // nothing consumed
	CHECK_EQ(usb_audio_fifo_read(&f, dst, 50), true);
}

static void test_full(void)
{
	usb_audio_fifo_reset(&f);
	uint32_t cap = usb_audio_fifo_free(&f);
	CHECK_EQ(usb_audio_fifo_write(&f, src, 8192), cap);   // clipped, not overrun
	CHECK_EQ(usb_audio_fifo_free(&f), 0);
	CHECK_EQ(usb_audio_fifo_write(&f, src, 1), 0);        // no space left
}

static void test_wraparound(void)
{
	// Drive head and tail several times round the buffer, checking data
	// integrity across the wrap rather than just near the start.
	usb_audio_fifo_reset(&f);
	int16_t v = 0;
	int16_t expect = 0;
	for (int round = 0; round < 40; round++) {
		int16_t chunk[181];
		for (int i = 0; i < 181; i++) chunk[i] = v++;
		CHECK_EQ(usb_audio_fifo_write(&f, chunk, 181), 181);
		CHECK_EQ(usb_audio_fifo_read(&f, dst, 181), true);
		for (int i = 0; i < 181; i++) CHECK_EQ(dst[i], expect++);
	}
}

static void test_frame_sized_traffic(void)
{
	// The real access pattern: the graph pushes 128-sample blocks, the ring
	// pulls alternating 88/90-sample stereo frames (44/45 stereo samples at
	// 44.1 kHz). These do not divide evenly, which is the point.
	usb_audio_fifo_reset(&f);
	int16_t block[256];
	for (int i = 0; i < 256; i++) block[i] = (int16_t)i;

	uint32_t produced = 0, consumed = 0;
	for (int i = 0; i < 500; i++) {
		if (usb_audio_fifo_free(&f) >= 256) {
			produced += usb_audio_fifo_write(&f, block, 256);
		}
		uint32_t want = (i % 10 == 9) ? 90 : 88;
		if (usb_audio_fifo_read(&f, dst, want)) consumed += want;
	}
	CHECK_EQ(consumed > 0, true);
	CHECK_EQ(consumed <= produced, true);
	CHECK_EQ(usb_audio_fifo_used(&f), produced - consumed);
}

static void test_null_safety(void)
{
	CHECK_EQ(usb_audio_fifo_used(0), 0);
	CHECK_EQ(usb_audio_fifo_free(0), 0);
	CHECK_EQ(usb_audio_fifo_write(0, src, 10), 0);
	CHECK_EQ(usb_audio_fifo_write(&f, 0, 10), 0);
	CHECK_EQ(usb_audio_fifo_read(0, dst, 10), false);
	CHECK_EQ(usb_audio_fifo_read(&f, 0, 10), false);
	usb_audio_fifo_reset(0);
}

int main(void)
{
	test_basic();
	test_partial_read_refused();
	test_full();
	test_wraparound();
	test_frame_sized_traffic();
	test_null_safety();
	printf("%d checks, %d failures\n", checks, failures);
	return failures ? 1 : 0;
}
