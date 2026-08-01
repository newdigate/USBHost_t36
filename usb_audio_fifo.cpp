// Copyright (c) 2026 Nicholas Newdigate
// SPDX-License-Identifier: MIT
#include "usb_audio_fifo.h"

// One slot is always left empty so head == tail unambiguously means "empty"
// rather than needing a separate count that both sides would have to update.
#define FIFO_MASK (USB_AUDIO_FIFO_SAMPLES - 1u)

void usb_audio_fifo_reset(usb_audio_fifo_t *f)
{
	if (!f) return;
	f->head = 0;
	f->tail = 0;
}

uint32_t usb_audio_fifo_used(const usb_audio_fifo_t *f)
{
	if (!f) return 0;
	return (f->head - f->tail) & FIFO_MASK;
}

uint32_t usb_audio_fifo_free(const usb_audio_fifo_t *f)
{
	if (!f) return 0;
	return FIFO_MASK - usb_audio_fifo_used(f);
}

uint32_t usb_audio_fifo_write(usb_audio_fifo_t *f, const int16_t *src, uint32_t count)
{
	if (!f || !src) return 0;

	uint32_t space = usb_audio_fifo_free(f);
	if (count > space) count = space;

	uint32_t h = f->head;
	for (uint32_t i = 0; i < count; i++) {
		f->buf[h] = src[i];
		h = (h + 1u) & FIFO_MASK;
	}
	f->head = h;          // publish only once the data is in place
	return count;
}

bool usb_audio_fifo_read(usb_audio_fifo_t *f, int16_t *dst, uint32_t count)
{
	if (!f || !dst) return false;
	if (usb_audio_fifo_used(f) < count) return false;

	uint32_t t = f->tail;
	for (uint32_t i = 0; i < count; i++) {
		dst[i] = f->buf[t];
		t = (t + 1u) & FIFO_MASK;
	}
	f->tail = t;          // release the space only once the data is out
	return true;
}
