// Copyright (c) 2026 Nicholas Newdigate
// SPDX-License-Identifier: MIT
//
// Sample FIFO between an audio producer and the USB isochronous ring.
//
// This is the seam the design puts between USBHost_t36 and the Audio library:
// the producer pushes interleaved samples, the ring pulls whole frames. Keeping
// it a plain ring of int16_t means USBHost_t36 never needs to know what
// audio_block_t is, and this file has no Arduino or USBHost_t36 dependency so
// it can be host-tested.
//
// Single-producer / single-consumer. The consumer runs from the USB service
// path and the producer from the audio graph, so head and tail are each written
// by exactly one side.
#ifndef USB_AUDIO_FIFO_H_
#define USB_AUDIO_FIFO_H_
#include <stdint.h>
#include <stddef.h>

// Roughly 46 ms of stereo at 44.1 kHz. Big enough that a late producer does not
// immediately underrun, small enough not to add meaningful latency on top of
// the ring's own depth.
#define USB_AUDIO_FIFO_SAMPLES 4096

typedef struct {
	int16_t  buf[USB_AUDIO_FIFO_SAMPLES];
	volatile uint32_t head;   // producer writes
	volatile uint32_t tail;   // consumer writes
} usb_audio_fifo_t;

void usb_audio_fifo_reset(usb_audio_fifo_t *f);

// Samples currently readable, and free space for writing.
uint32_t usb_audio_fifo_used(const usb_audio_fifo_t *f);
uint32_t usb_audio_fifo_free(const usb_audio_fifo_t *f);

// Push up to `count` samples. Returns how many were actually taken, which is
// fewer than requested when the FIFO is full -- the caller decides whether to
// drop or retry rather than having that policy baked in here.
uint32_t usb_audio_fifo_write(usb_audio_fifo_t *f, const int16_t *src, uint32_t count);

// Pop exactly `count` samples. Returns false and takes nothing if fewer are
// available: a partially filled USB frame would be worse than a silent one,
// because it shifts every later sample and the device has no way to resync.
bool usb_audio_fifo_read(usb_audio_fifo_t *f, int16_t *dst, uint32_t count);

#endif
