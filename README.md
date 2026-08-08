# USBHost_t36

USB host library for Teensy 3.6 and Teensy 4.x — connect USB devices
(keyboards, mice, MIDI, mass storage, hubs, Bluetooth, joysticks, and more) to
the USB **host** port of the i.MX RT / Kinetis processor.

See the [examples](examples/) for usage. On Teensy 4.x the host runs on the
**USB2 / USB_OTG2** controller (separate from USB1, which is the device/Serial
port).

## NXP MIMXRT1060-EVKB (i.MX RT1062)

This library needs **no code changes** for the MIMXRT1060-EVKB — it is the same
i.MX RT1062 silicon as the Teensy 4.x, and the USB-host controller/PHY/PLL
bring-up is board-agnostic.

Wiring / usage notes for the EVKB:

- **Plug the USB device into connector `J47`** — that is the board's USB_OTG2
  port, which the EVKB designates as the **Host** port (User Manual, Table 18).
  `J48` is USB_OTG1 = Device mode (do not use it for host).
- **No software VBUS enable is required.** The host port's 5 V is supplied by a
  dedicated `USB_OTG2_VBUS` hardware rail (User Manual, Table 12 / power tree),
  with no GPIO power-switch or jumper. This differs from the Teensy 4.1, which
  drives `EMC_40` (GPIO8.26) to switch on host power.
- **Do not port the Teensy 4.1 VBUS code to the EVKB.** On the EVKB `EMC_40` is
  part of the 32 MB SDRAM (SEMC) bus, so driving it would clash with the SDRAM.
  The `#ifdef ARDUINO_TEENSY41` guard in `ehci.cpp` correctly leaves it out.
- Keep the board adequately powered (e.g. via `J1`/`J45`), since bus-powered
  devices draw their current from the board's 5 V.

Note: this has been confirmed by source review and a clean compile for
`teensy:avr:mimxrt1060evkb`, and — since 2026-08-08 — by an automated QEMU gate
that enumerates an emulated UAC1 device on the RT1062's OTG2 host controller.
(An earlier revision of this note said the i.MX RT1062 QEMU model was
device-mode only. That was true when written and is no longer: `TYPE_CHIPIDEA`
derives from `TYPE_SYS_BUS_EHCI` and host support is shared across the RT1062
and RT1176 SoC models.)

★ **The RT1062's USB DMA master cannot reach DTCM either, so these buffers need
`DMAMEM` on this platform too.** Established on a MIMXRT1060-EVKB bench
2026-08-08, and it is worth stating plainly because it means **upstream has a
latent bug on its own home silicon**: `periodictable`, `enumbuf`, `enumsetup`
and the `memory.cpp` seed pools are declared without `DMAMEM`, so they land in
`.bss` — and `.bss` is **DTCM** in `imxrt1062.ld`, `imxrt1062_t41.ld` and
`imxrt1060_evkb.ld` alike. Only `.bss.dma`, which is what `DMAMEM` selects,
reaches OCRAM at `0x20200000`.

With `periodictable` at `0x20002000` (DTCM), the controller reported:

```
PORTSC1 = 0x10001805   CCS=1 PE=1 PP=1 -- device connected, powered, enabled
USBSTS  = 0x0000d09a   HCH=1 (halted) + SEI=1 (system error)
```

`SEI` is the controller faulting on its own DMA fetch of the periodic list.
Rebuilding with these buffers in `DMAMEM`/OCRAM cleared it — `USBSTS` became
`0x0000d080`, same board, same cable, one variable changed.

Two earlier revisions of this note got the reasoning wrong and are corrected
here: first that "`.bss` is already OCRAM on Teensy" (it is not), then that the
RT1062 could reach DTCM anyway because upstream works on a Teensy 4.1 (an
inference from upstream working, which the bench refuted).

**This does not mean USB host works on the EVKB.** Clearing `SEI` did not make
the port enumerate — the controller is still halted (`HCH=1`, `SEI` now clear)
and a second, unrelated failure remains open on that board.
