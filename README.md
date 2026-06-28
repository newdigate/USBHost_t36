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
`teensy:avr:mimxrt1060evkb`. It is not exercised under the i.MX RT1062 QEMU
model, whose USB controller is device-mode only (no EHCI host emulation).
