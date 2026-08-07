/* Vector-table entry point for cores whose interrupt table is STATIC.
 *
 * The CM7 installs USBHost::isr() at run time with attachInterruptVector() and
 * never needs this file. The i.MX RT1176's Cortex-M4 cannot: its vector table
 * is a flat array of C function pointers linked at build time, and
 * USBHost::isr is both private and C++-mangled. This gives that table a plain
 * C symbol to name.
 *
 * ★ It lives in its OWN translation unit rather than in ehci.cpp, and that is
 * deliberate. Defining it inside ehci.cpp perturbed input-section ordering
 * enough to move a call target, and all nine dependent images in the
 * rt1176-evkb tree changed byte-wise -- identical sizes, shifted addresses --
 * even though --gc-sections dropped the function itself. Functionally inert,
 * but it spends a regression signal this project relies on. In a separate file
 * that only static-vector builds add to their source list, images that do not
 * use it are untouched: nothing to compile, nothing to order, nothing to drop.
 *
 * Access to the private isr() is via a friend declaration in USBHost_t36.h,
 * not by making it public -- nothing else has any business calling the ISR.
 */
#include "USBHost_t36.h"

extern "C" void usbhost_isr_entry(void)
{
	USBHost::isr();
}
