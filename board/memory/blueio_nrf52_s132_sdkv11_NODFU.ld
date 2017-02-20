/* Copyright 2017 Inca Roads LLC.  All rights reserved.
 * Use of this source code is governed by licenses granted by the 
 * copyright holder including that found in the LICENSE file.
 */

/* Linker script to configure memory regions
 *
 * For S130 SoftDevice V2.0, see p. 52 of http://infocenter.nordicsemi.com/pdf/S130_SDS_v2.0.pdf
 * This will tell you what to use for the FLASH ORIGIN in any given release of the SoftDevice.
 * Compute the LENGTH by doing 0x40000 - ORIGIN.
 *
 * That doc also tells you the APP_RAM_BASE minimum, with LENGTH being 0x8000 - ORIGIN.
 * However, this minimum will never work for SD and BLE enabled.
 * The way to truly determine the numbers is to do a clean build with:
 * a) the parameters for RAM ORIGIN and LENGTH set to the above minimums, and
 * b) a test Nano wired to have a serial console (baudrate: 57600)
 * c) In the makefile, the -DDEBUG_USES_UART should be on, and -DDEBUG_USES_BLUETOOTH off
 * d) in the makefile, the debug flag -DNRF_LOG_USES_RTT=1 should be asserted, so that
 *    the softdevice handler emit TWO sets of RAM start address warnings to serial
 *
 * The first one can be ignored, because it is just telling you the MINIMUM.
 * However, the warnings after "sd_ble_enable: RAM START AT 0x" will tell you exactly
 * what parameters you need below.  Once you use those parameters below, build
 * and verify that the warnings went away, then revert the makefile back to normal.
 *
 *  Note that in main.c, if ble_enable_params.common_enable_params.vs_uuid_count is ==
 *      3, then set base=2600/len=5a00
 *      4, then set base=2610/len=59f0
 */


SEARCH_DIR(.)
GROUP(-lgcc -lc -lnosys)

MEMORY
{
  FLASH (rx) : ORIGIN = 0x1c000, LENGTH = 0x64000
  RAM (rwx) :  ORIGIN = 0x20002600, LENGTH = 0xda00
}

SECTIONS
{
  .fs_data_out ALIGN(4):
  {
    PROVIDE( __start_fs_data = .);
    KEEP(*(fs_data))
    PROVIDE( __stop_fs_data = .);
  } = 0
}

INCLUDE "nrf5x_common.ld"
