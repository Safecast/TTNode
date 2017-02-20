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
 *
 *  Note that before the iphone 7 MTU hackery, RAM (rwx) :  ORIGIN = 0x200027b8, LENGTH = 0xd848
 *
 *  NOTE for NRF52 DFU I found fixes here:
 *  https://devzone.nordicsemi.com/question/95193/sdk12pca10040ble_app_buttonless_dfu-not-resetting-to-bootloader/
 */


SEARCH_DIR(.)
GROUP(-lgcc -lc -lnosys)

MEMORY
{
  FLASH (rx) : ORIGIN = 0x1f000, LENGTH = 0x61000
  RAM (rwx) :  ORIGIN = 0x20003a40, LENGTH = 0xc5c0

  /** Location of non initialized RAM. Non initialized RAM is used for exchanging bond information
   *  from application to bootloader when using buttonluss DFU OTA.
   */
    NOINIT (rwx) :  ORIGIN = 0x20007F80, LENGTH = 0x80

  /** Location of bootloader setting in flash. */
    BOOTLOADER_SETTINGS (rw) : ORIGIN = 0x0007F000, LENGTH = 0x1000 

  /** Location in UICR where bootloader start address is stored. */
    UICR_BOOTLOADER (r) : ORIGIN = 0x10001014, LENGTH = 0x04

  /** Location of mbr params page in flash. */
    MBR_PARAMS_PAGE (rw) : ORIGIN = 0x0007E000, LENGTH = 0x1000

  /** Location in UICR where mbr params page address is stored. */
    UICR_MBR_PARAM_PAGE(r) : ORIGIN = 0x10001018, LENGTH = 0x04
    
}

SECTIONS
{

  /* Place the bootloader settings page in flash. */
    .bootloaderSettings(NOLOAD) :
      {

  } > BOOTLOADER_SETTINGS

  /* Write the bootloader address in UICR. */
    .uicrBootStartAddress :
      {
          KEEP(*(.uicrBootStartAddress))
            } > UICR_BOOTLOADER

  /* Place the mbr params page in flash. */
      .mbrParamsPage(NOLOAD) :
        {

  } > MBR_PARAMS_PAGE

  /* No init RAM section in bootloader. Used for bond information exchange. */
    .noinit(NOLOAD) :
      {

  } > NOINIT
    /* other placements follow here... */
    
  .fs_data :
  {
    PROVIDE(__start_fs_data = .);
    KEEP(*(.fs_data))
    PROVIDE(__stop_fs_data = .);
  } > RAM
} INSERT AFTER .data;

INCLUDE "nrf5x_common.ld"
