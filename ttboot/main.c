// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

// Bootloader for TTNode firmware

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "boards.h"
#include "nrf_delay.h"
#include "nrf_mbr.h"
#include "nrf_bootloader.h"
#include "nrf_bootloader_app_start.h"
#include "nrf_dfu.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "app_error.h"
#include "app_error_weak.h"
#include "nrf_bootloader_info.h"
#include "app_scheduler.h"
#include "app_timer_appsh.h"


#ifdef DFUFONA
#include "fona.h"
#endif

// Main bootloader entry
int main(void) {

    
    // Init the log package
    (void) NRF_LOG_INIT(NULL);
    NRF_LOG_INFO("TTBOOT: Inside main!\r\n");

    // Initialize our fona transport
    bool fSuccessfulInit = true;
#ifdef DFUFONA
    fSuccessfulInit = fona_dfu_init();
#endif

    // When doing BLE DFU testing, init stuff used for triggering and feedback of BLE DFU
#ifndef DFUFONA
    int pin;
    for (pin=LED_START; pin<=LED_STOP; pin++) {
        nrf_gpio_cfg_output(pin);
        nrf_gpio_pin_set(pin);
    }
    nrf_gpio_cfg_sense_input(BOOTLOADER_BUTTON, BUTTON_PULL, NRF_GPIO_PIN_SENSE_LOW);
#endif

    // Init the bootloader if we successfully initialized fona
    if (fSuccessfulInit) {
        NRF_LOG_INFO("TTBOOT: About to init bootloader\r\n");
        uint32_t ret_val = nrf_bootloader_init();
        APP_ERROR_CHECK(ret_val);
    }

    // Either there was no DFU functionality enabled in this project or the DFU module detected
    // no ongoing DFU operation and found a valid main application.
    NRF_LOG_INFO("TTBOOT: No DFU requested, so starting main application\r\n");

    // Halt serial I/O, so that we don't call this stuff after we're prepared to
    // deal with it at interrupt level
    fona_dfu_term();
    
    // Boot the main application.
    nrf_bootloader_app_start(MAIN_APPLICATION_START_ADDR);

    // Should never be reached.
    NRF_LOG_INFO("TTBOOT: Can't happen\r\n");

}
