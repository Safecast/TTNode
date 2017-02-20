// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

// TWI device/sensor processing

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "debug.h"
#include "boards.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_drv_twi.h"
#include "app_util_platform.h"
#include "gpio.h"
#include "config.h"
#include "timer.h"
#include "sensor.h"
#include "storage.h"
#include "comm.h"
#include "misc.h"
#include "twi.h"
#include "io.h"

#ifdef TWIX

// Maximum concurrent TWI commands
#define MAX_PENDING_TWI_TRANSACTIONS 12
static app_twi_t m_app_twi = APP_TWI_INSTANCE(0);
static int InitCount = 0;

// Get the address of the TWI context structure
app_twi_t *twi_context() {
    if (InitCount == 0)
        DEBUG_PRINTF("TWI not initialized!\n");
    return(&m_app_twi);
}
    
// Initialization of TWI subsystem
bool twi_init() {
    uint32_t err_code;

    // Exit if already initialized
    if (InitCount++ > 0)
        return true;
    
    if (debug(DBG_SENSOR_MAX))
        DEBUG_PRINTF("TWI Init\n");
    
    // Initialize TWI
    nrf_drv_twi_config_t const config = {
        .scl                = TWI_PIN_SCL,
        .sda                = TWI_PIN_SDA,
        .frequency          = NRF_TWI_FREQ_100K,
        .interrupt_priority = APP_IRQ_PRIORITY_LOW
    };
    APP_TWI_INIT(&m_app_twi, &config, MAX_PENDING_TWI_TRANSACTIONS, err_code);
    if (err_code != NRF_SUCCESS) {
        InitCount--;
        DEBUG_PRINTF("TWI init error = 0x%04x\n", err_code);
        return false;
    }

    return true;
}

// Termination of TWI, which must be precisely paired with calls to twi_init()
bool twi_term() {

    if (--InitCount == 0) {
        
        app_twi_uninit(&m_app_twi);
    
    if (debug(DBG_SENSOR_MAX))
        DEBUG_PRINTF("TWI Term\n");

    }

    return true;
    
}

#endif // TWIX
