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
#define MAX_PENDING_TWI_TRANSACTIONS 25
static app_twi_t m_app_twi = APP_TWI_INSTANCE(0);
static int InitCount = 0;
static int TransactionsInProgress = 0;
static int SchedulingErrors = 0;
static int CompletionErrors = 0;
static char ErrorLog[250] = "";

// Check the state of TWI
void twi_status_check(bool fVerbose) {
    DEBUG_PRINTF("TWI idle=%d init=%d t=%d se=%d ce=%d%s\n", app_twi_is_idle(&m_app_twi), InitCount, TransactionsInProgress, SchedulingErrors, CompletionErrors, fVerbose ? ErrorLog: "");
}

// Append to error log
void twi_err(char *prefix, char *context, ret_code_t error) {
    char buff[40];
    sprintf(buff, " %s:%s=%ld", prefix, context, error);
    if ((strlen(ErrorLog)+strlen(buff)) < (sizeof(ErrorLog)-2))
        strcat(ErrorLog, buff);
}

// Schedule a TWI transaction
bool twi_schedule(char *context, app_twi_transaction_t const * p_transaction) {
    ret_code_t error = app_twi_schedule(&m_app_twi, p_transaction);
    if (error != NRF_SUCCESS) {
        DEBUG_PRINTF("%s scheduling error: %d %04x\n", context, error, error);
        SchedulingErrors++;
        twi_err("S", context, error);
        return false;
    }
    TransactionsInProgress++;
    return true;
}

// Mark a TWI transaction as being completed
bool twi_completed(char *context, ret_code_t error) {
    TransactionsInProgress--;
    if (error != NRF_SUCCESS) {
        DEBUG_PRINTF("%s error: %d %04x\n", context, error, error);
        CompletionErrors++;
        twi_err("C", context, error);
        return false;
    }
    return true;
}

// Initialization of TWI subsystem
bool twi_init() {
    uint32_t err_code;

    // Exit if already initialized
    if (InitCount++ > 0) {

        if (debug(DBG_SENSOR_MAX))
            DEBUG_PRINTF("TWI Init nested, now %d users\n", InitCount);

        return true;
    }
    
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

    } else {

        if (debug(DBG_SENSOR_MAX))
            DEBUG_PRINTF("TWI Term nested, %d remaining)\n", InitCount);

    }
        

    return true;

}

#endif // TWIX
