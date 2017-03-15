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
#include "app_scheduler.h"
#include "timer.h"
#include "sensor.h"
#include "storage.h"
#include "comm.h"
#include "misc.h"
#include "twi.h"
#include "io.h"

#ifdef TWIX

// We use the app scheduler to ensure that we execute interrupt handlers safely
#define TWI_APP_SCHED

static twi_context_t transaction[25] = { { 0 } };

// Maximum concurrent TWI commands
#define MAX_PENDING_TWI_TRANSACTIONS 25
static app_twi_t m_app_twi = APP_TWI_INSTANCE(0);
static int InitCount = 0;
static int TransactionsInProgress = 0;
static int SchedulingErrors = 0;
static int CompletionErrors = 0;
static char ErrorLog[250] = "";

// Allocate a context block
static twi_context_t *find_transaction(char *c) {
    int i;

    // Attempt to find the transaction
    for (i=0; i<(sizeof(transaction) / sizeof(transaction[0])); i++)
        if (transaction[i].comment != NULL)
            if (strcmp(transaction[i].comment, c) == 0)
                return &transaction[i];

    // Allocate a slot
    for (i=0; i<(sizeof(transaction) / sizeof(transaction[0])); i++)
        if (transaction[i].comment == NULL) {
            transaction[i].index = i;
            transaction[i].comment = c;
            transaction[i].transactions_scheduled = 0;
            transaction[i].transactions_completed = 0;
            transaction[i].sensor = NULL;
            return &transaction[i];
        }

    // Can't happen
    while (true) {
        DEBUG_PRINTF("*** Array not big enough for the number of types of transaction\n");
        nrf_delay_ms(500);
    }

}

// Check the state of TWI
void twi_status_check(bool fVerbose) {

    // Debug messages
    if (fVerbose || SchedulingErrors || CompletionErrors) {
        DEBUG_PRINTF("TWI idle=%d init=%d t=%d se=%d ce=%d%s\n", app_twi_is_idle(&m_app_twi), InitCount, TransactionsInProgress, SchedulingErrors, CompletionErrors, ErrorLog);
    }

    // Display TWI transaction table
    if (fVerbose) {
        int i;
        char buffer[512];
        buffer[0] = '\0';
        for (i=0; i<(sizeof(transaction) / sizeof(transaction[0])); i++)
            if (transaction[i].comment != NULL && transaction[i].sensor != NULL) {
                char buff2[128];
                sprintf(buff2, "%s(%ld/%ld:%ld/%ld) ", transaction[i].comment, transaction[i].transactions_scheduled, transaction[i].transactions_completed, transaction[i].sched_error, transaction[i].transaction_error);
                strcat(buffer, buff2);
            }
        DEBUG_PRINTF("%s\n", buffer);
    }

    // See if any pending transactions are in a stuck state
    if (TransactionsInProgress != 0) {
        int i;
        for (i=0; i<(sizeof(transaction) / sizeof(transaction[0])); i++)
            if (transaction[i].transaction_began != 0) {
                twi_context_t *t = &transaction[i];
                if (!WouldSuppress(&t->transaction_began, 15))
                    DEBUG_PRINTF("*** %s HUNG: about to unconfigure and reset! ***\n", t->comment);
                if (!WouldSuppress(&t->transaction_began, 30)) {
                    // Unconfigure the sensor
                    sensor_unconfigure(t->sensor);
                    // Drain all TWI inits to zero
                    while (twi_term());
                    // Abort all in-progress transactions
                    sensor_abort_all();
                    // Clear local counters
                    InitCount = 0;
                    TransactionsInProgress = 0;
                    SchedulingErrors = CompletionErrors = 0;
                    ErrorLog[0] = '\0';
                }
            }
    }

}

// Append to error log
void twi_err(char *prefix, char *comment, ret_code_t error) {
    char buff[40];
    sprintf(buff, " %s:%s=%ld", prefix, comment, error);
    if ((strlen(ErrorLog)+strlen(buff)) < (sizeof(ErrorLog)-2))
        strcat(ErrorLog, buff);
}

// Process the callback at app sched level
void twi_callback_sched (void *p_event_data, uint16_t event_size) {
    uint16_t index = * (uint16_t *) p_event_data;
    twi_context_t *t = &transaction[index];
    t->callback(t->transaction_error, t);
}

// Our universal callback
void twi_callback(ret_code_t result, void *p_user_data) {

    // Find the transaction
    twi_context_t *t = find_transaction(p_user_data);

    // Mark it as completed
    t->transaction_began = 0;
    t->transaction_error = result;
    t->transactions_completed++;

    // Call the callback at app_sched level if we can
#ifdef TWI_APP_SCHED
    uint16_t index = t->index;
    if (app_sched_event_put(&index, sizeof(index), twi_callback_sched) != NRF_SUCCESS) {
        DEBUG_PRINTF("Can't schedule TWI callback!\n");
        t->callback(result, t);
    }
#else
    t->callback(result, t);
#endif

}

// Schedule a TWI transaction
bool twi_schedule(void *sensor, sensor_callback_t callback, app_twi_transaction_t const * p_transaction) {
    int i;
    twi_context_t *t = find_transaction(p_transaction->p_user_data);
    t->sensor = sensor;
    t->callback = (app_twi_callback_t) callback;
    for (i=0; i<5; i++) {
        t->sched_error = app_twi_schedule(&m_app_twi, p_transaction);
        if (t->sched_error != NRF_ERROR_BUSY)
            break;
        nrf_delay_ms(500);
    }
    if (t->sched_error != NRF_SUCCESS) {
        SchedulingErrors++;
        twi_err("S", t->comment, t->sched_error);
        return false;
    }
    t->transactions_scheduled++;
    t->transaction_began = get_seconds_since_boot();
    TransactionsInProgress++;
    return true;
}

// Mark a TWI transaction as being completed
bool twi_completed(twi_context_t *t) {
    // Bug check
    if (TransactionsInProgress == 0) {
        TransactionsInProgress = 0;
    }
    // Bump the transactions
    --TransactionsInProgress;
    // Handle errors
    if (t->transaction_error != NRF_SUCCESS) {
        CompletionErrors++;
        twi_err("C", t->comment, t->transaction_error);
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

    if (InitCount < 0) {
        DEBUG_PRINTF("What??\n");
        InitCount = 0;
        return false;
    }

    if (InitCount == 0)
        return false;

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
