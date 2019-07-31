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
#include "stats.h"
#include "ssd.h"

#ifdef TWIX

// We use the app scheduler to ensure that we execute interrupt handlers safely
#define TWI_APP_SCHED

#define MAX_PENDING_TWI_TRANSACTIONS 100
static twi_context_t transaction[MAX_PENDING_TWI_TRANSACTIONS] = { { 0 } };

// Maximum concurrent TWI commands
static app_twi_t m_app_twi = APP_TWI_INSTANCE(0);
static int disable_twi_debug_printf = 0;
static int InitCount = 0;
static int TransactionsInProgress = 0;
static int SchedulingErrors = 0;
static int CompletionErrors = 0;

// TWI configuration
nrf_drv_twi_config_t const config = {
    .scl                = TWI_PIN_SCL,
    .sda                = TWI_PIN_SDA,
    .frequency          = NRF_TWI_FREQ_100K,
    .interrupt_priority = APP_IRQ_PRIORITY_LOW
};

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
            transaction[i].callback = NULL;
            return &transaction[i];
        }

    // Can't happen
    while (true) {
        DEBUG_PRINTF("*** Array not big enough for the number of types of transaction\n");
        nrf_delay_ms(500);
    }

}

// Used so that we don't go recursive in DEBUG_PRINTF
bool twi_disable_twi_debug_printf() {
    return (disable_twi_debug_printf != 0);
    
}

// Bump errors and append to error log
void report_err() {
    int i;
    stats()->errors_twi++;
    stats()->errors_twi_info[0] = '\0';
    for (i=0; i<(sizeof(transaction) / sizeof(transaction[0])); i++)
        if (transaction[i].comment != NULL && transaction[i].comment[0] != '~' && transaction[i].callback != NULL) {
            if (transaction[i].sched_error || transaction[i].transaction_error) {
                char buff[40];
                sprintf(buff, "%s%s:%s%ld",
                        stats()->errors_twi_info[0] == '\0' ? "" : " ",
                        transaction[i].comment,
                        transaction[i].sched_error ? "S" : "C",
                        transaction[i].sched_error ? transaction[i].sched_error : transaction[i].transaction_error);
                // Only copy whole errors into the buffer
                if ((strlen(stats()->errors_twi_info)+strlen(buff)) < (sizeof(stats()->errors_twi_info)-2))
                    strcat(stats()->errors_twi_info, buff);
            }
        }
}

// Check the state of TWI
void twi_status_check(bool fVerbose) {
    static uint32_t suppress = 0;

    // Don't allow recursion because of DEBUG_PRINTF
    disable_twi_debug_printf++;

    // Output critical debug messages periodically
    if (!ShouldSuppress(&suppress, 300))
        if (stats()->errors_twi_info[0] != '\0')
            DEBUG_PRINTF("TWI errors: %s\n", stats()->errors_twi_info);

    if (fVerbose) {
        DEBUG_PRINTF("TWI idle=%d init=%d t=%d se=%d ce=%d %s\n", app_twi_is_idle(&m_app_twi), InitCount, TransactionsInProgress, SchedulingErrors, CompletionErrors, stats()->errors_twi_info);
    }

    // Display TWI transaction table
    if (fVerbose) {
        int i;
        char buffer[512];
        buffer[0] = '\0';
        for (i=0; i<(sizeof(transaction) / sizeof(transaction[0])); i++)
            if (transaction[i].comment != NULL && transaction[i].comment[0] != '~' && transaction[i].callback != NULL) {
                char buff2[128];
                sprintf(buff2, "%s(%ld/%ld:%ld/%ld) ", transaction[i].comment, transaction[i].transactions_scheduled, transaction[i].transactions_completed, transaction[i].sched_error, transaction[i].transaction_error);
                strcat(buffer, buff2);
            }
        DEBUG_PRINTF("%s\n", buffer);
    }

    // See if any pending transactions are in a stuck state
    bool fTerminatedTWI = false;
    if (TransactionsInProgress != 0) {
        int i, j;
        for (i=0; i<(sizeof(transaction) / sizeof(transaction[0])); i++) {
            twi_context_t *t = &transaction[i];
            // If the comment begins with "~", suppress errors and TWI reset
            if (t->transaction_began != 0 && t->comment != NULL && t->comment[0] != '~') {
                if (!WouldSuppress(&t->transaction_began, 30)) {
                    DEBUG_PRINTF("*** %s HUNG: unconfiguring, resetting TWI ***\n", t->comment);
                    // Substitute a special timeout error if we hang
                    if (t->transaction_error == NRF_SUCCESS) {
                        t->transaction_error = 99;
                        report_err();
                    }
                    // Unconfigure the sensor if not in burn test mode, else merely get it unstuck
                    if (t->sensor != NULL)
                        sensor_unconfigure(t->sensor);
                    if (!fTerminatedTWI) {
                        fTerminatedTWI = true;
                        // Drain all TWI inits to zero
                        while (twi_term());
#ifdef SSD
                        // Reset the display subsystem, because it cannot be deconfigured and
                        // if TWI is reset out from under it there is havoc.
                        ssd1306_force_reset();
#endif
                        // Abort in-progress transactions for ALL twi-based sensors
                        for (j=0; j<(sizeof(transaction) / sizeof(transaction[0])); j++)
                            if (transaction[j].transaction_began != 0) {
                                transaction[j].transaction_began = 0;
                                if (t->sensor != NULL)
                                    sensor_abort(t->sensor);
                            }
                    }
                    // Clear local counters
                    InitCount = 0;
                    TransactionsInProgress = 0;
                    SchedulingErrors = CompletionErrors = 0;
                }
            }
        }
    }

    disable_twi_debug_printf--;
    return;
}

// Process the callback at app sched level
void callback_sched (void *p_event_data, uint16_t event_size) {
    disable_twi_debug_printf++;
    uint16_t index = * (uint16_t *) p_event_data;
    twi_context_t *t = &transaction[index];
    t->callback(t->transaction_error, t);
    disable_twi_debug_printf--;
}

// Our universal callback
void twi_callback(ret_code_t result, void *p_user_data) {

    // Don't allow recursion because of DEBUG_PRINTF
    disable_twi_debug_printf++;

    // Find the transaction
    twi_context_t *t = find_transaction(p_user_data);

    // Mark it as completed
    t->transaction_began = 0;
    t->transaction_error = result;
    t->transactions_completed++;

    // Call the callback at app_sched level if we can
#ifdef TWI_APP_SCHED
    uint16_t index = t->index;
    if (app_sched_event_put(&index, sizeof(index), callback_sched) != NRF_SUCCESS) {
        t->callback(result, t);
    }
#else
    t->callback(result, t);
#endif

    // Done
    disable_twi_debug_printf--;
}

// Schedule a TWI transaction
bool twi_schedule(void *sensor, sensor_callback_t callback, app_twi_transaction_t const * p_transaction) {
    int i;
    twi_context_t *t;

    // Don't allow recursion because of DEBUG_PRINTF
    disable_twi_debug_printf++;

    // Find this transaction
    t = find_transaction(p_transaction->p_user_data);

    // This is a bug check that prevents one TWI transaction from being scheduled on top of
    // an instance of itself.  This will only protect us a single time, because we set the
    // transaction_began to 0, however it is better than blocking TWI transactions indefinitely.
    if (t->transaction_began != 0) {
        DEBUG_PRINTF("%s TWI double-schedule\n", p_transaction->p_user_data);
        nrf_delay_ms(250);
        t->transaction_began = 0;
        disable_twi_debug_printf--;
        return false;
    }
    t->sensor = sensor;
    t->callback = (app_twi_callback_t) callback;
    for (i=0; i<5; i++) {
        t->sched_error = app_twi_schedule(&m_app_twi, p_transaction);
        if (t->sched_error != NRF_ERROR_BUSY)
            break;
        DEBUG_PRINTF("%s busy\n", p_transaction->p_user_data);
        nrf_delay_ms(500);
    }
    if (t->sched_error != NRF_SUCCESS) {
        SchedulingErrors++;
        report_err();
        disable_twi_debug_printf--;
        return false;
    }
    t->transactions_scheduled++;
    t->transaction_began = get_seconds_since_boot();
    TransactionsInProgress++;
    disable_twi_debug_printf--;
    return true;
}

// Mark a TWI transaction as being completed
bool twi_completed(twi_context_t *t) {

    // Don't allow recursion because of DEBUG_PRINTF
    disable_twi_debug_printf++;

    // Bug check
    if (TransactionsInProgress == 0) {
        TransactionsInProgress = 0;
    }
    // Bump the transactions
    --TransactionsInProgress;
    // Handle errors
    if (t->transaction_error != NRF_SUCCESS) {
        // If the comment was "~", suppress errors
        if (t->comment != NULL && t->comment[0] != '~') {
            CompletionErrors++;
            report_err();
            disable_twi_debug_printf--;
            return false;
        }
    }
    disable_twi_debug_printf--;
    return true;
}

// Initialization of TWI subsystem
bool twi_init() {
    uint32_t err_code;

    // Don't allow recursion because of DEBUG_PRINTF
    disable_twi_debug_printf++;

    // Exit if already initialized
    if (InitCount++ > 0) {

        if (debug(DBG_SENSOR_MAX))
            DEBUG_PRINTF("TWI Init nested, now %d users\n", InitCount);

        disable_twi_debug_printf--;
        return true;
    }

    if (debug(DBG_SENSOR_MAX))
        DEBUG_PRINTF("TWI Init\n");

    // Power it on
    gpio_power_set(POWER_PIN_TWI, true);

    // Delay to allow the device to power on.  This is ** REQUIRED ** for TWI devices to function.
    nrf_delay_ms(MAX_NRF_DELAY_MS);

    // Initialize TWI
    APP_TWI_INIT(&m_app_twi, &config, MAX_PENDING_TWI_TRANSACTIONS, err_code);
    if (err_code != NRF_SUCCESS) {
        InitCount--;
        DEBUG_PRINTF("TWI init error = 0x%04x\n", err_code);
        stats()->errors_twi++;
        gpio_power_set(POWER_PIN_TWI, false);
        disable_twi_debug_printf--;
        return false;
    }

    disable_twi_debug_printf--;
    return true;
}

// Determine whether or not the current user is the final user of TWI
bool twi_one_user() {
    return (InitCount == 1);
}

// Termination of TWI, which must be precisely paired with calls to twi_init()
bool twi_term() {

    // Don't allow recursion because of DEBUG_PRINTF
    disable_twi_debug_printf++;

    // Just defensive programming
    if (InitCount <= 0) {
        InitCount = 0;
        disable_twi_debug_printf--;
        return false;
    }

    if (--InitCount == 0) {

        app_twi_uninit(&m_app_twi);
        gpio_power_set(POWER_PIN_TWI, false);

        if (debug(DBG_SENSOR_MAX))
            DEBUG_PRINTF("TWI Term\n");

    } else {

        if (debug(DBG_SENSOR_MAX))
            DEBUG_PRINTF("TWI Term nested, %d remaining\n", InitCount);

    }

    disable_twi_debug_printf--;
    return true;

}

#endif // TWIX
