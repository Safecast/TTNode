// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

#ifndef TWI_H__
#define TWI_H__

#include "app_twi.h"
#include "ublox.h"
#include "hih.h"
#include "max01.h"
#include "max43.h"

// The user context passed on TWI transactions, one per transaction type.
// The restriction is that there can only be one pending transaction of any
// given type.
struct twi_context_s {
    // An index in the table of self
    uint16_t index;
    // Sensor context
    void *sensor;
    // TWI transaction context
    char *comment;
    // Number of transactions
    uint32_t transactions_scheduled;
    uint32_t transactions_completed;
    // Nonzero if transaction is in progress
    uint32_t transaction_began;
    // Last transaction scheduling error code
    ret_code_t sched_error;
    // Last transaction error code
    ret_code_t transaction_error;
    // User callback
    app_twi_callback_t callback;
};
typedef struct twi_context_s twi_context_t;

typedef void (*sensor_callback_t) (ret_code_t result, twi_context_t *t);

bool twi_init();
bool twi_one_user();
bool twi_term();
void twi_status_check(bool);
bool twi_schedule(void *sensor, sensor_callback_t callback, app_twi_transaction_t const * p_transaction);
void twi_callback(ret_code_t result, void *p_user_data);
bool twi_completed(twi_context_t *t);
bool twi_disable_twi_debug_printf();

bool twi_driver_init();
ret_code_t twi_driver_tx(uint8_t i2caddr, uint8_t const *p_data, uint8_t length, bool no_stop);
bool twi_driver_term();

#endif // TWI_H__
