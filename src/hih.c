// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

// HIH6130 Temp/Mumidity Sensor

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
#include "app_twi.h"
#include "gpio.h"
#include "config.h"
#include "timer.h"
#include "sensor.h"
#include "storage.h"
#include "comm.h"
#include "misc.h"
#include "twi.h"
#include "io.h"

#ifdef TWIHIH6130

// HIH6130 Data Sheet
// http://www.phanderson.com/arduino/I2CCommunications.pdf

// HIH6130 7-Bit I2C Address
#define HIH6130_ADDRESS     0x27
#define HIH6130_DATA_LEN    4

// I/O buffer
typedef struct {
    // I/O buffer
    uint8_t buffer[HIH6130_DATA_LEN];
    // Current state
    bool envReported;
    float envTempC;
    float envHumRH;
} hih6130_data_t;
static hih6130_data_t ioTemp;

// TWI initialization
static bool fTWIInit = false;

// Callback when TWI data has been read
void temp_callback(ret_code_t result, twi_context_t *t) {
    uint8_t Hum_H, Hum_L, Temp_H, Temp_L, status;
    uint16_t Hum_X, Temp_X;
    float HumRH, TempC;

    if (!twi_completed(t)) {
        sensor_measurement_completed(t->sensor);
        return;
    }

    // Read the temperature
    status = (ioTemp.buffer[0] >> 6) & 0x03;
    UNUSED_VARIABLE(status);
    Hum_H = ioTemp.buffer[0] & 0x3f;
    Hum_L = ioTemp.buffer[1];
    Temp_H = ioTemp.buffer[2];
    Temp_L = ioTemp.buffer[3];
    Hum_X = (((uint16_t )Hum_H) << 8) | Hum_L;
    Temp_X = ((((uint16_t)Temp_H) << 8) | Temp_L) >> 2;
    if (Hum_X != 0 && Temp_X != 0) {
        HumRH = (float) Hum_X * 6.10e-3;
        TempC = (float) Temp_X * 1.007e-2 - 40.0;
        // Update the temp at the cmd level
        ioTemp.envTempC = TempC;
        ioTemp.envHumRH = HumRH;
        ioTemp.envReported = true;
    }

    // Mark the I/O as being completed
    sensor_measurement_completed(t->sensor);
}

// Measurement needed?  Say "no" just so as not to trigger an upload just because of this
bool s_hih6130_upload_needed(void *s) {
    return false;
}

// Measure temp
void s_hih6130_measure(void *s) {
    memset(ioTemp.buffer, 0, sizeof(ioTemp.buffer));
    static app_twi_transfer_t const transfers[] = {
        APP_TWI_READ(HIH6130_ADDRESS, &ioTemp.buffer[0], HIH6130_DATA_LEN, 0)
    };
    static app_twi_transaction_t const transaction = {
        .callback            = twi_callback,
        .p_user_data         = "HIH",
        .p_transfers         = transfers,
        .number_of_transfers = sizeof(transfers) / sizeof(transfers[0])
    };
    if (!twi_schedule(s, temp_callback, &transaction))
        sensor_unconfigure(s);
}

// The main access method for our data
bool s_hih6130_get_value(float *temp, float *humid) {
    if (temp != NULL)
        *temp = ioTemp.envTempC;
    if (humid != NULL)
        *humid = ioTemp.envHumRH;
    if (!ioTemp.envReported)
        return false;
    return true;
}

// Clear it out
void s_hih6130_clear_measurement() {
    ioTemp.envReported = false;
}

// Init sensor
bool s_hih6130_init(void *s, uint16_t param) {
    if (!twi_init())
        return false;
    fTWIInit = true;
    ioTemp.envReported = false;
    return true;
}

// Term sensor
bool s_hih6130_term() {
    if (fTWIInit) {
        twi_term();
        fTWIInit = false;
    }
    return true;
}

#endif // TWIHIH6130
