// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

// MAX17043 Fuel Gauge

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
#include "max43.h"

#ifdef TWIMAX17043

// MAX17043 Data Sheet
// https://datasheets.maximintegrated.com/en/ds/MAX17043-MAX17044.pdf

// MAX17043 7-Bit I2C Address
#define MAX17043_I2C_ADDRESS    0x36
#define MAX17043_ADDRESS_LEN    1
#define MAX17043_DATA_LEN       2

// MAX17043 Register Definitions - all contain 2 bytes of data and span 2 addresses
// R - 12-bit A/D measurement of battery voltage
#define MAX17043_VCELL    0x02
// R - 16-bit state of charge (SOC)
#define MAX17043_SOC      0x04
// W - Sends special commands to IC
#define MAX17043_MODE     0x06

// I/O buffer
typedef struct {
    // Common to all sensors
    void *sensor;
    // Address for TWI transfer
    uint8_t address[MAX17043_ADDRESS_LEN];
    // Buffer used for transfers (we only use a few bytes of it)
    uint8_t buffer[MAX17043_DATA_LEN];
    // State
    bool batteryReportedVoltage;
    float batteryVoltage;
    bool batteryReportedSOC;
    float batterySOC;
} max17043_data_t;
static max17043_data_t ioVoltage = {0};
static max17043_data_t ioSOC = {0};

// TWI initialization
static bool fTWIInitV = false;
static bool fTWIInitS = false;

// Callback when TWI data has been read, or timeout
void voltage_callback(ret_code_t result, void *io) {
    float voltage;
    uint16_t vCell;

    if (!twi_completed("MAX43-V", result)) {
        sensor_measurement_completed(ioVoltage.sensor);
        return;
    }

    // Extract battery voltage, noting that MSB comes before LSB on TWI
    vCell = (ioVoltage.buffer[0] << 8) | ioVoltage.buffer[1];
    vCell = (vCell) >> 4;
    voltage = (float) vCell / 800.0;
    ioVoltage.batteryVoltage = voltage;
    ioVoltage.batteryReportedVoltage = true;

    // Flag that this I/O has been completed.
    sensor_measurement_completed(ioVoltage.sensor);
}

// Measurement needed?  Say "no" just so as not to trigger an upload just because of this
bool s_max43_voltage_upload_needed(void *s) {
    return(false);
}

// Measure voltage
void s_max43_voltage_measure(void *s) {
    ioVoltage.sensor = s;
    ioVoltage.address[0] = MAX17043_VCELL;
    static app_twi_transfer_t const transfers[] = {
        APP_TWI_WRITE(MAX17043_I2C_ADDRESS, &ioVoltage.address[0], MAX17043_ADDRESS_LEN, APP_TWI_NO_STOP),
        APP_TWI_READ(MAX17043_I2C_ADDRESS, &ioVoltage.buffer[0], MAX17043_DATA_LEN, 0)
    };
    static app_twi_transaction_t const transaction = {
        .callback            = voltage_callback,
        .p_user_data         = NULL,
        .p_transfers         = transfers,
        .number_of_transfers = sizeof(transfers) / sizeof(transfers[0])
    };
    if (!twi_schedule("MAX43-V", &transaction))
        sensor_unconfigure(s);
}

// The main access method for our data
bool s_max43_voltage_get_value(float *voltage) {
    if (voltage != NULL)
        *voltage = ioVoltage.batteryVoltage;
    if (!ioVoltage.batteryReportedVoltage)
        return false;
    return true;
}

// Clear it out
void s_max43_voltage_clear_measurement() {
    ioVoltage.batteryReportedVoltage = false;
}

// Init sensor
bool s_max43_voltage_init(uint16_t param) {
    if (!twi_init())
        return false;
    fTWIInitV = true;
    s_max43_voltage_clear_measurement();
    return true;
}

// Term sensor
bool s_max43_voltage_term() {
    if (fTWIInitV) {
        twi_term();
        fTWIInitV = false;
    }
    return true;
}

// Callback when TWI data has been read
void soc_callback(ret_code_t result, void *io) {
    float percent;
    uint16_t soc;

    if (!twi_completed("MAX43-S", result)) {
        sensor_measurement_completed(ioSOC.sensor);
        return;
    }

    // Extract battery SOC, noting that MSB comes before LSB on TWI
    soc = (ioSOC.buffer[0] << 8) | ioSOC.buffer[1];
    percent = (soc & 0xFF00) >> 8;
    percent += (float) (((uint8_t) soc) / 256.0);
    ioSOC.batterySOC = percent;
    ioSOC.batteryReportedSOC = true;
    sensor_set_bat_soc(ioSOC.batterySOC);

    // Flag that this I/O has been completed.
    sensor_measurement_completed(ioSOC.sensor);
}

// Measurement needed?  Say "no" just so as not to trigger an upload just because of this
bool s_max43_soc_upload_needed(void *s) {
    return false;
}

// Measure SOC
void s_max43_soc_measure(void *s) {
    ioSOC.sensor = s;
    ioSOC.address[0] = MAX17043_SOC;
    static app_twi_transfer_t const transfers[] = {
        APP_TWI_WRITE(MAX17043_I2C_ADDRESS, &ioSOC.address[0], MAX17043_ADDRESS_LEN, APP_TWI_NO_STOP),
        APP_TWI_READ(MAX17043_I2C_ADDRESS, &ioSOC.buffer[0], MAX17043_DATA_LEN, 0)
    };
    static app_twi_transaction_t const transaction = {
        .callback            = soc_callback,
        .p_user_data         = NULL,
        .p_transfers         = transfers,
        .number_of_transfers = sizeof(transfers) / sizeof(transfers[0])
    };
    if (!twi_schedule("MAX43-S", &transaction))
        sensor_unconfigure(s);
}

// The main access method for our data
bool s_max43_soc_get_value(float *soc) {
    if (soc != NULL)
        *soc = ioSOC.batterySOC;
    if (!ioSOC.batteryReportedSOC)
        return false;
    return true;
}

// Clear it out
void s_max43_soc_clear_measurement() {
    ioSOC.batteryReportedSOC = false;
}

// Init sensor
bool s_max43_soc_init(uint16_t param) {
    if (!twi_init())
        return false;
    fTWIInitS = true;
    s_max43_soc_clear_measurement();
    return true;
}

// Term sensor
bool s_max43_soc_term() {
    if (fTWIInitS) {
        twi_term();
        fTWIInitS = false;
    }
    return true;
}

#endif // TWIMAX17043
