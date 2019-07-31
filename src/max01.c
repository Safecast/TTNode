// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

// MAX17201 Fuel Gauge

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
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
#include "send.h"
#include "stats.h"
#include "comm.h"
#include "misc.h"
#include "twi.h"
#include "ina.h"
#include "io.h"
#include "stats.h"
#include "battery.h"

#define CONFIG_MODE_TRIGGERED   0
#define CONFIG_MODE_CONTINUOUS  1
#define CONFIG_MODE CONFIG_MODE_CONTINUOUS

#ifdef TWIMAX17201

// Specification (interesting stuff starts on p.82)
// https://datasheets.maximintegrated.com/en/ds/MAX17201-MAX17215.pdf

// I2C address
#define MAX17201_I2C_ADDRESS                     (0x36)

// Format of registers
#define addrLen  sizeof(uint8_t)
#define dataLen sizeof(uint8_t)*2

// MAX1720X offsets
static uint8_t regSTATUS[3] = { 0x00, 0, 0 };   // Contains alert status and chip status
static uint8_t regREPCAP[3] = { 0x05, 0, 0 };   // Reported remaining capacity
static uint8_t regREPSOC[3] = { 0x06, 0, 0 };   // Reported state of charge
static uint8_t regAGE[3] = { 0x07, 0, 0 };      // Percentage of capacity compared to design capacity
static uint8_t regTEMP[3] = { 0x08, 0, 0 };     // Temperature
static uint8_t regVCELL[3] = { 0x09, 0, 0 };    // Lowest cell voltage of pack, or the cell voltage for one cell
static uint8_t regCURRENT[3] = { 0x0A, 0, 0 };  // Battery current
static uint8_t regCAPACITY[3] = { 0x10, 0, 0 }; // Full capacity estimation
static uint8_t regTTE[3] = { 0x11, 0, 0 };      // Time to empty
static uint8_t regAVCELL[3] = { 0x17, 0, 0 };   // Battery cycles
static uint8_t regTTF[3] = { 0x20, 0, 0 };      // Time to full
static uint8_t regDEVNAME[3] = { 0x21, 0, 0 };  // Device type (2 bytes, mask 0x000f = 1:MAX17201, 5:MAX17205)
static uint8_t regAGE_FORECAST[3] = { 0xb9, 0, 0 }; // Projected cycles  until full is below target "dead" capaacity
static uint8_t regVBAT[3] = { 0xda, 0, 0 };     // Battery pack voltage

#if defined(UNUSED_FOR_NOW)
static uint8_t regCOMMAND[3] = { 0x60, 0, 0 };  // Command register
static uint8_t regCONFIG[3] = { 0x1d, 0, 0 };   // Config register
static uint8_t regCONFIG2[3] = { 0xbb, 0, 0 };  // Config2 register
#endif

static bool reported = false;
static bool ever_reported = false;
static float reported_voltage = 4.0;
static float reported_soc = 100.0;
static float reported_current = 0.0;
static float sampled_voltage;
static float sampled_soc;
static float sampled_current;

#define PWR_SAMPLE_BINS (PWR_SAMPLE_PERIOD_SECONDS/PWR_SAMPLE_SECONDS)
static bool first_sample;
static uint16_t num_samples;
static uint16_t num_current_samples;
static uint16_t num_polls;
static bool measure_on_next_poll = false;

// TWI initialization
static bool fTWIInit = false;

// Callback when TWI data has been read, or timeout
void max01_callback(ret_code_t result, twi_context_t *t) {
    int16_t icombined;
    uint16_t ucombined;

    // If error, flag that this I/O has been completed.
    if (!twi_completed(t)) {
        stats()->errors_max01++;
        battery_set_soc_to_unknown();
        sensor_measurement_completed(t->sensor);
        return;
    }

    // Compute Device name
    bool isMax01 = (regDEVNAME[1] & 0x0f) == 0x01;
    // Compute voltage by converting mV to V
    ucombined = regVCELL[1] | (regVCELL[2] << 8);
    float voltage = (float) ucombined * 0.078125 / 1000;
    // Compute current by converting to mV/Ohm (mA)
    icombined = regCURRENT[1] | (regCURRENT[2] << 8);
    float current = (float) icombined * (0.0015625/0.01);
#ifdef scv1
    current = -current;
#endif
    // Compute SOC as %
    ucombined = regREPSOC[1] | (regREPSOC[2] << 8);
    float soc = (float) ucombined / 256;
    // Compute Temp as degrees C
    icombined = regTEMP[1] | (regTEMP[2] << 8);
    float temp = (float) icombined / 256;
    // Compute Capacity as mAh
    ucombined = regREPCAP[1] | (regREPCAP[2] << 8);
    float cap = (float) ucombined * 0.005/0.01;
    // Compute Time To Empty in seconds
    ucombined = regTTE[1] | (regTTE[2] << 8);
    float tte = (float) ucombined * 5.625;
    // Compute Time To Full in seconds
    ucombined = regTTF[1] | (regTTF[2] << 8);
    float ttf = (float) ucombined * 5.625;
    // Extract misc things
    uint16_t status = regSTATUS[1] | (regSTATUS[2] << 8);
    uint16_t age = regAGE[1] | (regAGE[2] << 8);
    uint16_t capacity = regCAPACITY[1] | (regCAPACITY[2] << 8);
    uint16_t avcell = regAVCELL[1] | (regAVCELL[2] << 8);
    uint16_t agef = regAGE_FORECAST[1] | (regAGE_FORECAST[2] << 8);
    uint16_t vbat = regVBAT[1] | (regVBAT[2] << 8);

    // Set the string for stats
    char buffer[128];
    sprintf(buffer, "%.1f/%.1f/%.1f/%.1f/%.1f/%.1f/%.1f/%04x/%d/%d/%d/%d/%d",
            voltage, current, soc, temp, cap, tte, ttf, status, age, capacity, avcell, agef, vbat);
    strlcpy(stats()->battery, buffer, sizeof(stats()->battery)-1);

    // Store it into the bin IF AND ONLY IF nobody is currently sucking power on the UART if in oneshot mode
    if (num_samples < PWR_SAMPLE_BINS) {
        sampled_voltage += voltage;
        sampled_soc += soc;
        if (!comm_oneshot_currently_enabled() || gpio_current_uart() == UART_NONE) {
            sampled_current += current;
            num_current_samples++;
        }
        num_samples++;
        first_sample = false;
    }

    // If we're not done, request another measurement
    if (num_samples < PWR_SAMPLE_BINS) {

        // We should never hit this, but this is a protective measure to ensure that
        // we don't just sit here for too long and hold up other sensors from operating
        if (num_polls++ < (PWR_SAMPLE_BINS*25))
            measure_on_next_poll = true;
        else {
            DEBUG_PRINTF("MAX01 Aborting because of unreleased UART\n", gpio_uart_name(gpio_current_uart()));
            battery_set_soc_to_unknown();
            sensor_measurement_completed(t->sensor);
        }

        // Ensure that we've got a good device
        if (!isMax01) {
            battery_set_soc_to_unknown();
            stats()->errors_max01++;
            DEBUG_PRINTF("MAX01 not detected! (0x%04x)\n", (regDEVNAME[2] << 8) | regDEVNAME[1]);
        }

        // Debug

        if (debug(DBG_SENSOR_MAX))
            DEBUG_PRINTF("MAX01 %.3fmA %.3fV %.3f%%\n", current, voltage, soc);
        if (debug(DBG_SENSOR_SUPERMAX)) {
            DEBUG_PRINTF("temp:%.3fC cap:%.3fmAh tte:%.3fs ttf:%.3fs\n", temp, cap, tte, ttf);
            DEBUG_PRINTF("status:0x%04x age:%d C:%d avcell:%d agef:%d vbat:%d\n", status, age, capacity, avcell, agef, vbat);
        }

    } else {

        // Compute the reported measurements
        reported_voltage = sampled_voltage / num_samples;
        reported_soc = sampled_soc / num_samples;
        if (num_current_samples == 0)
            reported_current = 0;
        else
            reported_current = sampled_current / num_current_samples;

        // When debugging current, just poll continuously
#ifdef CURRENTDEBUG

        // Start over
        s_max01_clear_measurement();
        measure_on_next_poll = true;

#else
        // Done
        reported = ever_reported = true;

        // Tell the sensor package that we retrieved an SOC value, and what it is
#if 0
        battery_set_soc(reported_soc);
#else
        battery_set_soc(battery_soc_from_voltage(reported_voltage));
#endif

        // Flag that this I/O has been completed.
        sensor_measurement_completed(t->sensor);

#endif  // !CURRENTDEBUG

        if (debug(DBG_SENSOR))
            DEBUG_PRINTF("BAT %.2fV %.0f%% %.0fmA\n", reported_voltage, reported_soc, reported_current);

    }

}

// Measurement needed?  Say "no" just so as to ensure continuous re-measuring even if upload pending
// Note that it WILL get uploaded opportunistically whenever something else needs uploading.
bool s_max01_upload_needed(void *s) {
    return false;
}

// Poller continuously re-measures when requested to do so
void s_max01_poll(void *s) {
    if (!sensor_is_polling_valid(s))
        return;
    if (measure_on_next_poll) {
        measure_on_next_poll = false;
        s_max01_measure(s);
    }
}

// Measure voltage
void s_max01_measure(void *s) {
    static app_twi_transfer_t const transfers[] = {
        APP_TWI_WRITE(MAX17201_I2C_ADDRESS, &regSTATUS[0], addrLen, APP_TWI_NO_STOP),
        APP_TWI_READ(MAX17201_I2C_ADDRESS,  &regSTATUS[1], dataLen, 0),
        APP_TWI_WRITE(MAX17201_I2C_ADDRESS, &regREPCAP[0], addrLen, APP_TWI_NO_STOP),
        APP_TWI_READ(MAX17201_I2C_ADDRESS,  &regREPCAP[1], dataLen, 0),
        APP_TWI_WRITE(MAX17201_I2C_ADDRESS, &regREPSOC[0], addrLen, APP_TWI_NO_STOP),
        APP_TWI_READ(MAX17201_I2C_ADDRESS,  &regREPSOC[1], dataLen, 0),
        APP_TWI_WRITE(MAX17201_I2C_ADDRESS, &regAGE[0], addrLen, APP_TWI_NO_STOP),
        APP_TWI_READ(MAX17201_I2C_ADDRESS,  &regAGE[1], dataLen, 0),
        APP_TWI_WRITE(MAX17201_I2C_ADDRESS, &regTEMP[0], addrLen, APP_TWI_NO_STOP),
        APP_TWI_READ(MAX17201_I2C_ADDRESS,  &regTEMP[1], dataLen, 0),
        APP_TWI_WRITE(MAX17201_I2C_ADDRESS, &regVCELL[0], addrLen, APP_TWI_NO_STOP),
        APP_TWI_READ(MAX17201_I2C_ADDRESS,  &regVCELL[1], dataLen, 0),
        APP_TWI_WRITE(MAX17201_I2C_ADDRESS, &regCURRENT[0], addrLen, APP_TWI_NO_STOP),
        APP_TWI_READ(MAX17201_I2C_ADDRESS,  &regCURRENT[1], dataLen, 0),
        APP_TWI_WRITE(MAX17201_I2C_ADDRESS, &regCAPACITY[0], addrLen, APP_TWI_NO_STOP),
        APP_TWI_READ(MAX17201_I2C_ADDRESS,  &regCAPACITY[1], dataLen, 0),
        APP_TWI_WRITE(MAX17201_I2C_ADDRESS, &regTTE[0], addrLen, APP_TWI_NO_STOP),
        APP_TWI_READ(MAX17201_I2C_ADDRESS,  &regTTE[1], dataLen, 0),
        APP_TWI_WRITE(MAX17201_I2C_ADDRESS, &regAVCELL[0], addrLen, APP_TWI_NO_STOP),
        APP_TWI_READ(MAX17201_I2C_ADDRESS,  &regAVCELL[1], dataLen, 0),
        APP_TWI_WRITE(MAX17201_I2C_ADDRESS, &regTTF[0], addrLen, APP_TWI_NO_STOP),
        APP_TWI_READ(MAX17201_I2C_ADDRESS,  &regTTF[1], dataLen, 0),
        APP_TWI_WRITE(MAX17201_I2C_ADDRESS, &regDEVNAME[0], addrLen, APP_TWI_NO_STOP),
        APP_TWI_READ(MAX17201_I2C_ADDRESS,  &regDEVNAME[1], dataLen, 0),
        APP_TWI_WRITE(MAX17201_I2C_ADDRESS, &regAGE_FORECAST[0], addrLen, APP_TWI_NO_STOP),
        APP_TWI_READ(MAX17201_I2C_ADDRESS,  &regAGE_FORECAST[1], dataLen, 0),
        APP_TWI_WRITE(MAX17201_I2C_ADDRESS, &regVBAT[0], addrLen, APP_TWI_NO_STOP),
        APP_TWI_READ(MAX17201_I2C_ADDRESS,  &regVBAT[1], dataLen, 0),
    };
    static app_twi_transaction_t const transaction = {
        .callback            = twi_callback,
        .p_user_data         = "MAX01",
        .p_transfers         = transfers,
        .number_of_transfers = sizeof(transfers) / sizeof(transfers[0])
    };
    if (!twi_schedule(s, max01_callback, &transaction)) {
        stats()->errors_max01++;
        battery_set_soc_to_unknown();
        sensor_unconfigure(s);
    }
}

// Show the current value
bool s_max01_show_value(uint32_t when, char *buffer, uint16_t length) {
    static uint32_t last = 0;
    char msg[128];
    if (when == last)
        return false;
    last = when;
    if (ever_reported)
        sprintf(msg, "BAT %.2fV %.0f%% %.0fmA", reported_voltage, reported_soc, reported_current);
    else
        sprintf(msg, "BAT not yet measured");
    strlcpy(buffer, msg, length);
    return true;
}

// The main access method for our data
bool s_max01_get_value(float *pLoadVoltage, float *pSOC, float *pCurrent) {
    if (pCurrent != NULL)
        *pCurrent = reported_current;
    if (pLoadVoltage != NULL)
        *pLoadVoltage = reported_voltage;
    if (pSOC != NULL)
        *pSOC = reported_soc;
    return (reported);
}

// Clear it out
void s_max01_clear_measurement() {
    reported = false;
    num_polls = 0;
    measure_on_next_poll = false;
    num_samples = 0;
    num_current_samples = 0;
    first_sample = true;
    sampled_soc = sampled_voltage = sampled_current = 0.0;
}

// Init sensor
bool s_max01_init(void *s, uint16_t param) {

    // Init TWI
    if (!twi_init()) {
        battery_set_soc_to_unknown();
        stats()->errors_max01++;
        return false;
    }
    fTWIInit = true;

    // Clear the measurement
    s_max01_clear_measurement();

    // Success
    return true;
}

// Term sensor
bool s_max01_term() {
    if (fTWIInit) {
        twi_term();
        fTWIInit = false;
    }
    return true;
}

#endif // TWIMAX17201
