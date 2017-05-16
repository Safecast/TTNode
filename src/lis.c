// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

// LIS3DH MEMS motion sensor

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
#include "comm.h"
#include "misc.h"
#include "twi.h"
#include "lis.h"
#include "io.h"
#include "stats.h"

#ifdef TWILIS3DH

#define LIS3DH_I2C_ADDRESS0      (0x18)     // if SDO/SA0 is GND it's 0x18
#define LIS3DH_I2C_ADDRESS1      (0x19)     // if SDO/SA0 is 3V3 it's 0x19

#define LIS_I2C_ADDRESS LIS3DH_I2C_ADDRESS1

// Source Materials
// http://www.st.com/en/mems-and-sensors/lis3dh.html
// http://www.st.com/resource/en/application_note/cd00290365.pdf
// https://learn.adafruit.com/adafruit-lis3dh-triple-axis-accelerometer-breakout/overview

#define WHO_AM_I 0b00110011

#define REG_STATUS_AUX 0x07
#define REG_OUT_ADC1_L 0x08
#define REG_OUT_ADC1_H 0x09
#define REG_OUT_ADC2_L 0x0a
#define REG_OUT_ADC2_H 0x0b
#define REG_OUT_ADC3_L 0x0c
#define REG_OUT_ADC3_H 0x0d
#define REG_INT_COUNTER_REG 0x0e
#define REG_WHO_AM_I 0x0f
#define REG_TEMP_CFG_REG 0x1f
#define REG_CTRL_REG1 0x20
#define REG_CTRL_REG2 0x21
#define REG_CTRL_REG3 0x22
#define REG_CTRL_REG4 0x23
#define REG_CTRL_REG5 0x24
#define REG_CTRL_REG6 0x25
#define REG_REFERENCE 0x26
#define REG_STATUS_REG 0x27
#define REG_OUT_X_L 0x28
#define REG_OUT_X_H 0x29
#define REG_OUT_Y_L 0x2a
#define REG_OUT_Y_H 0x2b
#define REG_OUT_Z_L 0x2c
#define REG_OUT_Z_H 0x2d
#define REG_FIFO_CTRL_REG 0x2e
#define REG_FIFO_SRC_REG 0x2f
#define REG_INT1_CFG 0x30
#define REG_INT1_SRC 0x31
#define REG_INT1_THS 0x32
#define REG_INT1_DURATION 0x33
#define REG_CLICK_CFG 0x38
#define REG_CLICK_SRC 0x39
#define REG_CLICK_THS 0x3a
#define REG_TIME_LIMIT 0x3b
#define REG_TIME_LATENCY 0x3c
#define REG_TIME_WINDOW 0x3d

#define STATUS_AUX_321OR 0x80
#define STATUS_AUX_3OR 0x40
#define STATUS_AUX_2OR 0x20
#define STATUS_AUX_1OR 0x10
#define STATUS_AUX_321DA 0x08
#define STATUS_AUX_3DA 0x04
#define STATUS_AUX_2DA 0x02
#define STATUS_AUX_1DA 0x01

#define CTRL_REG1_ODR3 0x80
#define CTRL_REG1_ODR2 0x40
#define CTRL_REG1_ODR1 0x20
#define CTRL_REG1_ODR0 0x10
#define CTRL_REG1_LPEN 0x08
#define CTRL_REG1_ZEN 0x04
#define CTRL_REG1_YEN 0x02
#define CTRL_REG1_XEN 0x01

#define RATE_1_HZ   0x10
#define RATE_10_HZ  0x20
#define RATE_25_HZ  0x30
#define RATE_50_HZ  0x40
#define RATE_100_HZ 0x50
#define RATE_200_HZ 0x60
#define RATE_400_HZ 0x70

#define CTRL_REG2_HPM1 0x80
#define CTRL_REG2_HPM0 0x40
#define CTRL_REG2_HPCF2 0x20
#define CTRL_REG2_HPCF1 0x10
#define CTRL_REG2_FDS 0x08
#define CTRL_REG2_HPCLICK 0x04
#define CTRL_REG2_HPIS2 0x02
#define CTRL_REG2_HPIS1 0x01

#define CTRL_REG3_I1_CLICK 0x80
#define CTRL_REG3_I1_AOI1 0x40
#define CTRL_REG3_I1_AOI2 0x20
#define CTRL_REG3_I1_DRDY1 0x10
#define CTRL_REG3_I1_DRDY2 0x08
#define CTRL_REG3_I1_WTM 0x04
#define CTRL_REG3_I1_OVERRUN 0x02

#define CTRL_REG4_BDU 0x80
#define CTRL_REG4_BLE 0x40
#define CTRL_REG4_FS1 0x20
#define CTRL_REG4_FS0 0x10
#define CTRL_REG4_HR 0x08
#define CTRL_REG4_ST1 0x04
#define CTRL_REG4_ST0 0x02
#define CTRL_REG4_SIM 0x01

#define CTRL_REG5_BOOT 0x80
#define CTRL_REG5_FIFO_EN 0x40
#define CTRL_REG5_LIR_INT1 0x08
#define CTRL_REG5_D4D_INT1 0x04

#define CTRL_REG6_I2_CLICK 0x80
#define CTRL_REG6_I2_INT2 0x40
#define CTRL_REG6_BOOT_I2 0x10
#define CTRL_REG6_H_LACTIVE 0x02

#define INT1_CFG_AOI 0x80
#define INT1_CFG_6D 0x40
#define INT1_CFG_ZHIE_ZUPE 0x20
#define INT1_CFG_ZLIE_ZDOWNE 0x10
#define INT1_CFG_YHIE_YUPE 0x08
#define INT1_CFG_YLIE_YDOWNE 0x04
#define INT1_CFG_XHIE_XUPE 0x02
#define INT1_CFG_XLIE_XDOWNE 0x01

#define INT1_SRC_IA 0x40
#define INT1_SRC_ZH 0x20
#define INT1_SRC_ZL 0x10
#define INT1_SRC_YH 0x08
#define INT1_SRC_YL 0x04
#define INT1_SRC_XH 0x02
#define INT1_SRC_XL 0x01

#define TEMP_CFG_ADC_PD 0x80
#define TEMP_CFG_TEMP_EN 0x40

#define FIFO_CTRL_BYPASS 0x00
#define FIFO_CTRL_FIFO 0x40
#define FIFO_CTRL_STREAM 0x80
#define FIFO_CTRL_STREAM_TO_FIFO 0xc0

#define FIFO_SRC_WTM 0x80
#define FIFO_SRC_OVRN 0x40
#define FIFO_SRC_EMPTY 0x20
#define FIFO_SRC_FSS_MASK 0x1f

#define  RECALIBRATION_MOVEMENT_DELAY 100

// Statics
static bool fInit = false;
static bool fTWIInitialized = false;
static bool fArmForMotionSensing = false;
static bool fPollRequestInProgress = false;

// These are so that our poller can truly validate that the interrupt is cleared
static bool fPollerShouldClearInterrupt = false;;
#define MAX_CONSECUTIVE_CLEAR 10
static uint16_t ConsecutiveClearInterrupts;

// TWI I/O data
static uint8_t who[2] = {REG_WHO_AM_I, 0};
static uint8_t reg1[2] = {REG_CTRL_REG1, 0};
static uint8_t reg2[2] = {REG_CTRL_REG2, 0};
static uint8_t reg3[2] = {REG_CTRL_REG3, 0};
static uint8_t reg4[2] = {REG_CTRL_REG4, 0};
static uint8_t reg5[2] = {REG_CTRL_REG5, 0};
static uint8_t reg6[2] = {REG_CTRL_REG6, 0};
static uint8_t int1_ths[2] = {REG_INT1_THS, 0};
static uint8_t int1_cfg[2] = {REG_INT1_CFG, 0};
static uint8_t int1_src[2] = {REG_INT1_SRC, 0};

// Callback when TWI data has been read, or timeout
void lis_callback(ret_code_t result, twi_context_t *t) {
    
    // If error, flag that this I/O has been completed.
    if (!twi_completed(t)) {
        fInit = false;  // Force reinitialization
        stats()->errors_lis++;
        gpio_motion_sense(MOTION_DISARM);
        sensor_measurement_completed(t->sensor);
        return;
    }

    // If we don't respond to the "who am I" command, something is wrong
    if (who[1] != WHO_AM_I) {
        fInit = false;  // Force reinitialization
        stats()->errors_lis++;
        DEBUG_PRINTF("LIS: Bad response %02x from WHO AM I!\n", who[1]);
        gpio_motion_sense(MOTION_DISARM);
        sensor_measurement_completed(t->sensor);
        return;
    }

    // It's ok for the poller to validate that the interrupt is cleared
    fPollerShouldClearInterrupt = true;
    ConsecutiveClearInterrupts = 0;
    
}

// Measure the sensor value
void s_lis_measure(void *s) {

    // Exit immediately if we don't need to do any measurement
    if (!fArmForMotionSensing) {
        if (debug(DBG_SENSOR))
            DEBUG_PRINTF("LIS: already armed\n");
        sensor_measurement_completed(s);
        return;
    }

    // clear out registers
    reg1[1] = reg2[1] = reg3[1] = reg4[1] = reg5[1] = reg6[1] = int1_ths[1] = int1_cfg[1] = 0;

    // Set low-power wake mode
    // Enable all axes
    reg1[1] |= CTRL_REG1_ZEN | CTRL_REG1_YEN | CTRL_REG1_XEN;
    // Enable 10 hz update rate
    reg1[1] |= CTRL_REG1_ODR1;
    // Enable low power mode
    reg1[1] |= CTRL_REG1_LPEN;
    // Enable high-pass filter on interrupt 1
    reg2[1] |= CTRL_REG2_FDS | CTRL_REG2_HPIS1;
    // Enable INT1
    reg3[1] |= CTRL_REG3_I1_AOI1;
    // Disable FIFO, enable latch interrupt on INT1_SRC
    reg5[1] |= CTRL_REG5_LIR_INT1;
    // Set Movement Threshold to 250mg
    int1_ths[1] = 16;
    // Configure the things that will interrupt us
    int1_cfg[1] |= INT1_CFG_ZHIE_ZUPE | INT1_CFG_YHIE_YUPE | INT1_CFG_XHIE_XUPE;

    static app_twi_transfer_t const transfers[] = {
        APP_TWI_WRITE(LIS_I2C_ADDRESS, &who[0], sizeof(who[0]), APP_TWI_NO_STOP),
        APP_TWI_READ(LIS_I2C_ADDRESS, &who[1], sizeof(who[1]), 0),
        APP_TWI_WRITE(LIS_I2C_ADDRESS, &reg1, sizeof(reg1), 0),
        APP_TWI_WRITE(LIS_I2C_ADDRESS, &reg2, sizeof(reg2), 0),
        APP_TWI_WRITE(LIS_I2C_ADDRESS, &reg3, sizeof(reg3), 0),
        APP_TWI_WRITE(LIS_I2C_ADDRESS, &reg4, sizeof(reg4), 0),
        APP_TWI_WRITE(LIS_I2C_ADDRESS, &reg5, sizeof(reg5), 0),
        APP_TWI_WRITE(LIS_I2C_ADDRESS, &reg6, sizeof(reg6), 0),
        APP_TWI_WRITE(LIS_I2C_ADDRESS, &int1_ths, sizeof(int1_ths), 0),
        APP_TWI_WRITE(LIS_I2C_ADDRESS, &int1_cfg, sizeof(int1_cfg), 0),
        APP_TWI_WRITE(LIS_I2C_ADDRESS, &int1_src[0], sizeof(int1_src[0]), APP_TWI_NO_STOP),
        APP_TWI_READ(LIS_I2C_ADDRESS, &int1_src[1], sizeof(int1_src[1]), 0),
    };
    static app_twi_transaction_t const transaction = {
        .callback            = twi_callback,
        .p_user_data         = "LIS",
        .p_transfers         = transfers,
        .number_of_transfers = sizeof(transfers) / sizeof(transfers[0])
    };
    if (!twi_schedule(s, lis_callback, &transaction)) {
        stats()->errors_lis++;
        sensor_unconfigure(s);
    }
    
}

// Dump output, for debugging

static uint8_t status_aux[2] = {REG_STATUS_AUX, 0};
static uint8_t out_adc1_l[2] = {REG_OUT_ADC1_L, 0};
static uint8_t out_adc1_h[2] = {REG_OUT_ADC1_H, 0};
static uint8_t out_adc2_l[2] = {REG_OUT_ADC2_L, 0};
static uint8_t out_adc2_h[2] = {REG_OUT_ADC2_H, 0};
static uint8_t out_adc3_l[2] = {REG_OUT_ADC3_L, 0};
static uint8_t out_adc3_h[2] = {REG_OUT_ADC3_H, 0};
static uint8_t out_x_l[2] = {REG_OUT_X_L, 0};
static uint8_t out_x_h[2] = {REG_OUT_X_H, 0};
static uint8_t out_y_l[2] = {REG_OUT_Y_L, 0};
static uint8_t out_y_h[2] = {REG_OUT_Y_H, 0};
static uint8_t out_z_l[2] = {REG_OUT_Z_L, 0};
static uint8_t out_z_h[2] = {REG_OUT_Z_H, 0};

void lis_poll_callback(ret_code_t result, twi_context_t *t) {

    // The poll request is no longer in progress
    fPollRequestInProgress = false;

    if (!twi_completed(t)) {
        fInit = false;  // Force reinitialization
        stats()->errors_lis++;
        sensor_measurement_completed(t->sensor);
        return;
    }
           
    if (debug(DBG_SENSOR_SUPERMAX))
        DEBUG_PRINTF("W%02x M%d A%02x C%02x T%02x | %02x%02x%02x%02x%02x%02x | %02x%02x %02x%02x %02x%02x | %02x%02x %02x%02x %02x%02x\n",
                 who[1], gpio_motion_sense(MOTION_QUERY_PIN), status_aux[1], int1_cfg[1], int1_ths[1],
                 reg1[1], reg2[1], reg3[1], reg4[1], reg5[1], reg6[1],
                 out_adc1_h[1], out_adc1_l[1], out_adc2_h[1], out_adc2_l[1], out_adc3_h[1], out_adc3_l[1],
                 out_x_h[1], out_x_l[1], out_y_h[1], out_y_l[1], out_z_h[1], out_z_l[1]);

    // Check the INT pin, and validate that it is clear for 10 consecutive polls
    if (gpio_motion_sense(MOTION_QUERY_PIN)) {
        ConsecutiveClearInterrupts = 0;
        return;
    }

    if (ConsecutiveClearInterrupts++ < MAX_CONSECUTIVE_CLEAR)
        return;

    fPollerShouldClearInterrupt = false;

    // We've now been clear for 10 consecutive polls.  We're done.
    if (debug(DBG_SENSOR))
        DEBUG_PRINTF("MOTION re-armed\n");

    gpio_motion_sense(MOTION_ARM);

    // Flag that this I/O has been completed
#ifdef MOTIONDEBUGMAX
    // We're debugging the sensor, so don't mark it as complete so that we stay measuring forever
#else
    sensor_measurement_completed(t->sensor);
#endif

}

void s_lis_poll(void *s) {

    // Exit if we're not supposed to be here
    if (!sensor_is_polling_valid(s))
        return;

    // Exit if not initialized or TWI error
    if (!fInit)
        return;

    // Exit if we're not supposed to clear the interrupt yet
    if (!fPollerShouldClearInterrupt)
        return;

    // Exit if a poll request is still in progress
    if (fPollRequestInProgress)
        return;

    static app_twi_transfer_t const transfers[] = {
        APP_TWI_WRITE(LIS_I2C_ADDRESS, &status_aux[0], sizeof(status_aux[0]), APP_TWI_NO_STOP),
        APP_TWI_READ(LIS_I2C_ADDRESS, &status_aux[1], sizeof(status_aux[1]), 0),
        APP_TWI_WRITE(LIS_I2C_ADDRESS, &out_adc1_l[0], sizeof(out_adc1_l[0]), APP_TWI_NO_STOP),
        APP_TWI_READ(LIS_I2C_ADDRESS, &out_adc1_l[1], sizeof(out_adc1_l[1]), 0),
        APP_TWI_WRITE(LIS_I2C_ADDRESS, &out_adc1_h[0], sizeof(out_adc1_h[0]), APP_TWI_NO_STOP),
        APP_TWI_READ(LIS_I2C_ADDRESS, &out_adc1_h[1], sizeof(out_adc1_h[1]), 0),
        APP_TWI_WRITE(LIS_I2C_ADDRESS, &out_adc2_l[0], sizeof(out_adc2_l[0]), APP_TWI_NO_STOP),
        APP_TWI_READ(LIS_I2C_ADDRESS, &out_adc2_l[1], sizeof(out_adc2_l[1]), 0),
        APP_TWI_WRITE(LIS_I2C_ADDRESS, &out_adc2_h[0], sizeof(out_adc2_h[0]), APP_TWI_NO_STOP),
        APP_TWI_READ(LIS_I2C_ADDRESS, &out_adc2_h[1], sizeof(out_adc2_h[1]), 0),
        APP_TWI_WRITE(LIS_I2C_ADDRESS, &out_adc3_l[0], sizeof(out_adc3_l[0]), APP_TWI_NO_STOP),
        APP_TWI_READ(LIS_I2C_ADDRESS, &out_adc3_l[1], sizeof(out_adc3_l[1]), 0),
        APP_TWI_WRITE(LIS_I2C_ADDRESS, &out_adc3_h[0], sizeof(out_adc3_h[0]), APP_TWI_NO_STOP),
        APP_TWI_READ(LIS_I2C_ADDRESS, &out_adc3_h[1], sizeof(out_adc3_h[1]), 0),
        APP_TWI_WRITE(LIS_I2C_ADDRESS, &out_x_l[0], sizeof(out_x_l[0]), APP_TWI_NO_STOP),
        APP_TWI_READ(LIS_I2C_ADDRESS, &out_x_l[1], sizeof(out_x_l[1]), 0),
        APP_TWI_WRITE(LIS_I2C_ADDRESS, &out_x_h[0], sizeof(out_x_h[0]), APP_TWI_NO_STOP),
        APP_TWI_READ(LIS_I2C_ADDRESS, &out_x_h[1], sizeof(out_x_h[1]), 0),
        APP_TWI_WRITE(LIS_I2C_ADDRESS, &out_y_l[0], sizeof(out_y_l[0]), APP_TWI_NO_STOP),
        APP_TWI_READ(LIS_I2C_ADDRESS, &out_y_l[1], sizeof(out_y_l[1]), 0),
        APP_TWI_WRITE(LIS_I2C_ADDRESS, &out_y_h[0], sizeof(out_y_h[0]), APP_TWI_NO_STOP),
        APP_TWI_READ(LIS_I2C_ADDRESS, &out_y_h[1], sizeof(out_y_h[1]), 0),
        APP_TWI_WRITE(LIS_I2C_ADDRESS, &out_z_l[0], sizeof(out_z_l[0]), APP_TWI_NO_STOP),
        APP_TWI_READ(LIS_I2C_ADDRESS, &out_z_l[1], sizeof(out_z_l[1]), 0),
        APP_TWI_WRITE(LIS_I2C_ADDRESS, &out_z_h[0], sizeof(out_z_h[0]), APP_TWI_NO_STOP),
        APP_TWI_READ(LIS_I2C_ADDRESS, &out_z_h[1], sizeof(out_z_h[1]), 0),
        APP_TWI_WRITE(LIS_I2C_ADDRESS, &int1_cfg[0], sizeof(int1_cfg[0]), APP_TWI_NO_STOP),
        APP_TWI_READ(LIS_I2C_ADDRESS, &int1_cfg[1], sizeof(int1_cfg[1]), 0),
        APP_TWI_WRITE(LIS_I2C_ADDRESS, &int1_ths[0], sizeof(int1_ths[0]), APP_TWI_NO_STOP),
        APP_TWI_READ(LIS_I2C_ADDRESS, &int1_ths[1], sizeof(int1_ths[1]), 0),
        APP_TWI_WRITE(LIS_I2C_ADDRESS, &int1_src[0], sizeof(int1_src[0]), APP_TWI_NO_STOP),
        APP_TWI_READ(LIS_I2C_ADDRESS, &int1_src[1], sizeof(int1_src[1]), 0),
        APP_TWI_WRITE(LIS_I2C_ADDRESS, &who[0], sizeof(who[0]), APP_TWI_NO_STOP),
        APP_TWI_READ(LIS_I2C_ADDRESS, &who[1], sizeof(who[1]), 0),
        APP_TWI_WRITE(LIS_I2C_ADDRESS, &reg1[0], sizeof(reg1[0]), APP_TWI_NO_STOP),
        APP_TWI_READ(LIS_I2C_ADDRESS, &reg1[1], sizeof(reg1[1]), 0),
        APP_TWI_WRITE(LIS_I2C_ADDRESS, &reg2[0], sizeof(reg2[0]), APP_TWI_NO_STOP),
        APP_TWI_READ(LIS_I2C_ADDRESS, &reg2[1], sizeof(reg2[1]), 0),
        APP_TWI_WRITE(LIS_I2C_ADDRESS, &reg3[0], sizeof(reg3[0]), APP_TWI_NO_STOP),
        APP_TWI_READ(LIS_I2C_ADDRESS, &reg3[1], sizeof(reg3[1]), 0),
        APP_TWI_WRITE(LIS_I2C_ADDRESS, &reg4[0], sizeof(reg4[0]), APP_TWI_NO_STOP),
        APP_TWI_READ(LIS_I2C_ADDRESS, &reg4[1], sizeof(reg4[1]), 0),
        APP_TWI_WRITE(LIS_I2C_ADDRESS, &reg5[0], sizeof(reg5[0]), APP_TWI_NO_STOP),
        APP_TWI_READ(LIS_I2C_ADDRESS, &reg5[1], sizeof(reg5[1]), 0),
        APP_TWI_WRITE(LIS_I2C_ADDRESS, &reg6[0], sizeof(reg6[0]), APP_TWI_NO_STOP),
        APP_TWI_READ(LIS_I2C_ADDRESS, &reg6[1], sizeof(reg6[1]), 0),
    };
    static app_twi_transaction_t const transaction = {
        .callback            = twi_callback,
        .p_user_data         = "LIS-POLL",
        .p_transfers         = transfers,
        .number_of_transfers = sizeof(transfers) / sizeof(transfers[0])
    };
    if (!twi_schedule(s, lis_poll_callback, &transaction)) {
        stats()->errors_lis++;
        sensor_unconfigure(s);
    }
    fPollRequestInProgress = true;

}

// Init sensor
bool s_lis_init(void *s, uint16_t param) {

    // Clear things used by the poller
    fPollerShouldClearInterrupt = false;
    fPollRequestInProgress = false;
    
    // Determine whether or not we are going to reset the chip
    if (!fInit) {

        // If we've never initialized, let's arm
        fArmForMotionSensing = true;
        fInit = true;

    } else {

        // If we sense that motion has occurred, let's reset the interrupt so that
        // future motion will be detected.  Otherwise, we'll just do nothing
        // because we're already armed and the pin doesn't need to be cleared
        if (gpio_motion_sense(MOTION_QUERY_PIN))
            fArmForMotionSensing = true;
        else
            fArmForMotionSensing = false;

    }

    // Init TWI
    if (fArmForMotionSensing) {
        if (!twi_init()) {
            fInit = false;  // Force reinitialization
            fArmForMotionSensing = false;
            stats()->errors_lis++;
            return false;
        }
        fTWIInitialized = true;
    }

    // Success
    return true;

}

// Term sensor
bool s_lis_term() {

    if (fTWIInitialized) {
        twi_term();
        fTWIInitialized = false;
    }

    return true;

}

#endif // TWILIS3DH
