// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

// Plantower PMS device/sensor processing
//
//  PMS2003, PMS3003:
//    24 byte long messages via UART 9600 8N1 (3.3V TTL)
//    HEADER: (4 bytes), 2 pairs of bytes each of which MSB then LSB
//    [0,1]   Begin message       (hex:424D, ASCII 'BM')
//    [2,3]   Message body length (hex:0014, decimal 20)
//    BODY: (20 bytes), 10 pairs of bytes each of which MSB then LSB
//    [4,5]   PM 1.0 [ug/m3] (TSI standard)
//    [6,7]   PM 2.5 [ug/m3] (TSI standard)
//    [8,9]   PM 10. [ug/m3] (TSI standard)
//    [10,11] PM 1.0 [ug/m3] (std. atmosphere)
//    [12,13] PM 2.5 [ug/m3] (std. atmosphere)
//    [14,15] PM 10. [ug/m3] (std. atmosphere)
//    [16,17] unknown
//    [18,19] unknown
//    [20,21] unknown
//    [22,23] cksum of BODY bytes
//
//  PMS1003, PMS5003, PMS7003:
//    32 byte long messages via UART 9600 8N1 (3.3V TTL)
//    HEADER: (4 bytes), 2 pairs of bytes each of which MSB then LSB
//    [0,1]   Begin message       (hex:424D, ASCII 'BM')
//    [2,3]   Message body length (hex:001C, decimal 28)
//    BODY: (28 bytes), 14 pairs of bytes each of which MSB then LSB
//    [4,5]   PM 1.0 [ug/m3] (TSI standard)
//    [6,7]   PM 2.5 [ug/m3] (TSI standard)
//    [8,9]   PM 10. [ug/m3] (TSI standard)
//    [10,11] PM 1.0 [ug/m3] (std. atmosphere)
//    [12,13] PM 2.5 [ug/m3] (std. atmosphere)
//    [14,15] PM 10. [ug/m3] (std. atmosphere)
//    [16,17] num. particles with diameter > 0.3 um in 100 cm3 of air
//    [18,19] num. particles with diameter > 0.5 um in 100 cm3 of air
//    [20,21] num. particles with diameter > 1.0 um in 100 cm3 of air
//    [22,23] num. particles with diameter > 2.5 um in 100 cm3 of air
//    [24,25] num. particles with diameter > 5.0 um in 100 cm3 of air
//    [26,27] num. particles with diameter > 10. um in 100 cm3 of air
//    [28,29] unknown
//    [30,31] cksum of BODY bytes

#ifdef PMSX

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
#include "twi.h"
#include "gpio.h"
#include "config.h"
#include "timer.h"
#include "sensor.h"
#include "storage.h"
#include "comm.h"
#include "misc.h"
#include "pms.h"
#include "io.h"

// Define states
#define STATE_WAITING_FOR_HEADER0       0
#define STATE_WAITING_FOR_HEADER1       1
#define STATE_BUFFERING                 2
static uint16_t state = STATE_WAITING_FOR_HEADER0;

// Header length
#if defined(PMS2003) || defined(PMS3003)
#define SAMPLE_LENGTH 24
#endif
#if defined(PMS1003) || defined(PMS5003) || defined(PMS7003)
#define SAMPLE_LENGTH 32
#endif
#ifndef SAMPLE_LENGTH
Error - unknown PMS sensor type
#endif

static bool settling = true;
static uint8_t sample[SAMPLE_LENGTH];
static uint8_t sample_received;

struct sample_s {
    uint16_t PM1;
    uint16_t PM2_5;
    uint16_t PM10;
};
typedef struct sample_s sample_t;
static sample_t samples[PMS_SAMPLE_MAX_BINS];
static uint16_t num_valid_reports;
static uint16_t num_samples;
static bool pms_polling_ok = false;
static uint16_t sample_checksum;
static uint32_t count_00_30;
static uint32_t count_00_50;
static uint32_t count_01_00;
static uint32_t count_02_50;
static uint32_t count_05_00;
static uint32_t count_10_00;
static uint16_t count_seconds;

static bool reported = false;
static sample_t reported_pm;
static uint32_t reported_count_00_30;
static uint32_t reported_count_00_50;
static uint32_t reported_count_01_00;
static uint32_t reported_count_02_50;
static uint32_t reported_count_05_00;
static uint32_t reported_count_10_00;
static uint16_t reported_count_seconds;

// For the TWI interface
#if defined(PMSX) && PMSX==IOTWI
#define TWI_ADDRESS       0x12
#define TWI_DATA_LEN      SAMPLE_LENGTH
static uint8_t twi_buffer[TWI_DATA_LEN];
#endif

// Init sensor just after each power-on
void s_pms_done_settling() {

    // Clear out the values
    s_pms_clear_measurement();

}

// Term sensor just before each power-off
bool s_pms_term() {
    pms_polling_ok = false;
    if (num_valid_reports == 0) {
        DEBUG_PRINTF("PMS term: no valid reports!\n");
        return false;
    }
    return true;
}

// One-time initialization of sensor
bool s_pms_init(uint16_t param) {
    settling = true;
    state = STATE_WAITING_FOR_HEADER0;
    sample_received = 0;
    num_valid_reports = 0;
    pms_polling_ok = true;
    return true;
}

// Process a fully-gathered sample
void process_sample() {

#define extract(msb,lsb) ( (sample[msb] << 8) | sample[lsb] )

    // Extract the checksum.  If it's same as last time, drop this sample.
    // We do this because the unit seems to redundantly report several identical
    // responses before changing to the next one, and the number of identical
    // responses seems to vary quite a bit.  Since statistically it seems
    // VERY unlikely that the particle count of all sizes is literally
    // the same, this appears to be a safe technique.
#if defined(PMS1003) || defined(PMS5003) || defined(PMS7003)
    uint16_t pms_checksum = extract(30,31);
#else
    uint16_t pms_checksum = extract(22,23);
#endif
    if (pms_checksum == sample_checksum)
        return;
    sample_checksum = pms_checksum;

#if defined(PMS1003) || defined(PMS5003) || defined(PMS7003)
    uint16_t pms_c00_30 = extract(16,17);
    uint16_t pms_c00_50 = extract(18,19);
    uint16_t pms_c01_00 = extract(20,21);
    uint16_t pms_c02_50 = extract(22,23);
    uint16_t pms_c05_00 = extract(24,25);
    uint16_t pms_c10_00 = extract(26,27);
    // Disregard the sample if it's invalid.  (Statistically, you can't really count zero particles.)
    if ((pms_c00_30 + pms_c00_50 + pms_c01_00 + pms_c02_50 + pms_c05_00 + pms_c10_00) == 0)
        return;
    if (count_seconds < AIR_SAMPLE_TOTAL_SECONDS) {
        count_00_30 += pms_c00_30;
        count_00_50 += pms_c00_50;
        count_01_00 += pms_c01_00;
        count_02_50 += pms_c02_50;
        count_05_00 += pms_c05_00;
        count_10_00 += pms_c10_00;
    }
#endif

#if defined(PMS1003) || defined(PMS2003) || defined(PMS3003) || defined(PMS5003) || defined(PMS7003)
    uint16_t pms_tsi_01_0 = extract(4,5);
    uint16_t pms_tsi_02_5 = extract(6,7);
    uint16_t pms_tsi_10_0 = extract(8,9);
    uint16_t pms_std_01_0 = extract(10,11);
    uint16_t pms_std_02_5 = extract(12,13);
    uint16_t pms_std_10_0 = extract(14,15);
    UNUSED_VARIABLE(pms_std_01_0);
    UNUSED_VARIABLE(pms_std_02_5);
    UNUSED_VARIABLE(pms_std_10_0);
    if (num_samples < PMS_SAMPLE_MAX_BINS) {
        samples[num_samples].PM1 = pms_tsi_01_0;
        samples[num_samples].PM2_5 = pms_tsi_02_5;
        samples[num_samples].PM10 = pms_tsi_10_0;
        num_samples++;
    }
#endif

    if (debug(DBG_AIR|DBG_SENSOR_MAX) && !settling) {
#if defined(PMS1003) || defined(PMS5003) || defined(PMS7003)
        DEBUG_PRINTF("PMS %d %d %d (%d %d %d %d %d %d)\n", pms_tsi_01_0, pms_tsi_02_5, pms_tsi_10_0, pms_c00_30, pms_c00_50, pms_c01_00, pms_c02_50, pms_c05_00, pms_c10_00);
#else
        DEBUG_PRINTF("PMS %d %d %d\n", pms_tsi_01_0, pms_tsi_02_5, pms_tsi_10_0);
#endif
    }

}

// Process byte received from the device
void pms_received_byte(uint8_t databyte) {

    switch (state) {

    case STATE_WAITING_FOR_HEADER0:
        if (databyte != 0x42)
            break;
        sample_received = 0;
        if (sample_received < SAMPLE_LENGTH)
            sample[sample_received++] = databyte;
        state = STATE_WAITING_FOR_HEADER1;
        break;

    case STATE_WAITING_FOR_HEADER1:
        if (databyte != 0x4D) {
            state = STATE_WAITING_FOR_HEADER0;
            break;
        }
        if (sample_received < SAMPLE_LENGTH)
            sample[sample_received++] = databyte;
        state = STATE_BUFFERING;
        break;

    case STATE_BUFFERING:
        if (sample_received < SAMPLE_LENGTH)
            sample[sample_received++] = databyte;
        if (sample_received >= SAMPLE_LENGTH) {
            state = STATE_WAITING_FOR_HEADER0;
            process_sample();
        }
        break;

    }

}

// Measurement needed?
bool s_pms_upload_needed(void *s) {
#if defined(PMS1003) || defined(PMS5003) || defined(PMS7003)
    return(s_pms_get_value(NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL));
#else
    return(s_pms_get_value(NULL, NULL, NULL));
#endif
}

// Make sure that all sensors on the PMS device have been read
void s_pms_measure(void *s) {

    reported_pm.PM1 = reported_pm.PM2_5 = reported_pm.PM10 = 0.0;
    reported_count_00_30 = 0;
    reported_count_00_50 = 0;
    reported_count_01_00 = 0;
    reported_count_02_50 = 0;
    reported_count_05_00 = 0;
    reported_count_10_00 = 0;
    reported_count_seconds = 0;

    // Avoid div by zero in the case of bad data!
    if (num_samples) {
        int i;

        for (i=0; i<num_samples; i++) {
            reported_pm.PM1 += samples[i].PM1;
            reported_pm.PM2_5 += samples[i].PM2_5;
            reported_pm.PM10 += samples[i].PM10;
        }

        reported_pm.PM1 = reported_pm.PM1 / num_samples;
        reported_pm.PM2_5 = reported_pm.PM2_5 / num_samples;
        reported_pm.PM10 = reported_pm.PM10 / num_samples;

        reported_count_00_30 = count_00_30;
        reported_count_00_50 = count_00_50;
        reported_count_01_00 = count_01_00;
        reported_count_02_50 = count_02_50;
        reported_count_05_00 = count_05_00;
        reported_count_10_00 = count_10_00;

        reported_count_seconds = count_seconds;

        num_valid_reports++;

    }

#if AIR_IGNORE_INCOMPLETE_SAMPLES
    if (reported_count_seconds >= AIR_SAMPLE_TOTAL_SECONDS)
        reported = true;
#else
    reported = true;
#endif

    if (debug(DBG_AIR|DBG_SENSOR_MAX))
        DEBUG_PRINTF("%sPMS reported(%d/%d) %d %d %d (%d %d %d %d %d %d)\n",
                     reported_count_seconds < AIR_SAMPLE_TOTAL_SECONDS ? "BAD: " : "",
                     num_samples, reported_count_seconds,
                     reported_pm.PM1, reported_pm.PM2_5, reported_pm.PM10,
                     reported_count_00_30, reported_count_00_50, reported_count_01_00,
                     reported_count_02_50, reported_count_05_00, reported_count_10_00);

    // Done with this sensor
    sensor_measurement_completed(s);
}

// Clear the values
void s_pms_clear_measurement() {
    reported = false;
    num_samples = 0;
    count_00_30 = count_00_50 = count_01_00 = count_02_50 = count_05_00 = count_10_00 = 0;
    count_seconds = 0;
    settling = false;
}

#if defined(PMSX) && PMSX==IOTWI
void twi_callback(ret_code_t result, void *io) {
    int i;

    if (!twi_completed("PMS", result))
        return;

    if (debug(DBG_SENSOR_MAX)) {
        DEBUG_PRINTF("TWI: ");
        for (i=0; i<sizeof(twi_buffer); i++)
            DEBUG_PRINTF("%02x", twi_buffer[i]);
        DEBUG_PRINTF("\n");
    }

    for (i=0; i<sizeof(twi_buffer); i++)
        pms_received_byte(twi_buffer[i]);

}
#endif

// Poller
void s_pms_poll(void *s) {

    // Exit if we're not supposed to be here
    if (!sensor_is_polling_valid(s))
        return;
    if (!pms_polling_ok)
        return;

    // Bump the number of seconds we've been counting, but stop at the max
    // because that's when the actual counts stop getting added to the total
    if (count_seconds < AIR_SAMPLE_TOTAL_SECONDS)
        count_seconds += AIR_SAMPLE_SECONDS;

    // Issue the TWI command
#if defined(PMSX) && PMSX==IOTWI
    memset(twi_buffer, 0, sizeof(twi_buffer));
    static app_twi_transfer_t const transfers[] = {
        APP_TWI_READ(TWI_ADDRESS, twi_buffer, sizeof(twi_buffer), 0)
    };
    static app_twi_transaction_t const transaction = {
        .callback            = twi_callback,
        .p_user_data         = NULL,
        .p_transfers         = transfers,
        .number_of_transfers = sizeof(transfers) / sizeof(transfers[0])
    };
    twi_schedule("PMS", &transaction);
#endif

}

// Get the values

#if defined(PMS1003) || defined(PMS5003) || defined(PMS7003)
bool s_pms_get_value(uint16_t *ppms_pm01_0, uint16_t *ppms_pm02_5, uint16_t *ppms_pm10_0,
                     uint32_t *ppms_c00_30, uint32_t *ppms_c00_50, uint32_t *ppms_c01_00, uint32_t *ppms_c02_50, uint32_t *ppms_c05_00, uint32_t *ppms_c10_00, uint16_t *ppms_csecs) {
#else
    bool s_pms_get_value(uint16_t *ppms_pm01_0, uint16_t *ppms_pm02_5, uint16_t *ppms_pm10_0) {
#endif

        if (!reported)
            return false;

        if (ppms_pm01_0 != NULL)
            *ppms_pm01_0 = reported_pm.PM1;
        if (ppms_pm02_5 != NULL)
            *ppms_pm02_5 = reported_pm.PM2_5;
        if (ppms_pm10_0 != NULL)
            *ppms_pm10_0 = reported_pm.PM10;

#if defined(PMS1003) || defined(PMS5003) || defined(PMS7003)
        if (ppms_c00_30 != NULL)
            *ppms_c00_30 = reported_count_00_30;
        if (ppms_c00_50 != NULL)
            *ppms_c00_50 = reported_count_00_50;
        if (ppms_c01_00 != NULL)
            *ppms_c01_00 = reported_count_01_00;
        if (ppms_c02_50 != NULL)
            *ppms_c02_50 = reported_count_02_50;
        if (ppms_c05_00 != NULL)
            *ppms_c05_00 = reported_count_05_00;
        if (ppms_c10_00 != NULL)
            *ppms_c10_00 = reported_count_10_00;
        if (ppms_csecs != NULL)
            *ppms_csecs = reported_count_seconds;
#endif

        return true;
    }

#endif // PMSX
