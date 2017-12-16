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
#include "app_scheduler.h"
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
#include "stats.h"

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

static uint8_t sample_to_process[SAMPLE_LENGTH];
static uint8_t sample_to_process_length;
static uint8_t sample[SAMPLE_LENGTH];
static uint8_t sample_received_length;
static float samples_PM1[PMS_SAMPLE_MAX_BINS];
static float samples_PM2_5[PMS_SAMPLE_MAX_BINS];
static float samples_PM10[PMS_SAMPLE_MAX_BINS];
static uint16_t num_samples;
static uint16_t num_valid_samples;
static uint16_t num_nonzero_samples;
static uint16_t num_valid_reports;
static uint16_t num_samples_recorded;
static uint16_t num_samples_left_to_skip;
static bool pms_polling_ok = false;
static uint16_t previous_sample_checksum;
static uint32_t samples_count_00_30;
static uint32_t samples_count_00_50;
static uint32_t samples_count_01_00;
static uint32_t samples_count_02_50;
static uint32_t samples_count_05_00;
static uint32_t samples_count_10_00;
static uint16_t samples_count_seconds;
static uint32_t count_began;

static bool displayed_latency;
static uint32_t consecutive_std;

static bool     reported = false;
static bool     ever_reported = false;
static uint16_t reported_pm_1;
static uint16_t reported_pm_2_5;
static uint16_t reported_pm_10;
static float    reported_std_1;
static float    reported_std_2_5;
static float    reported_std_10;
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

// Term sensor just before each power-off
bool s_pms_term() {
    pms_polling_ok = false;
    if (num_valid_reports == 0) {
        DEBUG_PRINTF("PMS term: no valid reports!\n");
        stats()->errors_pms++;
        return false;
    }
    return true;
}

// One-time initialization of sensor
bool s_pms_init(void *s, uint16_t param) {
    s_pms_clear_measurement();
    state = STATE_WAITING_FOR_HEADER0;
    num_samples = 0;
    num_valid_samples = 0;
    num_nonzero_samples = 0;
    sample_received_length = 0;
    num_valid_reports = 0;
    pms_polling_ok = true;
    previous_sample_checksum = 0xDEAD;
    // Do a bit of settling each time we power up
    num_samples_left_to_skip = 50;
    return true;
}

// Process a fully-gathered sample
void sample_event_handler(void *unused1, uint16_t unused2) {
    uint16_t pms_01_0 = 0;
    uint16_t pms_02_5 = 0;
    uint16_t pms_10_0 = 0;
    uint16_t pms_c00_30 = 0;
    uint16_t pms_c00_50 = 0;
    uint16_t pms_c01_00 = 0;
    uint16_t pms_c02_50 = 0;
    uint16_t pms_c05_00 = 0;
    uint16_t pms_c10_00 = 0;

    // The macro we'll use to extract bytes from the sample
#define extract(msb,lsb) ( (sample_to_process[msb] << 8) | sample_to_process[lsb] )

    // Exit if we're not initialized.  This happens because data comes in immediately after power,
    // but BEFORE we've actually initialized the sensor.
    if (!pms_polling_ok)
        return;

    // Exit if we've already reported the value
    if (reported)
        return;

    // Report the latency until first sample, which - if too long - can indicate hardware issues
    if (!displayed_latency) {
        displayed_latency = true;
        if ((get_seconds_since_boot() - count_began) > 5) {
            DEBUG_PRINTF("PMS: %lds init latency\n", get_seconds_since_boot() - count_began);
            stats()->errors_pms++;
        }
    }

    // Exit if we're skipping samples, waiting for settling, and remember when
    // the sampling actually began.
    if (num_samples_left_to_skip != 0) {
        if (--num_samples_left_to_skip == 0)
            count_began = get_seconds_since_boot();
        return;
    }

    // Extract the checksum
#if defined(PMS1003) || defined(PMS5003) || defined(PMS7003)
    uint16_t pms_checksum = extract(30,31);
#else
    uint16_t pms_checksum = extract(22,23);
#endif

#if defined(PMS1003) || defined(PMS5003) || defined(PMS7003)
    pms_c00_30 = extract(16,17);
    pms_c00_50 = extract(18,19);
    pms_c01_00 = extract(20,21);
    pms_c02_50 = extract(22,23);
    pms_c05_00 = extract(24,25);
    pms_c10_00 = extract(26,27);
#endif

#if defined(PMS1003) || defined(PMS2003) || defined(PMS3003) || defined(PMS5003) || defined(PMS7003)
#ifdef USE_US_TSI_CF1_STANDARD
    pms_01_0 = extract(4,5);
    pms_02_5 = extract(6,7);
    pms_10_0 = extract(8,9);
#else // USE_INTERNATIONAL_ATMOSPHERIC_PM_ESTIMATION_ALSO_USED_BY_AQICN
    pms_01_0 = extract(10,11);
    pms_02_5 = extract(12,13);
    pms_10_0 = extract(14,15);
#endif
#endif

    // Zero value testing, just to make sure the code works and allows 0's all the way through
#ifdef AIR_ZERO_TEST
    pms_c00_30 = pms_c00_50 = pms_c01_00 = 0;
    pms_c02_50 = pms_c05_00 = pms_c10_00 = 0;
    pms_01_0 = pms_02_5 = pms_10_0 = 0;
#endif

    // Compute whether or not the sample is zero
    uint32_t sum = 0;
    sum += pms_c00_30;
    sum += pms_c00_50;
    sum += pms_c01_00;
    sum += pms_c02_50;
    sum += pms_c05_00;
    sum += pms_c10_00;
    sum += pms_01_0;
    sum += pms_02_5;
    sum += pms_10_0;

    // Bump the total that we've looked at
    num_samples++;
    if (sum != 0)
        num_nonzero_samples++;

    // Note that this is an awful kludge. If same checksum as last, drop this sample.
    // We do this because the unit seems to redundantly report several identical
    // responses before changing to the next one, and the number of identical
    // responses seems to vary quite a bit.  Since statistically it seems
    // VERY unlikely that the particle count of all sizes is literally
    // the same, this appears to be a safe technique.
    if (sum != 0 && pms_checksum == previous_sample_checksum)
        return;
    previous_sample_checksum = pms_checksum;

    // Record the sample stats
    num_valid_samples++;

    // Exit if we're not prepared to record it into a bin
    if (num_samples_recorded >= PMS_SAMPLE_MAX_BINS)
        return;

    // Add it to the running count
    samples_count_seconds = (uint16_t) (get_seconds_since_boot() - count_began);
    samples_count_00_30 += pms_c00_30;
    samples_count_00_50 += pms_c00_50;
    samples_count_01_00 += pms_c01_00;
    samples_count_02_50 += pms_c02_50;
    samples_count_05_00 += pms_c05_00;
    samples_count_10_00 += pms_c10_00;
    samples_PM1[num_samples_recorded] = pms_01_0;
    samples_PM2_5[num_samples_recorded] = pms_02_5;
    samples_PM10[num_samples_recorded] = pms_10_0;
    num_samples_recorded++;

    // Debug
#if defined(PMS1003) || defined(PMS5003) || defined(PMS7003)
    DEBUG_PRINTF("PMS %d %d %d (%d %d %d)\n", pms_01_0, pms_02_5, pms_10_0, pms_c00_30, pms_c00_50, pms_c01_00);
#else
    DEBUG_PRINTF("PMS %d %d %d\n", pms_01_0, pms_02_5, pms_10_0);
#endif

}

// Process byte received from the device
void pms_received_byte(uint8_t databyte) {

    switch (state) {

    case STATE_WAITING_FOR_HEADER0:
        if (databyte != 0x42)
            break;
        sample_received_length = 0;
        sample[sample_received_length++] = databyte;
        state = STATE_WAITING_FOR_HEADER1;
        break;

    case STATE_WAITING_FOR_HEADER1:
        if (databyte != 0x4D) {
            state = STATE_WAITING_FOR_HEADER0;
            break;
        }
        if (sample_received_length < SAMPLE_LENGTH)
            sample[sample_received_length++] = databyte;
        state = STATE_BUFFERING;
        break;

    case STATE_BUFFERING:
        if (sample_received_length < SAMPLE_LENGTH)
            sample[sample_received_length++] = databyte;
        if (sample_received_length >= SAMPLE_LENGTH) {
            state = STATE_WAITING_FOR_HEADER0;
            sample_to_process_length = sample_received_length;
            memcpy(sample_to_process, sample, SAMPLE_LENGTH);
            // Don't even bother to enqueue event if we know that it won't be recorded
            if (pms_polling_ok && !reported && num_samples_recorded < PMS_SAMPLE_MAX_BINS)
                app_sched_event_put(NULL, 0, sample_event_handler);
        }
        break;

    }

}

// Measurement needed?
bool s_pms_upload_needed(void *s) {
#if defined(PMS1003) || defined(PMS5003) || defined(PMS7003)
    return(s_pms_get_value(NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL));
#else
    return(s_pms_get_value(NULL, NULL, NULL, NULL, NULL, NULL));
#endif
}

// Make sure that all sensors on the PMS device have been read
void s_pms_measure(void *s) {
    float std1, std2_5, std10;

    reported_pm_1 = reported_pm_2_5 = reported_pm_10 = 0.0;
    reported_std_1 = reported_std_2_5 = reported_std_10 = 0.0;
    std1 = std2_5 = std10 = 0.0;
    reported_count_00_30 = 0;
    reported_count_00_50 = 0;
    reported_count_01_00 = 0;
    reported_count_02_50 = 0;
    reported_count_05_00 = 0;
    reported_count_10_00 = 0;
    reported_count_seconds = 0;

    // Avoid div by zero in the case of bad data!
    if (num_samples_recorded) {
        int i;

        for (i=0; i<num_samples_recorded; i++) {
            reported_pm_1 += samples_PM1[i];
            reported_pm_2_5 += samples_PM2_5[i];
            reported_pm_10 += samples_PM10[i];
        }
        reported_pm_1 = reported_pm_1 / num_samples_recorded;
        reported_pm_2_5 = reported_pm_2_5 / num_samples_recorded;
        reported_pm_10 = reported_pm_10 / num_samples_recorded;

        reported_count_00_30 = samples_count_00_30;
        reported_count_00_50 = samples_count_00_50;
        reported_count_01_00 = samples_count_01_00;
        reported_count_02_50 = samples_count_02_50;
        reported_count_05_00 = samples_count_05_00;
        reported_count_10_00 = samples_count_10_00;

        // Compute the standard deviations
        reported_std_1 = std1 = compute_maximum_deviation(samples_PM1, num_samples_recorded);
        reported_std_2_5 = std2_5 = compute_maximum_deviation(samples_PM2_5, num_samples_recorded);
        reported_std_10 = std10 = compute_maximum_deviation(samples_PM10, num_samples_recorded);

        // Apply a filter to the reported STD values to save bandwidth
        if (reported_pm_1 < AIR_MATERIAL_PM || reported_std_1 < (reported_pm_1*AIR_MATERIAL_STD_MULTIPLE))
            reported_std_1 = 0;
        if (reported_pm_2_5 < AIR_MATERIAL_PM || reported_std_2_5 < (reported_pm_2_5*AIR_MATERIAL_STD_MULTIPLE))
            reported_std_2_5 = 0;
        if (reported_pm_10 < AIR_MATERIAL_PM || reported_std_10 < (reported_pm_10*AIR_MATERIAL_STD_MULTIPLE))
            reported_std_10 = 0;

        // Valid
        reported_count_seconds = samples_count_seconds;
        num_valid_reports++;

    }

    // If we haven't measured sufficiently long, it's an error
    if (reported_count_seconds >= PMS_SAMPLE_PERIOD_MINIMUM_SECONDS)
        reported = ever_reported = true;
    else
        stats()->errors_pms++;

    // If high variance, don't allow it to pollute our data.  If repeated high variance, it's an error.
    if (reported_std_1 != 0 || reported_std_2_5 != 0 || reported_std_10 != 0) {
        if (++consecutive_std > 3)
            stats()->errors_pms++;
    } else
        consecutive_std = 0;

    // Debug
    if (debug(DBG_SENSOR_MAX)) {
        uint16_t num_zero = num_samples - num_nonzero_samples;
        uint16_t num_invalid = num_samples - num_valid_samples;
        if (!reported || consecutive_std != 0)
            DEBUG_PRINTF("PMS FAIL(recorded %d, zero %d, invalid %d, total %d) %d %d %d",
                         num_samples_recorded, num_zero, num_invalid, num_samples, reported_pm_1, reported_pm_2_5, reported_pm_10);
        else {
            char extra[64] = "";
            if (num_zero != 0)
                sprintf(extra, " (recorded %d, zero %d, invalid %d, total %d)", num_samples_recorded, num_zero, num_invalid, num_samples);
            if ((reported_pm_1 + reported_pm_2_5 + reported_pm_10) != 0)
                DEBUG_PRINTF("PMS reported%s %d %d %d", extra, reported_pm_1, reported_pm_2_5, reported_pm_10);
            else
                DEBUG_PRINTF("PMS reported%s %d %d %d (%d %d %d)", extra,
                             reported_pm_1, reported_pm_2_5, reported_pm_10,
                             reported_count_00_30, reported_count_00_50, reported_count_01_00);
        }
        DEBUG_PRINTF(" {%.0f %.0f %.0f} in %ds\n", std1, std2_5, std10, reported_count_seconds);
    }

    // Done with this sensor
    sensor_measurement_completed(s);
}

// Clear the values
void s_pms_clear_measurement() {
    reported = false;
    num_samples_recorded = 0;
    consecutive_std = 0;
    displayed_latency = false;
    samples_count_00_30 = samples_count_00_50 = samples_count_01_00 = samples_count_02_50 = samples_count_05_00 = samples_count_10_00 = 0;
    count_began = get_seconds_since_boot();
}

#if defined(PMSX) && PMSX==IOTWI
void pmstwi_callback(ret_code_t result, twi_context_t *t) {
    int i;

    if (!twi_completed(t)) {
        stats()->errors_pms++;
        return;
    }

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

    // Bump the number of seconds we've been counting, but stop at the max because
    // it's a waste to measure after we can no longer record the samples
    if (num_samples_recorded >= PMS_SAMPLE_MAX_BINS)
        return;

    // Issue the TWI command
#if defined(PMSX) && PMSX==IOTWI
    memset(twi_buffer, 0, sizeof(twi_buffer));
    static app_twi_transfer_t const transfers[] = {
        APP_TWI_READ(TWI_ADDRESS, twi_buffer, sizeof(twi_buffer), 0)
    };
    static app_twi_transaction_t const transaction = {
        .callback            = twi_callback,
        .p_user_data         = "PMS",
        .p_transfers         = transfers,
        .number_of_transfers = sizeof(transfers) / sizeof(transfers[0])
    };
    if (!twi_schedule(s, pmstwi_callback, &transaction))
        stats()->errors_pms++;

#endif

}

// Show the current value
bool s_pms_show_value(uint32_t when, char *buffer, uint16_t length) {
    static uint32_t last = 0;
    char msg[128];
    if (when == last)
        return false;
    last = when;
    if (ever_reported)
        sprintf(msg, "PMS %d %d %d", reported_pm_1, reported_pm_2_5, reported_pm_10);
    else
        sprintf(msg, "PMS not yet measured");
    strlcpy(buffer, msg, length);
    return true;
}

// Get the values
#if defined(PMS1003) || defined(PMS5003) || defined(PMS7003)
bool s_pms_get_value(uint16_t *ppms_pm01_0, uint16_t *ppms_pm02_5, uint16_t *ppms_pm10_0,
                     float *pstd_01_0, float *pstd_02_5, float *pstd_10_0,
                     uint32_t *ppms_c00_30, uint32_t *ppms_c00_50, uint32_t *ppms_c01_00, uint32_t *ppms_c02_50, uint32_t *ppms_c05_00, uint32_t *ppms_c10_00, uint16_t *ppms_csecs) {
#else
    bool s_pms_get_value(uint16_t *ppms_pm01_0, uint16_t *ppms_pm02_5, uint16_t *ppms_pm10_0
                         float *pstd_01_0, float *pstd_02_5, float *pstd_10_0) {
#endif

        if (ppms_pm01_0 != NULL)
            *ppms_pm01_0 = reported_pm_1;
        if (ppms_pm02_5 != NULL)
            *ppms_pm02_5 = reported_pm_2_5;
        if (ppms_pm10_0 != NULL)
            *ppms_pm10_0 = reported_pm_10;
        if (pstd_01_0 != NULL)
            *pstd_01_0 = reported_std_1;
        if (pstd_02_5 != NULL)
            *pstd_02_5 = reported_std_2_5;
        if (pstd_10_0 != NULL)
            *pstd_10_0 = reported_std_10;

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

        return reported;

    }

#endif // PMSX
