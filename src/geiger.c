// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

// Geiger tube support

#include <stdint.h>
#include <string.h>
#include "debug.h"
#include "nrf.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_gpiote.h"
#include "nrf_drv_gpiote.h"
#include "app_gpiote.h"
#include "boards.h"
#include "softdevice_handler.h"
#include "storage.h"
#include "config.h"
#include "serial.h"
#include "timer.h"
#include "sensor.h"
#include "io.h"
#include "gpio.h"
#include "geiger.h"
#include "comm.h"
#include "misc.h"
#include "twi.h"
#include "stats.h"

#ifdef GEIGERX

// These counters are maintained by the lowest level, which is
// the GPIOTE interrupt counter.  These are the only things
// maintained at that lowest level.
static bool valuesHaveBeenUpdated = false;
static bool value0IsReportable = false;
static bool value0EverReportable = false;
static uint32_t reportableValue0;
static uint32_t lastValue0;
static bool value1IsReportable = false;
static bool value1EverReportable = false;
static uint32_t reportableValue1;
static uint32_t lastValue1;
static bool geiger0IsAvailable = false;
static uint32_t geiger0InterruptCount = 0;
static uint32_t geiger0InterruptCount_total = 0;
static bool geiger1IsAvailable = false;
static uint32_t geiger1InterruptCount = 0;
static uint32_t geiger1InterruptCount_total = 0;
static bool geigerPowerOn = false;
static int bucketsLeftDuringSettling = 0;
static int bucketsLeftToFillAfterPowerOn = 0;
static bool geigerSensorMeasurementInProgress = false;
static uint32_t geigerSensorMeasurementBegan = 0;

// These counters are maintained by the geiger poller, which
// runs continuously but interrupts an an extremely low rate.
// The number of geiger buckets is tuned so that it is at LEAST
// as large as the typical frequency that we send the updates
// to the service.
#define GEIGER_INTEGRATION_BUCKETS (MAX(GEIGER_MOBILE_INTEGRATION_SECONDS,GEIGER_FIXED_INTEGRATION_SECONDS)/GEIGER_BUCKET_SECONDS)
#define INVALID_COUNT 0xFFFFFFFFL
static uint16_t currentBucket = 0;
static uint32_t bucket0[GEIGER_INTEGRATION_BUCKETS];
static uint32_t bucket1[GEIGER_INTEGRATION_BUCKETS];

// Forwards
void geiger_power_on();

// Geiger events
void geiger0_event() {
    geiger0InterruptCount++;
}
void geiger1_event() {
    geiger1InterruptCount++;
}

// Get the number of integration seconds based on current mode
uint16_t geiger_integration_seconds() {
    if (sensor_op_mode() == OPMODE_MOBILE)
        return GEIGER_MOBILE_INTEGRATION_SECONDS;
    return GEIGER_FIXED_INTEGRATION_SECONDS;
}

// Clear reported values
void s_geiger_clear_measurement() {
    value0IsReportable = value1IsReportable = false;
    valuesHaveBeenUpdated = false;
}

// Display last known geiger values
bool s_geiger_show_value(uint32_t when, char *buffer, uint16_t length) {
    static uint32_t last = 0;
    char msg[128];
    if (when == last)
        return false;
    last = when;
    if (value0EverReportable && value1EverReportable) {
        sprintf(msg, "CPM %ldcpm %ldcpm", reportableValue0, reportableValue1);
    } else if (value0EverReportable) {
        sprintf(msg, "CPM0 %ldcpm", reportableValue0);
    } else if (value1EverReportable) {
        sprintf(msg, "CPM1 %ld cpm", reportableValue1);
    } else
        sprintf(msg, "CPM not yet measured");
    strlcpy(buffer, msg, length);
    return true;
}

// Report on geiger values
bool s_geiger_get_value(bool *pAvail0, uint32_t *pCPM0, bool *pAvail1, uint32_t *pCPM1) {
    if (pAvail0 != NULL)
        *pAvail0 = value0IsReportable;
    if (pCPM0 != NULL)
        *pCPM0 = reportableValue0;
    if (pAvail1 != NULL)
        *pAvail1 = value1IsReportable;
    if (pCPM1 != NULL)
        *pCPM1 = reportableValue1;
    return(value0IsReportable || value1IsReportable);
}

// Skip this sensor if we're in a mode in which we need to get something sent out
// Note that we call this with NULL elsewhere in this file, so do not use the argument
bool g_geiger_skip(void *g_do_not_use_or_you_will_segfault) {

    // Do special skip processing if we're in mobile mode
    if (sensor_op_mode() == OPMODE_MOBILE) {

        // If currently selected, skip this so that we don't generate a massive
        // volume of data while online
        if (!comm_is_deselected())
            return true;

        // If we're not in fona mode, generate some data so we will switch
        // quickly to Fona mode on the next reselect
        if (comm_mode() != COMM_FONA) {
            return false;
        }

        // If currently the GPS is inactive, skip the measurement
        if (!comm_gps_active()) {
            return true;
        }

    }
    return false;
}

// Measurement needed?
bool s_geiger_upload_needed(void *s) {

    // If nothing has been updated since we cleared the value, no upload needed
    if (!valuesHaveBeenUpdated)
        return false;

    // See if there's a value
    return(s_geiger_get_value(NULL, NULL, NULL, NULL));
}

// Measure geiger values by analyzing the buckets maintained by poller
void s_geiger_measure(void *s) {

    // Exit without completing sensor if the values aren't yet available
    if (!s_geiger_get_value(NULL, NULL, NULL, NULL)) {

        // This is defensive coding for bad hardware, but if for any reason we've
        // been measuring for much longer than we should ever be measuring, "complete"
        // the measurement so that we don't hang in this sensor measurement indefinitely.
        if (sensor_op_mode() != OPMODE_MOBILE)
            if (!ShouldSuppress(&geigerSensorMeasurementBegan, GEIGER_FIXED_INTEGRATION_SECONDS*4)) {
                DEBUG_PRINTF("GEIGER polling failed!\n");
                stats()->errors_geiger++;
                sensor_measurement_completed(s);
            }

        return;
    }

    // Error handling
#define MAXTESTCPM 500
#if G0!=0
    if (!geiger0IsAvailable)
        stats()->errors_geiger++;
    else if (value0IsReportable && reportableValue0 > MAXTESTCPM)
        stats()->errors_geiger++;
#endif
#if G1!=0
    if (!geiger1IsAvailable)
        stats()->errors_geiger++;
    else if (value1IsReportable && reportableValue1 > MAXTESTCPM)
        stats()->errors_geiger++;
#endif

    // Debugging
    if (value0IsReportable && value1IsReportable) {
        DEBUG_PRINTF("GEIGER reported %ld %ld\n", reportableValue0, reportableValue1);
    } else if (value0IsReportable) {
        DEBUG_PRINTF("GEIGER reported %ld -\n", reportableValue0);
    } else if (value1IsReportable) {
        DEBUG_PRINTF("GEIGER reported - %ld\n", reportableValue1);
    }

    // Done
    sensor_measurement_completed(s);

}

// Update the buckets where we accumulate the data from the counts.
// This *must* be called every GEIGER_BUCKET_SECONDS.
void geiger_bucket_update() {
    int i;
    uint32_t interruptCount0, interruptCount1;

    // Grab the values from the interrupt counters, and clear them out
    interruptCount0 = geiger0InterruptCount;
    geiger0InterruptCount = 0;
    interruptCount1 = geiger1InterruptCount;
    geiger1InterruptCount = 0;

    // Take note of when the geigers become available
#define PULSE_DEBOUNCE 5
    geiger0InterruptCount_total += interruptCount0;
    if (!geiger0IsAvailable && geiger0InterruptCount_total > PULSE_DEBOUNCE) {
        geiger0IsAvailable = true;
        if (debug(DBG_SENSOR))
            DEBUG_PRINTF("Geiger #0 detected\n");
    }
    geiger1InterruptCount_total += interruptCount1;
    if (!geiger1IsAvailable && geiger1InterruptCount_total > PULSE_DEBOUNCE) {
        geiger1IsAvailable = true;
        if (debug(DBG_SENSOR))
            DEBUG_PRINTF("Geiger #1 detected\n");
    }

    // Process geiger settling and filling
    if (bucketsLeftToFillAfterPowerOn > 0) {
        --bucketsLeftToFillAfterPowerOn;

        if (bucketsLeftDuringSettling > 0) {
            --bucketsLeftDuringSettling;
            if (debug(DBG_SENSOR))
                DEBUG_PRINTF("CPM settling (+%d +%d)\n", interruptCount0, interruptCount1);
            return;
        }

    } else {

        // Don't generate anything reportable if we're skipping
        if (g_geiger_skip(NULL))
            return;


    }

    // Insert the up-to-date interrupt counters into the bucket
    if (++currentBucket >= GEIGER_INTEGRATION_BUCKETS)
        currentBucket = 0;
    bucket0[currentBucket] = interruptCount0;
    bucket1[currentBucket] = interruptCount1;

    // Sum up the bucket contents
    uint32_t cpm0 = 0;
    uint32_t cpm0buckets = 0;
    if (geiger0IsAvailable) {
        value0IsReportable = true;
        for (i = 0; i < GEIGER_INTEGRATION_BUCKETS; i++) {
            if (bucket0[i] == INVALID_COUNT)
                value0IsReportable = false;
            else {
                cpm0 += bucket0[i];
                cpm0buckets++;
            }
        }
        if (value0IsReportable)
            value0EverReportable = true;
    }
    uint32_t cpm1 = 0;
    uint32_t cpm1buckets = 0;
    if (geiger1IsAvailable) {
        value1IsReportable = true;
        for (i = 0; i < GEIGER_INTEGRATION_BUCKETS; i++) {
            if (bucket1[i] == INVALID_COUNT)
                value1IsReportable = false;
            else {
                cpm1 += bucket1[i];
                cpm1buckets++;
            }
        }
        if (value1IsReportable)
            value1EverReportable = true;
    }

    // Compute compensated means
    float bucketsPerMinute = (float) 60 / GEIGER_BUCKET_SECONDS;
    float mean, compensated, divisor, secondsPerBucketPerMinute;
    lastValue0 = 0;
    if (cpm0buckets) {
        secondsPerBucketPerMinute  = ((float) cpm0buckets) / bucketsPerMinute;
        mean = (float) cpm0 / secondsPerBucketPerMinute;
        divisor = 1 - (mean * 1.8833e-6);
        if (divisor)
            compensated = mean / divisor;
        else
            compensated = 0.0;
        lastValue0 = (uint32_t) compensated;
        if (value0IsReportable) {
            reportableValue0 = lastValue0;
            valuesHaveBeenUpdated = true;
        }
    }
    lastValue1 = 0;
    if (cpm1buckets) {
        secondsPerBucketPerMinute  = ((float) cpm1buckets) / bucketsPerMinute;
        mean = (float) cpm1 / secondsPerBucketPerMinute;
        divisor = 1 - (mean * 1.8833e-6);
        if (divisor)
            compensated = mean / divisor;
        else
            compensated = 0.0;
        lastValue1 = (uint32_t) compensated;
        if (value1IsReportable) {
            reportableValue1 = lastValue1;
            valuesHaveBeenUpdated = true;
        }
    }

    // Done
    int totalIterations = geiger_integration_seconds()/GEIGER_BUCKET_SECONDS;
    int percentComplete = (int) (((float) (totalIterations - bucketsLeftToFillAfterPowerOn) / totalIterations) * 100);
    if (percentComplete < 0) percentComplete = 0;
    if (geiger0IsAvailable && geiger1IsAvailable)
        DEBUG_PRINTF("CPM %d %d (%d %d) %d%%\n", lastValue0, lastValue1, interruptCount0, interruptCount1, percentComplete);
    else if (geiger0IsAvailable)
        DEBUG_PRINTF("CPM %d - (%d %d) %d%%\n", lastValue0, interruptCount0, interruptCount1, percentComplete);
    else if (geiger1IsAvailable)
        DEBUG_PRINTF("CPM - %d (%d %d) %d%%\n", lastValue1, interruptCount0, interruptCount1, percentComplete);

}

// Geiger master poller, utilized only when the sensor's poller isn't already
// being used.  This is for mobile mode handling.
void geiger_poll() {

    // We're the designated polling handler in mobile mode
    if (sensor_op_mode() != OPMODE_MOBILE)
        return;

    // Ensure that geiger power is on
    geiger_power_on();

    // Call the GEIGER_BUCKET_SECONDS poller
    geiger_bucket_update();

}

// Geiger poller
void s_geiger_poll(void *s) {

    // Exit if we're not supposed to be here
    if (!sensor_is_polling_valid(s))
        return;

    // Exit if power isn't on.
    if (!geigerPowerOn)
        return;

    // We're the designated polling handler in non-mobile mode
    if (sensor_op_mode() == OPMODE_MOBILE)
        return;

    // Call the GEIGER_BUCKET_SECONDS poller
    geiger_bucket_update();

}

// Turn on geiger if it's not on
void geiger_power_on() {

    // Turn on the power.  Note that we do our own
    // power management outside the sensor package so that
    // we can control whether or not we leave it turned on
    // across many iterations.
    if (!geigerPowerOn) {
        int i;

        // Turn on the power
#ifdef POWER_PIN_GEIGER
        gpio_power_set(POWER_PIN_GEIGER, true);
        DEBUG_PRINTF("s-geiger power ON\n");
#endif
        geigerPowerOn = true;

        // Init the buckets
        currentBucket = 0;
        for (i = 0; i < GEIGER_INTEGRATION_BUCKETS; i++) {
            bucket0[i] = INVALID_COUNT;
            bucket1[i] = INVALID_COUNT;
        }

        // After powering on, allow settling for stabilization.  When we're in mobile mode,
        // power-on only happens up-front and the geiger stays running continuously.
        bucketsLeftDuringSettling = GEIGER_SETTLING_SECONDS/GEIGER_BUCKET_SECONDS;
        bucketsLeftToFillAfterPowerOn = geiger_integration_seconds()/GEIGER_BUCKET_SECONDS + bucketsLeftDuringSettling;

        // Init the current values
        geiger0InterruptCount = 0;
        geiger1InterruptCount = 0;

        // Clear whether or not the values are reportable
        s_geiger_clear_measurement();

    }

}

// Initialize geiger
bool s_geiger_init(void *s, uint16_t param) {

    // Note that we're now within sensor processing
    geigerSensorMeasurementInProgress = true;
    geigerSensorMeasurementBegan = get_seconds_since_boot();

    // Turn on power if it's not
    geiger_power_on();

    // Success
    return true;
}

// Terminate geiger
bool s_geiger_term() {

    // Note that we're no longer within sensor processing
    geigerSensorMeasurementInProgress = false;

    // Turn off the power if and only if we're not in mobile mode
    if (geigerPowerOn && sensor_op_mode() != OPMODE_MOBILE) {
        geigerPowerOn = false;
#ifdef POWER_PIN_GEIGER
        gpio_power_set(POWER_PIN_GEIGER, false);
        DEBUG_PRINTF("s-geiger power OFF\n");
#endif
    }

    return true;

}

#endif // GEIGERX
