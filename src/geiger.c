// Geiger support

#include <stdint.h>
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
#include "twi.h"

#ifdef GEIGER

// These counters are maintained by the lowest level, which is
// the GPIOTE interrupt counter.  These are the only things
// maintained at that lowest level.
static bool m_geiger_reported = false;
static bool m_geiger0_avail = false;
static uint32_t m_geiger0_seconds = 0;
static uint32_t m_geiger0_count = 0;
static uint32_t m_geiger0_count_total = 0;
static bool m_geiger1_avail = false;
static uint32_t m_geiger1_seconds = 0;
static uint32_t m_geiger1_count = 0;
static uint32_t m_geiger1_count_total = 0;

// These counters are maintained by the geiger poller, which
// runs continuously but interrupts an an extremely low rate.
// The number of geiger buckets is tuned so that it is at LEAST
// as large as the typical frequency that we send the updates
// to the service.
#define GEIGER_BUCKETS (GEIGER_SAMPLE_SECONDS/GEIGER_BUCKET_SECONDS)
#define INVALID_COUNT 0xFFFFFFFFL
static uint16_t PrevValue0 = 0;
static uint32_t CounterValues0[GEIGER_BUCKETS];
static uint16_t PrevValue1 = 0;
static uint32_t CounterValues1[GEIGER_BUCKETS];
static uint32_t bucketsFilled0 = 0;
static uint32_t bucketsFilled1 = 0;

// These counters are maintained at the highest level, which
// is the sensor measurement call.  That call is done at
// a frequency that we
static uint32_t cpmValue0;
static uint32_t cpmValue1;

// Clear reported values
void s_geiger_clear_measurement() {
    m_geiger_reported = false;
}

// Report on geiger values
bool s_geiger_get_value(bool *pAvail0, uint32_t *pCPM0, bool *pAvail1, uint32_t *pCPM1) {
    if (pAvail0 != NULL)
        *pAvail0 = (m_geiger_reported && m_geiger0_avail);
    if (pCPM0 != NULL)
        *pCPM0 = cpmValue0;
    if (pAvail1 != NULL)
        *pAvail1 = (m_geiger_reported && m_geiger1_avail);
    if (pCPM1 != NULL)
        *pCPM1 = cpmValue1;
    return(m_geiger_reported);
}

// Initialize geiger
bool s_geiger_init() {
    int i;

    // Init the buckets
    PrevValue0 = 0;
    bucketsFilled0 = 0;
    for (i = 0; i < GEIGER_BUCKETS; i++)
        CounterValues0[i] = INVALID_COUNT;

    // Init the current value
    m_geiger0_count = 0;
    m_geiger0_seconds = 0;

    // Init the buckets
    PrevValue1 = 0;
    bucketsFilled1 = 0;
    for (i = 0; i < GEIGER_BUCKETS; i++)
        CounterValues1[i] = INVALID_COUNT;

    // Init the current value
    m_geiger1_count = 0;
    m_geiger1_seconds = 0;

    // Success
    return true;
}

// Measurement needed?
bool s_geiger_upload_needed(void *s) {
    return(s_geiger_get_value(NULL, NULL, NULL, NULL));
}

// Measure geiger values by analyzing the buckets maintained by poller
void s_geiger_measure(void *s) {
    int i;
    uint32_t bucketsPerMinute;
    uint32_t cpm0, cpm0Buckets;
    float mean0, compensated0;
    uint32_t cpm1, cpm1Buckets;
    float mean1, compensated1;

    // Compute the sums of all the buckets
    for (i = cpm0 = cpm0Buckets = 0; i < GEIGER_BUCKETS; i++) {
        if (CounterValues0[i] != INVALID_COUNT) {
            cpm0 += CounterValues0[i];
            cpm0Buckets++;
        }
    }

    // Calculate the mean count per minute, but only if there are at least one minute's worth of buckets.
    bucketsPerMinute = 60 / GEIGER_BUCKET_SECONDS;
    if (cpm0Buckets < bucketsPerMinute)
        mean0 = 0;
    else
        mean0 = (float) cpm0 / ((float) cpm0Buckets / bucketsPerMinute);

    // Apply the Medcom-recommended compensation that is consistent across ALL Safecast devices
    compensated0 = mean0 / (1 - (mean0 * 1.8833e-6));

    // Save the current measurements
    cpmValue0 = (uint32_t) compensated0;

    // See if the geiger has become available
    if (!m_geiger0_avail && m_geiger0_count_total > 5) {
        if (debug(DBG_SENSOR))
            DEBUG_PRINTF("Geiger #0 detected\n");
        m_geiger0_avail = true;
    }

    // Mark the measurement as available if it's statistically ok
    if (m_geiger0_avail && cpm0Buckets >= bucketsPerMinute)
        m_geiger_reported = true;

    // Compute the sums of all the buckets
    for (i = cpm1 = cpm1Buckets = 0; i < GEIGER_BUCKETS; i++) {
        if (CounterValues1[i] != INVALID_COUNT) {
            cpm1 += CounterValues1[i];
            cpm1Buckets++;
        }
    }

    // Calculate the mean count per minute, but only if there are at least one minute's worth of buckets.
    bucketsPerMinute = 60 / GEIGER_BUCKET_SECONDS;
    if (cpm1Buckets < bucketsPerMinute)
        mean1 = 0;
    else
        mean1 = (float) cpm1 / ((float) cpm1Buckets / bucketsPerMinute);

    // Apply the Medcom-recommended compensation that is consistent across ALL Safecast devices
    compensated1 = mean1 / (1 - (mean1 * 1.8833e-6));

    // Save the current measurements
    cpmValue1 = (uint32_t) compensated1;

    // See if geiger is available
    if (!m_geiger1_avail && m_geiger1_count_total > 5) {
        if (debug(DBG_SENSOR))
            DEBUG_PRINTF("Geiger #1 detected\n");
        m_geiger1_avail = true;
    }

    // Mark the measurement as available if it's statistically ok
    if (m_geiger1_avail && cpm1Buckets >= bucketsPerMinute)
        m_geiger_reported = true;

    // Mark the measurement as having been completed
    sensor_measurement_completed(s);
}

// Geiger poller
void s_geiger_poll(void *g) {
    uint32_t thisValue0, thisValue1;

    // Exit if we're not supposed to be here
    if (!sensor_is_polling_valid(g))
        return;

    // Update counters.  Note that we don't do this on the very first iteration
    // so that we don't ever have a partially-filled bucket
    thisValue0 = m_geiger0_count;
    m_geiger0_count = 0;
    PrevValue0++;
    if (PrevValue0 >= GEIGER_BUCKETS)
        PrevValue0 = 0;
    if (bucketsFilled0++)
        CounterValues0[PrevValue0] = thisValue0;
    m_geiger0_seconds += GEIGER_BUCKET_SECONDS;

    thisValue1 = m_geiger1_count;
    m_geiger1_count = 0;
    PrevValue1++;
    if (PrevValue1 >= GEIGER_BUCKETS)
        PrevValue1 = 0;
    if (bucketsFilled1++)
        CounterValues1[PrevValue1] = thisValue1;
    m_geiger1_seconds += GEIGER_BUCKET_SECONDS;

}

// Geiger events
void geiger0_event() {
        m_geiger0_count++;
        m_geiger0_count_total++;
}
void geiger1_event() {
        m_geiger1_count++;
        m_geiger1_count_total++;
}

#endif // GEIGER
