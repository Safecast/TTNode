// Composite Virtual Particle Sensor
//
// I have gathered enough data that I've come to the conclusion that the most reliable way of
// gathering and comparing measurements from the two particle counters is to run them concurrently,
// having the PMS sensor "breathing the exhaust" of the OPC sensor.  This works well because the
// OPC sensor's physical design is focused around inhanling air through a panel-mounted tube,
// exhaling it from its fan.  The PMS sensor is not as carefully designed to separate intake from
// exhaust, and its design (with intake and exhaust relatively near each other) more-or-less suggests
// that it is measuring the nature of the ambient air surrounding it.
//
// In any case, this module creates a virtual "composite sensor" out of both the OPC and PMS sensors,
// ensuring that they run concurrently.  It also handles various error paths in which one or the other
// sensors is either unconfigured or otherwise unavailable.

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "debug.h"
#include "boards.h"
#include "sensor.h"
#include "gpio.h"
#include "air.h"
#include "opc.h"
#include "pms.h"

#ifdef AIRX

// If true, we stop retrying.  If false, we retry forever.
#define DECONFIGURE_FAILED_SENSORS false

#define MAX_FAILURES 3

uint32_t opc_total_init_failures = 0;
uint32_t opc_init_failures = 0;
uint32_t pms_total_init_failures = 0;
uint32_t pms_init_failures = 0;
uint32_t opc_total_term_failures = 0;
uint32_t opc_term_failures = 0;
uint32_t pms_total_term_failures = 0;
uint32_t pms_term_failures = 0;
bool opc_active = false;
bool pms_active = false;

// Measurement needed?
bool s_air_upload_needed(void *s) {
    bool upload = false;
#ifdef SPIOPC
    if (s_opc_upload_needed(s))
        upload = true;
#endif
#ifdef PMSX
    if (s_pms_upload_needed(s))
        upload = true;
#endif
    return upload;
}

// Measure it
void s_air_measure(void *s) {
#ifdef SPIOPC
    if (opc_active)
        s_opc_measure(NULL);
#endif
#ifdef PMSX
    if (pms_active)
        s_pms_measure(NULL);
#endif
    sensor_measurement_completed(s);
}

// Poller
void s_air_poll(void *g) {

    // Exit if we're not supposed to be here
    if (!sensor_is_polling_valid(g))
        return;

#ifdef SPIOPC
    if (opc_active)
        s_opc_poll(g);
#endif
#ifdef PMSX
    if (pms_active)
        s_pms_poll(g);
#endif

}

// Init sensor just after each power-on
void s_air_done_settling() {
#ifdef SPIOPC
    if (opc_active)
        s_opc_done_settling();
#endif
#ifdef PMSX
    if (pms_active)
        s_pms_done_settling();
#endif
}

// Init sensor just after each power-on
bool s_air_init() {
    bool pms_disabled = false;
    bool skip_this_init;
    
#ifdef SPIOPC
    // Initialize if we haven't had too many failures
#if DECONFIGURE_FAILED_SENSORS
    skip_this_init = (opc_init_failures > MAX_FAILURES || opc_term_failures > MAX_FAILURES);
#else
    skip_this_init = false;
#endif

    if (!skip_this_init) {
        if (!s_opc_init()) {
            opc_init_failures++;
            opc_total_init_failures++;
            if (debug(DBG_SENSOR))
                DEBUG_PRINTF("Air: OPC Init Failure #%d/%d\n", opc_init_failures, opc_total_init_failures);
        } else {
            opc_active = true;
            opc_init_failures = 0;
        }
    }
#endif

#ifdef PMSX

    // In the case (specifically LoRaWAN mode) in which the UART cannot be switched
    // to PMS even though sensor-defs.h "requested" it, don't even try.  We will
    // simply run with OPC.  This should never happen unless we truly couldn't
    // get our UART request granted, but just as a defensive measure we require this
    // to happen MAX_FAILURES times before we actually give up on the sensor.
#if PMSX==IOUART
    if (gpio_current_uart() != UART_PMS && pms_init_failures < MAX_FAILURES) {
        if (debug(DBG_SENSOR))
            DEBUG_PRINTF("Air: PMS Init Failure because current UART is not PMS\n");
        pms_disabled = true;
        pms_init_failures++;
        pms_total_init_failures++;
    }
#endif

    // Initialize if we haven't had too many failures
#if DECONFIGURE_FAILED_SENSORS
    skip_this_init = (pms_disabled || pms_init_failures > MAX_FAILURES || pms_term_failures > MAX_FAILURES);
#else
    skip_this_init = pms_disabled;
#endif

    if (!skip_this_init) {
        if (!s_pms_init()) {
            pms_init_failures++;
            pms_total_init_failures++;
            if (debug(DBG_SENSOR))
                DEBUG_PRINTF("Air: PMS Init Failure #%d/%d\n", pms_init_failures, pms_total_init_failures);
        } else {
            pms_active = true;
            pms_init_failures = 0;
        }
    }

#endif

    // If BOTH have failed with sequential errors, deconfigure the sensor
#if DECONFIGURE_FAILED_SENSORS
    if ((opc_init_failures > MAX_FAILURES || opc_term_failures > MAX_FAILURES)
        && (pms_init_failures > MAX_FAILURES || pms_term_failures > MAX_FAILURES)) {
        if (debug(DBG_SENSOR))
            DEBUG_PRINTF("Air: Too may failures; deconfiguring AIR entirely\n");
        return false;
    }
#endif
    
    // Done
    return true;
    
}

// Term sensor just before each power-off
bool s_air_term() {
#ifdef SPIOPC
    if (opc_active) {
        if (!s_opc_term()) {
            opc_term_failures++;
            opc_total_term_failures++;
            if (debug(DBG_SENSOR))
                DEBUG_PRINTF("Air: OPC Term Failure #%d/%d\n", opc_term_failures, opc_total_term_failures);
        } else {
            opc_term_failures = 0;
        }
        opc_active = false;
    }
#endif
#ifdef PMSX
    if (pms_active) {
        if (!s_pms_term()) {
            pms_term_failures++;
            pms_total_term_failures++;
            if (debug(DBG_SENSOR))
                DEBUG_PRINTF("Air: OPC PMS Failure #%d/%d\n", pms_term_failures, pms_total_term_failures);
        } else {
            pms_term_failures = 0;
        }
        pms_active = false;
    }
#endif
    return true;
}

#endif // AIRX
