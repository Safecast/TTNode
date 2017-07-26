// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

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

// Show the current value
bool s_air_show_value(uint32_t when, char *buffer, uint16_t length) {
    bool is_opc, is_pms;
    char msg_opc[128], msg_pms[128];
    is_opc = s_opc_show_value(when, msg_opc, sizeof(msg_opc)-1);
    is_pms = s_pms_show_value(when, msg_pms, sizeof(msg_pms)-1);
    if (is_opc && is_pms) {
        char msg[256];
        sprintf(msg, "%s\n%s", msg_pms, msg_opc);
        strlcpy(buffer, msg, length);
    } else if (is_opc) {
        strlcpy(buffer, msg_opc, length);
    } else if (is_pms) {
        strlcpy(buffer, msg_pms, length);
    } else {
        return false;
    }
    return true;
}

// Poller
void s_air_poll(void *s) {

    // Exit if we're not supposed to be here
    if (!sensor_is_polling_valid(s))
        return;

#ifdef SPIOPC
    if (opc_active)
        s_opc_poll(s);
#endif
#ifdef PMSX
    if (pms_active)
        s_pms_poll(s);
#endif

}

// Init sensor just after each power-on
bool s_air_init(void *s, uint16_t param) {

#ifdef SPIOPC
    if (s_opc_init(s, param))
        opc_active = true;
#endif

#ifdef PMSX

    // In the case (specifically LoRaWAN mode) in which the UART cannot be switched
    // to PMS even though sensor-defs.h "requested" it, don't even try.  We will
    // simply run with OPC.  This should never happen because of code in the sensor
    // scheduler that requests UART, but this is just defensive coding.
#if PMSX==IOUART
    if (gpio_current_uart() != UART_PMS) {
        if (debug(DBG_SENSOR))
            DEBUG_PRINTF("Air: PMS Init Failure because current UART is not PMS\n");
        return false;
    } else {
        if (s_pms_init(s, param))
            pms_active = true;
    }
#else
    if (s_pms_init(s, param))
        pms_active = true;
#endif

#endif

    // Done
    return true;

}

// Term sensor just before each power-off
bool s_air_term() {
#ifdef SPIOPC
    if (opc_active) {
        opc_active = false;
        s_opc_term();
    }
#endif
#ifdef PMSX
    if (pms_active) {
        pms_active = false;
        s_pms_term();
    }
#endif
    return true;
}

#endif // AIRX
