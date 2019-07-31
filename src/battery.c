// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

// Battery level handling

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "debug.h"
#include "config.h"
#include "battery.h"
#include "sensor.h"

// Battery level auto-adjustment logic (except when debugging, as indicated by BTKEEPALIVE)
#ifdef BATTERYDEBUG
#define BATTERY_AUTOADJUST false
#else
#if defined(TWIMAX17043) || defined(TWIMAX17201) || defined(TWIINA219)
#define BATTERY_AUTOADJUST true
#else
#define BATTERY_AUTOADJUST false
#endif
#endif

// Parameters defining high-power activities
#define BATTERY_SOC_HIGHPOWER_MIN           100.0
#define BATTERY_SOC_HIGHPOWER_MAX           110.0

// Statics
static float lastKnownBatterySOC = 0;
static bool batteryRecoveryMode = false;

// Default this to TRUE so that we charge up to MAX at boot before starting to draw down
static bool fullBatteryRecoveryMode = true;

// Set the last known SOC
void battery_set_soc_to_unknown() {
    lastKnownBatterySOC = 100.0;
}

// Set the last known SOC
void battery_set_soc(float SOC) {
    lastKnownBatterySOC = SOC;
}

// Get the last known SOC.  Note that this is an extremely low level routine, so
// don't modify it to call anything that might potentially go recursive.
float battery_soc() {

    if (lastKnownBatterySOC == 0)
        return 100.0;
    
    return (lastKnownBatterySOC);
}

// Get name of battery status, for debugging
char *battery_status_name() {
    switch (battery_status()) {
    case BAT_MOBILE:
        return "BAT_MOBILE";
    case BAT_FULL:
        return "BAT_FULL";
    case BAT_NORMAL:
        return "BAT_NORMAL";
    case BAT_LOW:
        return "BAT_LOW";
    case BAT_WARNING:
        return "BAT_WARNING";
    case BAT_EMERGENCY:
        return "BAT_EMERGENCY";
    case BAT_DEAD:
        return "BAT_DEAD";
    case BAT_TEST:
        return "BAT_TEST";
    case BAT_BURN:
        return "BAT_BURN";
    }
    return "BAT_ UNKNOWN";
}

// Returns a value based on the battery status.  Note that these values
// are defined in sensor.h as a bit mask, so they can be tested via "==" or switch
// OR via bitwise-&, as opposed to *needing* to do == or a switch statement.
// Note that this is an extremely low level routine, so
// don't modify it to call anything that might potentially go recursive.
uint16_t battery_status() {

    // Handle certain compile-time test modes
#if defined(BATTERY_FULL)
    return BAT_FULL;
#endif
#if defined(BATTERY_NORMAL)
    return BAT_NORMAL;
#endif
    
    // Exit if battery is truly dead, no matter what mode we are in
#if BATTERY_AUTOADJUST
    if (lastKnownBatterySOC != 0 && lastKnownBatterySOC < 10.0)
        return BAT_DEAD;
#endif
    
    // Special-case certain operating modes
    
    switch (sensor_op_mode()) {
    case OPMODE_TEST_BURN:
        return BAT_BURN;
    case OPMODE_TEST_FAST:
        return BAT_TEST;
    case OPMODE_MOBILE:
        return BAT_MOBILE;
    case OPMODE_TEST_DEAD:
        return BAT_NO_SENSORS;
    }

    // Exit if we can't auto-adjust
#if !BATTERY_AUTOADJUST
    return BAT_NORMAL;
#endif

    // If it's never yet been set, just treat as normal
    if (lastKnownBatterySOC == 0)
        return BAT_NORMAL;

    // Recovery mode
    if (batteryRecoveryMode) {
        if (lastKnownBatterySOC < 70.0)
            return BAT_EMERGENCY;
        batteryRecoveryMode = false;
        return BAT_NORMAL;
    } else {

        // Important note: Sadly, I learned the hard way that because of
        // the internal chemistry of LIPO batteries, they must NEVER be
        // allowed to discharge below 3.0V per cell or else they will
        // suffer internal damage. Copper shunts may form within the
        // cells that may cause an electrical short.  Therefore, this
        // code goes through extraordinary lengths to ensure that we
        // cease draining the battery when it gets low.

        if (lastKnownBatterySOC < 20.0) {
            batteryRecoveryMode = true;
            return BAT_EMERGENCY;
        }
    }

    // Danger mode
    if (lastKnownBatterySOC < 60.0)
        return BAT_LOW;

    // Danger mode
    if (lastKnownBatterySOC < 40.0)
        return BAT_WARNING;

    // Determine if this is a full battery, debouncing it between min & max
    if (lastKnownBatterySOC < BATTERY_SOC_HIGHPOWER_MIN) {
        fullBatteryRecoveryMode = true;
        return BAT_NORMAL;
    }
    if (fullBatteryRecoveryMode && lastKnownBatterySOC < BATTERY_SOC_HIGHPOWER_MAX)
        return BAT_NORMAL;
    fullBatteryRecoveryMode = false;
    return BAT_FULL;

}

// Compute a simulated SOC value based on voltage data

// Note that our goal is that 100% means "normal full", however
// we will try to actively do higher-power activities while modulating the
// charging to between HIGHPOWER_MIN-HIGHPOWER_MAX, while always doing high-power activities
// above HIGHPOWER_MAX under the assumption that this means we're plugged-in.
//
// Important note: Sadly, I learned the hard way that because of
// the internal chemistry of LIPO batteries, they must NEVER be
// allowed to discharge below 3.2V per cell or else they will
// suffer internal damage. Copper shunts may form within the
// cells that may cause an electrical short.
//
// http://batteryuniversity.com/learn/article/how_to_prolong_lithium_based_batteries
//
// Further, note that there is NO CODE in this project that explicitly tests
// for battery voltages.  Everything is based on SOC, so if the
// fuel gauge driver uses voltage to compute SOC, this code's behavior
// is super-critical to overall device performance.
float battery_soc_from_voltage(float voltage) {
#ifdef scv0
    float minV = 3.5;
    float maxV = 4.0;
#endif
#ifdef scv1    
    float minV = 3.5;
    float maxV = 3.9;
#endif
#ifdef scv2
    float minV = 3.5;
    float maxV = 4.0;
#endif
    float soc;
    float curV = voltage;
    if (curV < minV)
        curV = 0;
    else
        curV -= minV;
    maxV -= minV;
    soc = (curV * 100.0) / maxV; // Assume linear drain because of our device's behavior

    // Done
    return(soc);
}
