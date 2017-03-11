// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

// Sensor scheduler

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "debug.h"
#include "misc.h"
#include "config.h"
#include "io.h"
#include "twi.h"
#include "pms.h"
#include "opc.h"
#include "air.h"
#include "gpio.h"
#include "bme0.h"
#include "bme1.h"
#include "comm.h"
#include "send.h"
#include "ina.h"
#include "ugps.h"
#include "lis.h"
#include "geiger.h"
#include "sensor.h"
#include "storage.h"
#include "timer.h"
#include "nrf_delay.h"
#include "custom_board.h"

// Special handling for battery
static float lastKnownBatterySOC = 100.0;
static bool batteryRecoveryMode = false;
static bool fHammerMode = false;
#ifdef BATDEBUG
static bool fBatteryTestMode = true;
#else
static bool fBatteryTestMode = false;
#endif

// Default this to TRUE so that we charge up to MAX at boot before starting to draw down
static bool fullBatteryRecoveryMode = true;

// Mobile mode
static uint16_t MobileMode = MOBILE_OFF;

// Sensor test mode
static bool fTestModeRequested = false;
static bool fTestModeActive = false;

// Instantiate the static sensor state table definitions
#include "sensor-defs.h"

// Forward reference to init, which is called at first poll
static bool fInit = false;
void sensor_init();

// Set the last known SOC
void sensor_set_bat_soc(float SOC) {
    lastKnownBatterySOC = SOC;
}

// Get the last known SOC
float sensor_get_bat_soc() {
    return (lastKnownBatterySOC);
}

// Force "mobile mode"
bool sensor_set_mobile_mode(uint16_t mode) {
    if (comm_mode() != COMM_FONA) {
        DEBUG_PRINTF("Mobile mode only available when using FONA.\n");
        return false;
    }
    if (storage()->gps_latitude != 0 || storage()->gps_longitude != 0) {
        DEBUG_PRINTF("Mobile mode doesn't make sense with static GPS configuration.\n");
        return false;
    }
    MobileMode = mode;
    if (mode == MOBILE_ON) {
        // Tell the GPS module to improve its location
        s_ugps_update();
        // Accelerate enabling the mobile modules
        sensor_group_schedule_now("g-ugps");
        sensor_group_schedule_now("g-geiger");
    }
    return true;
}

// Skip handler for sensors that shouldn't ever be active if in mobile mode
bool g_mobile_skip(void *g) {
    return (MobileMode != 0);
}

// Sense to see if we should be behaving as though we are in mobile mode
uint16_t sensor_mobile_mode() {
    if (!comm_would_be_buffered())
        return MOBILE_OFF;
    return MobileMode;
}

// See if sensors indicate that we're in-motion
bool sensor_currently_in_motion() {
    if (MobileMode != 0)
        return false;
    return(gpio_motion_sense(MOTION_QUERY));
}

// Force "battery test mode" for testing
bool sensor_toggle_battery_test_mode() {
    fBatteryTestMode = !fBatteryTestMode;
    return fBatteryTestMode;
}

// Force "full sensor test mode" for testing
bool sensor_toggle_hammer_test_mode() {
    fHammerMode = !fHammerMode;
    return fHammerMode;
}

// Return true if any test mode is turned on except for battery test
// mode, during which we want communications to happen.
bool sensor_test_mode() {
    return(fHammerMode || fTestModeRequested || fTestModeActive);
}

// Get name of battery status, for debugging
char *sensor_get_battery_status_name() {
    switch (sensor_get_battery_status()) {
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
    }
    return "BAT_ UNKNOWN";
}

// Returns a value based on the battery status.  Note that these values
// are defined in sensor.h as a bit mask, so they can be tested via "==" or switch
// OR via bitwise-&, as opposed to *needing* to do == or a switch statement.
uint16_t sensor_get_battery_status() {

    if (fBatteryTestMode)
        return BAT_TEST;

    if (MobileMode)
        return BAT_MOBILE;

#if defined(BATTERY_FULL)
    return BAT_FULL;
#endif

#if defined(BATTERY_NORMAL) || (!BATTERY_AUTOADJUST)
    return BAT_NORMAL;
#endif

    // If it's never yet been set, just treat as normal
    if (lastKnownBatterySOC == 0)
        return BAT_NORMAL;

    // Exit if battery is dead
    if (lastKnownBatterySOC < 5.0)
        return BAT_DEAD;

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
    if (lastKnownBatterySOC < SOC_HIGHPOWER_MIN) {
        fullBatteryRecoveryMode = true;
        return BAT_NORMAL;
    }
    if (fullBatteryRecoveryMode && lastKnownBatterySOC < SOC_HIGHPOWER_MAX)
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
float sensor_compute_soc_from_voltage(float voltage) {
    float soc;
    float minV = 3.5;
    float maxV = 4.0;
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

// Mark a sensor which is being processed as being completed
void sensor_measurement_completed(sensor_t *s) {
    // Note that we support calling of sensor routines directly in absence
    // of the sensor packaging having been involved, in which case this
    // will be null
    if (s == NULL)
        return;
    s->state.is_completed = true;
    s->state.is_polling_valid = false;
    if (debug(DBG_SENSOR))
        DEBUG_PRINTF("%s is measured.\n", s->name);
}

// Mark a sensor as being permanently unconfigured because of an error
void sensor_unconfigure(sensor_t *s) {
    // Note that we support calling of sensor routines directly in absence
    // of the sensor packaging having been involved, in which case this
    // will be null
    if (s == NULL)
        return;
    if (s->term_power != NO_HANDLER)
        s->term_power();
    s->state.is_processing = false;
    s->state.is_completed = true;
    s->state.is_polling_valid = false;
    s->state.is_configured = false;
    if (debug(DBG_SENSOR))
        DEBUG_PRINTF("%s unconfigured\n", s->name);
}

// Determine whether or not polling is valid right now
bool sensor_group_is_polling_valid(group_t *g) {
    if (debug(DBG_SENSOR_SUPERDUPERMAX)) {
        if (!g->state.is_polling_valid)
            DEBUG_PRINTF("%s spurious poll ignored\n", g->name);
        else
            DEBUG_PRINTF("%s poll\n", g->name);
    }
    return(g->state.is_polling_valid);
}

// Determine whether or not polling is valid right now
bool sensor_is_polling_valid(sensor_t *s) {
    if (debug(DBG_SENSOR_SUPERDUPERMAX)) {
        if (!s->state.is_polling_valid)
            DEBUG_PRINTF("%s spurious poll ignored\n", s->name);
        else
            DEBUG_PRINTF("%s poll\n", s->name);
    }
    return(s->state.is_polling_valid);
}

// Look up a sensor group by name
void *sensor_group_name(char *name) {
    group_t **gp, *g;
    for (gp = &sensor_groups[0]; (g = *gp) != END_OF_LIST; gp++) {
        if (strcmp(g->name, name) == 0)
            return g;
    }
    return NULL;
}

// Return true if this sensor is being tested
bool sensor_is_being_tested(sensor_t *s) {
    return(fTestModeActive && s->state.is_being_tested);
}

// Look up a sensor group by name
void sensor_test(char *name) {
    group_t **gp, *g;
    sensor_t **sp, *s;
    fTestModeActive = false;
    fTestModeRequested = false;
    for (gp = &sensor_groups[0]; (g = *gp) != END_OF_LIST; gp++) {
        g->state.is_being_tested = false;
        for (sp = &g->sensors[0]; (s = *sp) != END_OF_LIST; sp++) {
            if (strcmp(s->name, name) != 0)
                s->state.is_being_tested = false;
            else {
                g->state.is_being_tested = true;
                s->state.is_being_tested = true;
                DEBUG_PRINTF("Sensor Test Mode requested for %s %s\n", g->name, s->name);
                fTestModeRequested = true;
            }
        }
    }
    if (!fTestModeRequested) {
        if (name[0] == '\0')
            DEBUG_PRINTF("Sensor Test Mode now disabled\n");
        else
            DEBUG_PRINTF("Sensor not found\n");
    }
}

// Mark a sensor group as needing to be measured NOW
bool sensor_group_schedule_now(char *gname) {
    group_t *g = sensor_group_name(gname);
    if (g == NULL)
        return false;
    g->state.last_repeated = 0;
    return true;
}

// Mark all sensors within an entire group as having been completed
bool sensor_group_completed(group_t *g) {
    sensor_t **sp, *s;
    bool somethingCompleted = false;
    // Note that we support calling of sensor routines directly in absence
    // of the sensor packaging having been involved, in which case this
    // will be null
    if (g == NULL)
        return false;
    // Turn off polling
    g->state.is_polling_valid = false;
    // Mark all sensors as completed
    for (sp = &g->sensors[0]; (s = *sp) != END_OF_LIST; sp++)
        if (s->state.is_configured && !s->state.is_completed) {
            s->state.is_completed = true;
            s->state.is_polling_valid = false;
            somethingCompleted = true;
        }
    if (somethingCompleted && debug(DBG_SENSOR))
        DEBUG_PRINTF("%s is completed.\n", g->name);
    return (somethingCompleted);
}

// Mark all sensors within an entire group as having been unconfigured
void sensor_group_unconfigure(group_t *g, uint32_t err_code) {
    sensor_t **sp, *s;
    // Note that we support calling of sensor routines directly in absence
    // of the sensor packaging having been involved, in which case this
    // will be null
    if (g == NULL)
        return;
    for (sp = &g->sensors[0]; (s = *sp) != END_OF_LIST; sp++)
        if (s->state.is_configured) {
            s->state.is_completed = true;
            s->state.is_polling_valid = false;
            s->state.is_configured = false;
        }
    g->state.is_configured = false;
    if (debug(DBG_SENSOR))
        DEBUG_PRINTF("%s unconfigured (0x%04x)\n", g->name, err_code);
}

// Determine if group is powered on
bool sensor_group_powered_on(group_t *g) {
    return(g->state.is_powered_on);
}

// Test to see if any sensor is powered on
bool sensor_group_any_exclusive_powered_on() {
    group_t **gp, *g;
    if (!fInit)
        return false;
    if (fHammerMode || fTestModeActive)
        return false;
    for (gp = &sensor_groups[0]; (g = *gp) != END_OF_LIST; gp++) {
        if (g->state.is_configured && g->power_set != NO_HANDLER && g->power_exclusive && g->state.is_powered_on)
            return true;
    }
    return false;
}

// Test to see if anything in any group has already been measured
bool sensor_any_upload_needed() {
    group_t **gp, *g;
    sensor_t **sp, *s;

    if (!fInit)
        return false;
    if (fHammerMode || fTestModeActive)
        return false;

    for (gp = &sensor_groups[0]; (g = *gp) != END_OF_LIST; gp++) {
        if (g->state.is_configured) {
            for (sp = &g->sensors[0]; (s = *sp) != END_OF_LIST; sp++)
                if (s->state.is_configured && s->upload_needed != NO_HANDLER)
                    if (s->upload_needed(s)) {
                        if (sensor_currently_in_motion()) {
                            if (debug(DBG_SENSOR))
                                DEBUG_PRINTF("SENSOR: upload pending, but device is currently in-motion\n");
                            return false;
                        }
                        return true;
                    }
        }
    }
    if (debug(DBG_SENSOR_SUPERDUPERMAX))
        DEBUG_PRINTF("No sensors have pending measurements.\n");
    return false;
}

// Get a group's repeat minutes, adjusted for debugging
uint16_t group_repeat_minutes(group_t *g) {
    uint16_t battery_status = sensor_get_battery_status();
    uint16_t repeat_minutes = 0;
    repeat_t *r;

    // If overridden, use it
    if (g->state.repeat_minutes_override != 0) {
        if (debug(DBG_SENSOR_SUPERDUPERMAX))
            DEBUG_PRINTF("%s repeat overriden with %dm\n", g->name, g->state.repeat_minutes_override);
        return g->state.repeat_minutes_override;
    }

    // Loop, finding the appropriate battery status
    for (r = g->repeat;; r++) {
        if ((battery_status & r->active_battery_status) != 0) {
            repeat_minutes = r->repeat_minutes;
            break;
        }
    }

    // Bug check
    if (repeat_minutes == 0) {
        while (true) {
            DEBUG_PRINTF("%s repeat minutes not found for %s\n", g->name, sensor_get_battery_status_name());
            nrf_delay_ms(MAX_NRF_DELAY_MS);
        }
    }

    // If we're testing, just double it
    if (battery_status == BAT_TEST)
        return (repeat_minutes/2);

    // Debug
    if (debug(DBG_SENSOR_SUPERDUPERMAX))
        DEBUG_PRINTF("%s repeat for %s is %dm\n", g->name, sensor_get_battery_status_name(), repeat_minutes);

    return(repeat_minutes);
}

// Show the entire sensor state
void sensor_show_state() {
    group_t **gp, *g;
    sensor_t **sp, *s;
    uint32_t seconds_since_boot = get_seconds_since_boot();

    if (!fInit) {
        DEBUG_PRINTF("Not yet initialized.\n");
        return;
    }

    DEBUG_PRINTF("Battery:%d Motion:%d UART:%d\n", sensor_get_battery_status(), sensor_currently_in_motion(), gpio_current_uart());

    for (gp = &sensor_groups[0]; (g = *gp) != END_OF_LIST; gp++) {
        if (g->state.is_configured && ((sensor_get_battery_status() & g->active_battery_status) != 0)) {
            if (g->skip_handler != NO_HANDLER && !fTestModeActive)
                if (g->skip_handler(g))
                    continue;
            bool fOverdue = false;
            int nextsecs = (group_repeat_minutes(g)*60) - (seconds_since_boot-g->state.last_repeated);
            if (nextsecs < 0) {
                nextsecs = -nextsecs;
                fOverdue = true;
            }
            int nextmin = nextsecs/60;
            nextsecs -= nextmin*60;
            char buff[128];
            if (fOverdue)
                sprintf(buff, "is overdue to resample by %dm%ds", nextmin, nextsecs);
            else if (g->state.last_repeated == 0)
                sprintf(buff, "next up");
            else
                sprintf(buff, "next up %dm%ds", nextmin, nextsecs);
            if (g->power_exclusive && sensor_group_any_exclusive_powered_on())
                strcat(buff, " when power avail");
            if (g->uart_required != UART_NONE && gpio_current_uart() != UART_NONE)
                strcat(buff, " when UART avail");
            if (comm_uart_switching_allowed() && g->uart_requested != UART_NONE && gpio_current_uart() != UART_NONE)
                strcat(buff, " when UART avail");
            if (g->state.is_being_tested)
                strcat(buff, " (being tested)");
            DEBUG_PRINTF("%s %s\n", g->name, g->state.is_processing ? (g->state.is_settling ? "now settling" : "now sampling") : buff);

            for (sp = &g->sensors[0]; (s = *sp) != END_OF_LIST; sp++) {
                if (s->state.is_configured) {
                    bool fUploadNeeded = false;
                    if (s->upload_needed != NO_HANDLER)
                        fUploadNeeded = s->upload_needed(s);
                    char buff[40];
                    if (s->state.is_processing)
                        sprintf(buff, "%s for %ds", g->state.is_settling ? "now settling" : "now sampling", (int) (seconds_since_boot - g->state.last_settled));
                    else
                        strcpy(buff, "waiting");
                    if (s->state.is_being_tested)
                        strcat(buff, " (being tested)");
                    if (s->state.is_processing || s->state.is_being_tested || fUploadNeeded)
                        DEBUG_PRINTF("   %s %s%s\n", s->name, s->state.is_completed ? "completed" : buff, fUploadNeeded ? ", waiting to upload" : "");
                }
            }

        }

    }

}

// Standard power on/off handler
void sensor_set_pin_state(uint16_t pin, bool init, bool enable) {
    if (pin != SENSOR_PIN_UNDEFINED) {
        if (init)
            gpio_power_init(pin, enable);
        else
            gpio_power_set(pin, enable);
    }
}

// Poll, advancing the state machine
void sensor_poll() {
    static int inside_poll = 0;
    bool groups_currently_active;
    int pending, configured_sensors;
    group_t **gp, *g;
    sensor_t **sp, *s;

    // Exit if we haven't yet initialized GPS, which is a big signal that we're not yet ready to proceed,
    // except for the case of UGPS when we need sensor processing to acquire GPS
#ifndef UGPS
    uint16_t status = comm_gps_get_value(NULL, NULL, NULL);
    if (status != GPS_NOT_CONFIGURED)
        if (status != GPS_LOCATION_FULL && status != GPS_LOCATION_PARTIAL)
            return;
#endif

    // Initialize if we haven't yet done so
    if (!fInit) {
        sensor_init();
        fInit = true;
    }

    // Debug TWI
#ifdef TWIX
    if (debug(DBG_SENSOR_SUPERDUPERMAX))
        twi_status_check(false);
#endif

    // Exit if we're already inside the poller.  This DOES happen if one of the handlers (such as an
    // init handler) takes an incredibly long time because of, say, a retry loop.
    if (inside_poll++ != 0) {
        inside_poll--;
        return;
    }

    if (debug(DBG_SENSOR_SUPERDUPERMAX))
        DEBUG_PRINTF("sensor_poll enter\n");

    // Loop over all configured sensors in all configured groups
    groups_currently_active = 0;

    for (gp = &sensor_groups[0]; (g = *gp) != END_OF_LIST; gp++) {

        // If not configured, skip this group
        if (!g->state.is_configured)
            continue;

        // If test mode and not being tested, bail
        if (fTestModeActive && !g->state.is_being_tested)
            continue;

        // Are we completely idle?
        if (!g->state.is_processing && !g->state.is_settling) {

            if (debug(DBG_SENSOR_SUPERDUPERMAX))
                DEBUG_PRINTF("%s !processing !settling\n", g->name);

            // If we've requested test mode, don't schedule anything new
            if (fTestModeRequested)
                continue;

            // Skip if this group doesn't need to be processed right now
            if (g->skip_handler != NO_HANDLER && !fTestModeActive)
                if (g->skip_handler(g)) {
                    if (debug(DBG_SENSOR_SUPERDUPERMAX))
                        DEBUG_PRINTF("Skipping %s at its request.\n", g->name);
                    continue;
                }

            // If ALL of the sensors in this group already have pending measurements,
            // skip the group because it's senseless to keep measuring.
            bool fSkipGroup = true;
            int Sensors = 0;
            for (sp = &(*gp)->sensors[0]; (s = *sp) != END_OF_LIST; sp++) {
                if (s->state.is_configured) {
                    Sensors++;
                    if (s->upload_needed == NO_HANDLER || !s->upload_needed(s)) {
                        fSkipGroup = false;
                        break;
                    }
                }
            }
            if (Sensors == 0)
                g->state.is_configured = false;
            if (fSkipGroup && !fHammerMode && !fTestModeActive) {
                if (debug(DBG_SENSOR_SUPERDUPERMAX)) {
                    if (Sensors == 0)
                        DEBUG_PRINTF("Skipping %s because no sensors are found.\n", g->name);
                    else
                        DEBUG_PRINTF("Skipping %s because all its sensors' uploads are pending.\n", g->name);
                }
                continue;
            }

            // If this sensor group is a particular power hog and needs to be run
            // only when other exclusives aren't powered on, skip the group if anyone else
            // is currently powered on.
            if (g->power_exclusive && sensor_group_any_exclusive_powered_on()) {
                if (debug(DBG_SENSOR_SUPERDUPERMAX))
                    DEBUG_PRINTF("Skipping %s because something else is powered on.\n", g->name);
                continue;
            }

            // If this sensor group requires a uart, but the UART is busy, skip the group
            if (g->uart_required != UART_NONE && gpio_current_uart() != UART_NONE) {
                if (debug(DBG_SENSOR_SUPERDUPERMAX))
                    DEBUG_PRINTF("Skipping %s because the required UART is busy.\n", g->name);
                continue;
            }

            // If this sensor group requests a uart (but is allowed to run without it if
            // it CAN'T be granted), but the UART is busy, skip the group
            if (comm_uart_switching_allowed() && g->uart_requested != UART_NONE && gpio_current_uart() != UART_NONE) {
                if (debug(DBG_SENSOR_SUPERDUPERMAX))
                    DEBUG_PRINTF("Skipping %s because the requested UART is busy.\n", g->name);
                continue;
            }

            // If we're not in the right battery status, skip it
            if ((sensor_get_battery_status() & g->active_battery_status) == 0) {
                if (debug(DBG_SENSOR_SUPERDUPERMAX))
                    DEBUG_PRINTF("Skipping %s because 0x%04x doesn't map to %s\n", g->name, g->active_battery_status, sensor_get_battery_status_name());
                continue;
            }

            // If we're not in the right comms mode, skip it
            if ((comm_mode() & g->active_comm_mode) == 0) {
                if (debug(DBG_SENSOR_SUPERDUPERMAX))
                    DEBUG_PRINTF("Skipping %s because %04x doesn't map to active comm mode (%04x).\n", g->name, g->active_comm_mode, comm_mode());
                continue;
            }

            // If we're in the repeat idle period for this group, just go to next group
            if (!fHammerMode && !fTestModeActive)
                if (ShouldSuppressConsistently(&g->state.last_repeated, group_repeat_minutes(g)*60))
                    continue;

            // Initialize sensor state and refresh configuration state
            configured_sensors = 0;

            for (sp = &(*gp)->sensors[0]; (s = *sp) != END_OF_LIST; sp++) {
                if (s->state.is_configured) {

                    s->state.is_settling = false;
                    s->state.is_processing = false;
                    s->state.is_completed = false;

                    // Deconfigure sensors that are encountering too many SEQUENTIAL errors
                    if (s->state.init_failures > 3 || s->state.term_failures > 3) {
                        if (debug(DBG_SENSOR))
                            DEBUG_PRINTF("DECONFIGURING %s: %d init errors, %d term errors\n", s->name, s->state.init_failures, s->state.term_failures);
                        s->state.is_configured = false;
                    } else {
                        configured_sensors++;
                    }
                }
            }

            if (configured_sensors == 0) {
                g->state.is_configured = false;
                continue;
            }

            // Begin processing
            g->state.is_processing = true;

            if (debug(DBG_SENSOR_SUPERDUPERMAX))
                DEBUG_PRINTF("%s processing\n", g->name);

            // Power ON the module
            if (g->power_set != NO_HANDLER) {
                g->power_set(g->power_set_parameter, false, true);
                g->state.is_powered_on = true;
                // Delay a bit before proceeding to do anything at all
                nrf_delay_ms(MAX_NRF_DELAY_MS);
                if (debug(DBG_SENSOR|DBG_SENSOR_MAX))
                    DEBUG_PRINTF("%s power pin #%d ON\n", g->name, g->power_set_parameter);
            }

            // Select the UART if one is required or requested
            if (g->uart_required != UART_NONE)
                gpio_uart_select(g->uart_required);
            if (comm_uart_switching_allowed() && g->uart_requested != UART_NONE)
                gpio_uart_select(g->uart_requested);

            // Call the sensor power-on init functions
            for (sp = &g->sensors[0]; (s = *sp) != END_OF_LIST; sp++) {

                // If not configured, don't bother initializing anything else
                if (!s->state.is_configured)
                    continue;

                // If there's an init handler to be called after power is turned on, call it
                if (s->init_power != NO_HANDLER) {
                    if (!s->init_power(s->init_parameter))
                        s->state.init_failures++;
                    else
                        s->state.init_failures = 0;
                }

            }

            // Begin the settling period
            g->state.last_settled = get_seconds_since_boot();
            g->state.is_settling = true;

            if (debug(DBG_SENSOR_SUPERDUPERMAX))
                DEBUG_PRINTF("%s settling\n", g->name);

            if (g->settling_seconds != 0 && debug(DBG_SENSOR))
                DEBUG_PRINTF("Begin %s settling for %ds\n", g->name, g->settling_seconds);

            // Start the group app timer when the power is applied, because a core part of the
            // settling period for certain TWI devices (i.e. GPS) is that you need to
            // "warm them up" by pulling data out of them on a continuous basis.
            if (g->poll_handler != NO_HANDLER && !g->poll_continuously && g->poll_during_settling) {
                if (debug(DBG_SENSOR))
                    DEBUG_PRINTF("Starting %s timer\n", g->name);
                app_timer_start(g->state.group_timer.timer_id, APP_TIMER_TICKS(g->poll_repeat_milliseconds, APP_TIMER_PRESCALER), g);
                // Release the poller so that it's ok to proceed
                g->state.is_polling_valid = true;
            }

            // Do the same for sensor timers
            for (sp = &(*gp)->sensors[0]; (s = *sp) != END_OF_LIST; sp++) {

                // Skip unconfigured sensors
                if (!s->state.is_configured)
                    continue;

                // Enable the poller
                if (s->poll_handler != NO_HANDLER && !s->poll_continuously && s->poll_during_settling) {
                    if (debug(DBG_SENSOR))
                        DEBUG_PRINTF("Starting %s timer\n", s->name);
                    app_timer_start(s->state.sensor_timer.timer_id, APP_TIMER_TICKS(s->poll_repeat_milliseconds, APP_TIMER_PRESCALER), s);
                    // Release the poller so that it's ok to proceed
                    s->state.is_polling_valid = true;
                }

            }

        }

        // Are we in the settling period?
        if (g->state.is_processing && g->state.is_settling) {
            groups_currently_active++;

            if (debug(DBG_SENSOR_SUPERDUPERMAX))
                DEBUG_PRINTF("%s processing settling\n", g->name);

            // If we're in the settling idle period for this group, just go to the next group
            if (g->settling_seconds != 0)
                if (ShouldSuppress(&g->state.last_settled, g->settling_seconds))
                    continue;

            // Stop the settling period.
            g->state.is_settling = false;

            // If there's a handler to be called after group settling, call it
            if (g->done_settling != NO_HANDLER)
                g->done_settling();
            for (sp = &(*gp)->sensors[0]; (s = *sp) != END_OF_LIST; sp++)
                if (s->state.is_configured && s->done_group_settling != NO_HANDLER)
                    s->done_group_settling();

            // Start the app timer when settling is over, if that's what was requested
            if (g->poll_handler != NO_HANDLER && !g->poll_continuously && !g->poll_during_settling) {
                if (debug(DBG_SENSOR))
                    DEBUG_PRINTF("Starting %s timer\n", g->name);
                app_timer_start(g->state.group_timer.timer_id, APP_TIMER_TICKS(g->poll_repeat_milliseconds, APP_TIMER_PRESCALER), g);
                // Release the poller so that it's ok to proceed
                g->state.is_polling_valid = true;
            }

            // Do the same for sensor timers
            for (sp = &(*gp)->sensors[0]; (s = *sp) != END_OF_LIST; sp++) {

                // Skip unconfigured sensors
                if (!s->state.is_configured)
                    continue;

                // Enable the poller
                if (s->poll_handler != NO_HANDLER && !s->poll_continuously && !s->poll_during_settling) {
                    if (debug(DBG_SENSOR))
                        DEBUG_PRINTF("Starting %s timer\n", s->name);
                    app_timer_start(s->state.sensor_timer.timer_id, APP_TIMER_TICKS(s->poll_repeat_milliseconds, APP_TIMER_PRESCALER), s);
                    // Release the poller so that it's ok to proceed
                    s->state.is_polling_valid = true;
                }

            }

        }

        // Is it time to do some processing?
        if (g->state.is_processing && !g->state.is_settling) {
            groups_currently_active++;

            if (debug(DBG_SENSOR_SUPERDUPERMAX))
                DEBUG_PRINTF("%s processing !settling\n", g->name);

            // Loop over all sensors in this group, looking for work to do
            for (sp = &(*gp)->sensors[0]; (s = *sp) != END_OF_LIST; sp++) {

                // Skip unconfigured sensors
                if (!s->state.is_configured)
                    continue;

                // If test mode and not being tested, bail
                if (fTestModeActive && !s->state.is_being_tested)
                    continue;

                // Is this a candidate for initiating work?
                if (!s->state.is_processing && !s->state.is_completed) {

                    // Begin processing
                    s->state.is_processing = true;

                    if (s->state.is_being_tested)
                        DEBUG_PRINTF("Now testing %s\n", s->name);

                    // Begin the settling period
                    s->state.last_settled = get_seconds_since_boot();
                    s->state.is_settling = true;

                    if (s->settling_seconds != 0 && debug(DBG_SENSOR))
                        DEBUG_PRINTF("Begin %s %s for %ds\n", s->name, g->poll_handler == NO_HANDLER ? "settling" : "sampling", s->settling_seconds);

                }

                // Are we in the settling period?
                if (s->state.is_processing && s->state.is_settling) {

                    // If we're in the settling idle period for this sensor, stop processing sensors
                    if (s->settling_seconds != 0)
                        if (ShouldSuppress(&s->state.last_settled, s->settling_seconds))
                            break;

                    // Stop the settling period.
                    s->state.is_settling = false;

                    // If there's a handler to be called after settling, call it
                    if (s->done_settling != NO_HANDLER)
                        s->done_settling();

                    if (s->measure != NO_HANDLER && debug(DBG_SENSOR))
                        DEBUG_PRINTF("Measuring %s\n", s->name);

                    // Initiate the measurement.  We'll advance to the next state
                    // when this method, or some deep callback, calls sensor_completed(s)
                    if (s->measure != NO_HANDLER)
                        s->measure(s);

                }

                // Is this one processing?  If so, we don't want to move beyond it
                if (s->state.is_processing && !s->state.is_completed)
                    break;

            } // loop over sensors


        } // if work to do for a given sensor group

        // Count the number of sensors with work left to do
        for (pending = 0, sp = &(*gp)->sensors[0]; (s = *sp) != END_OF_LIST; sp++)
            if (s->state.is_configured)
                if (!s->state.is_completed)
                    if (!fTestModeActive || s->state.is_being_tested)
                        pending++;

        if (debug(DBG_SENSOR_SUPERDUPERMAX))
            DEBUG_PRINTF("%s still not completed\n", g->name);

        // If none of the sensors has any work pending, we're done with this group.
        if (pending == 0) {

            // Stop the app timer if one had been requested
            if (g->poll_handler != NO_HANDLER && !g->poll_continuously) {
                app_timer_stop(g->state.group_timer.timer_id);
                if (debug(DBG_SENSOR))
                    DEBUG_PRINTF("Stopped %s timer\n", g->name);
                // Stop the poller so that it doesn't do any more processing
                g->state.is_polling_valid = false;
            }

            for (sp = &g->sensors[0]; (s = *sp) != END_OF_LIST; sp++) {

                // If not configured, don't bother initializing anything else
                if (!s->state.is_configured)
                    continue;

                // Do the same for sensor pollers
                if (s->poll_handler != NO_HANDLER && !s->poll_continuously) {
                    app_timer_stop(s->state.sensor_timer.timer_id);
                    if (debug(DBG_SENSOR))
                        DEBUG_PRINTF("Stopped %s timer\n", s->name);
                    // Stop the poller so that it doesn't do any more processing
                    s->state.is_polling_valid = false;
                }

            }

            // Call the sensor power-off preparation functions
            for (sp = &g->sensors[0]; (s = *sp) != END_OF_LIST; sp++) {

                // If not configured, don't bother initializing anything else
                if (!s->state.is_configured)
                    continue;

                // If there's a term handler to be called before power is turned off, call it
                if (s->term_power != NO_HANDLER) {
                    if (!s->term_power())
                        s->state.term_failures++;
                    else
                        s->state.term_failures = 0;
                }

            }

            // Deselect the UART if one was selected
            if (g->uart_required != UART_NONE)
                gpio_uart_select(UART_NONE);
            if (comm_uart_switching_allowed() && g->uart_requested != UART_NONE)
                gpio_uart_select(UART_NONE);

            // Power OFF the module
            if (g->power_set != NO_HANDLER) {
                g->power_set(g->power_set_parameter, false, false);
                g->state.is_powered_on = false;
                if (debug(DBG_SENSOR|DBG_SENSOR_MAX))
                    DEBUG_PRINTF("%s power pin #%d OFF\n", g->name, g->power_set_parameter);
            }

            // Clear our own state, setting us to idle.
            g->state.is_processing = false;

            if (debug(DBG_SENSOR_SUPERDUPERMAX))
                DEBUG_PRINTF("%s !processing\n", g->name);

            if (debug(DBG_SENSOR))
                DEBUG_PRINTF("%s completed\n", g->name);

        }

    } // Looping across groups

    // If no groups are currently active and test mode was requested, we
    // can now enter it.
    if (fTestModeRequested) {
        if (groups_currently_active == 0) {
            fTestModeRequested = false;
            fTestModeActive = true;
            DEBUG_PRINTF("Sensor Test Mode now active\n");
        } else {
            DEBUG_PRINTF("Sensor Test Mode waiting for %d sensors to complete\n", groups_currently_active);
        }
    }

    // Done

    if (debug(DBG_SENSOR_SUPERDUPERMAX))
        DEBUG_PRINTF("sensor_poll exit\n");
    inside_poll--;

    return;

}

// Initialize
void sensor_init() {
    group_t **gp, *g;
    sensor_t **sp, *s;
    STORAGE *c = storage();
    uint32_t init_time = get_seconds_since_boot();

    // Loop over all sensors in all sensor groups
    for (gp = &sensor_groups[0]; (g = *gp) != END_OF_LIST; gp++) {

        // If not configured, don't bother initializing anything else
        g->state.is_configured = (g->storage_product == c->product);
        if (!g->state.is_configured)
            continue;

        // Modify the sensor parameters to reflect what's in the storage parameters
        g->state.repeat_minutes_override = 0;
        char *psp, *pgn;
        psp = c->sensor_params;
        while (true) {
            // Exit if nothing left to parse in sensor parameters
            if (*psp == '\0')
                break;
            // Compare what we're sitting at with the current group name
            pgn = g->name;
            while (true) {
                if (*psp == '\0' || *pgn == '\0' || *psp != *pgn)
                    break;
                psp++;
                pgn++;

            }
            // If we fully recognize the group name, process it
            if (*pgn == '\0' && *psp == '.') {
                // See if it's a subfield that we recognize
#define repeat_field ".r="
                if (memcmp(psp, repeat_field, sizeof(repeat_field)) == 0) {
                    psp += sizeof(repeat_field);
                    uint16_t v = (uint16_t) strtol(psp, &psp, 0);
                    if (debug(DBG_SENSOR))
                        DEBUG_PRINTF("%s override repeat_minutes with %d\n", g->name, v);
                    g->state.repeat_minutes_override = (uint16_t) v;
                }
            }
            // Skip to the next psp parameter
            while (*psp != '\0')
                if (*psp++ == '/')
                    break;
        }

        // Init the state of the sensor group
        g->state.is_settling = false;
        g->state.is_processing = false;
        g->state.is_polling_valid = false;

        // If it's to be sensed immediately, do it, else delay one poll interval
        if (g->sense_at_boot)
            g->state.last_repeated = 0;
        else
            g->state.last_repeated = init_time;

        // Power OFF the module as its initial state
        if (g->power_set == NO_HANDLER)
            g->state.is_powered_on = true;
        else {
            g->power_set(g->power_set_parameter, true, false);
            g->state.is_powered_on = false;
            if (debug(DBG_SENSOR))
                DEBUG_PRINTF("%s power pin #%d OFF\n", g->name, g->power_set_parameter);
        }

        // If this group requested an app timer, create it
        if (g->poll_handler != NO_HANDLER) {
            memset(&g->state.group_timer.timer_data, 0, sizeof(g->state.group_timer.timer_data));
            g->state.group_timer.timer_id = &g->state.group_timer.timer_data;
            app_timer_create(&g->state.group_timer.timer_id, APP_TIMER_MODE_REPEATED, g->poll_handler);
            // Start it at init if we're polling continuously
            if (g->poll_continuously) {
                app_timer_start(g->state.group_timer.timer_id, APP_TIMER_TICKS(g->poll_repeat_milliseconds, APP_TIMER_PRESCALER), g);
                // Release the poller so that it's ok to proceed
                g->state.is_polling_valid = true;
            }
            // The code below is defensive programming because there's a known-buggy behavior in the
            // nRF handling of app timers, such that if we stop the timer before a single tick has happened,
            // the stop fails to "take". So we ensure that the settling period is at a minimum of the timer period plus slop.
            uint32_t min_settling_seconds = (g->poll_repeat_milliseconds / 1000) + 5;
            if (g->settling_seconds < min_settling_seconds)
                g->settling_seconds = min_settling_seconds;
        }

        // Loop over all sensors in the group
        uint16_t configured_sensors = 0;
        for (sp = &(*gp)->sensors[0]; (s = *sp) != END_OF_LIST; sp++) {

            // If not configured, don't bother initializing anything else
            s->state.is_configured = ((s->storage_sensor_mask & c->sensors) != 0);
            if (!s->state.is_configured)
                continue;
            configured_sensors++;

            // Initialize sensor state
            s->state.is_settling = false;
            s->state.is_processing = false;
            s->state.is_completed = false;
            s->state.init_failures = 0;
            s->state.term_failures = 0;
            s->state.is_polling_valid = false;

            // If this sensor requested an app timer, create it
            if (s->poll_handler != NO_HANDLER) {
                memset(&s->state.sensor_timer.timer_data, 0, sizeof(s->state.sensor_timer.timer_data));
                s->state.sensor_timer.timer_id = &s->state.sensor_timer.timer_data;
                app_timer_create(&s->state.sensor_timer.timer_id, APP_TIMER_MODE_REPEATED, s->poll_handler);
                // Start it at init if we're polling continuously
                if (s->poll_continuously) {
                    app_timer_start(s->state.sensor_timer.timer_id, APP_TIMER_TICKS(s->poll_repeat_milliseconds, APP_TIMER_PRESCALER), s);
                    // Release the poller so that it's ok to proceed
                    s->state.is_polling_valid = true;
                }
                // The code below is defensive programming because there's a known-buggy behavior in the
                // nRF handling of app timers, such that if we stop the timer before a single tick has happened,
                // the stop fails to "take". So we ensure that the settling period is at a minimum of the timer period plus slop.
                uint32_t min_settling_seconds = (s->poll_repeat_milliseconds / 1000) + 5;
                if (s->settling_seconds < min_settling_seconds)
                    s->settling_seconds = min_settling_seconds;
            }

            // If there's an init handler, call it
            if (s->init_once != NO_HANDLER) {
                if (!s->init_once(s->init_parameter))
                    s->state.init_failures++;
                else
                    s->state.init_failures = 0;
            }
        }

        // Deconfigure the group if there are no configured sensors
        if (configured_sensors == 0)
            g->state.is_configured = false;


    }

}
