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
#include "spi.h"
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
#include "battery.h"

#define GPS_SENSOR_GROUP "g-ugps"

// Statics
#ifdef BURN
static uint16_t operating_mode = OPMODE_TEST_BURN;
#elif defined(FAST)
static uint16_t operating_mode = OPMODE_TEST_FAST;
#else
static uint16_t operating_mode = OPMODE_NORMAL;
#endif
static uint16_t mobile_period = 0;
static uint32_t mobile_session = 0;

// Temporary operating mode
static uint16_t temporary_op_mode;
static uint32_t temporary_op_mode_set_at;
static uint32_t temporary_op_mode_duration_seconds = 0L;

// Sensor test mode
static bool fTestModeRequested = false;

// Instantiate the static sensor state table definitions
#include "sensor-defs.h"

// Forward reference to init, which is called at first poll
static bool fInit = false;
void sensor_init();

// Get the time suppression
uint16_t sensor_get_mobile_upload_period() {
    return mobile_period;
}

// Set the mobile time suppression
void sensor_set_mobile_upload_period(uint16_t seconds) {
    mobile_period = seconds;
    if (mobile_period == 0)
        DEBUG_PRINTF("Mobile upload period set to maximum rate.\n");
    else
        DEBUG_PRINTF("Mobile upload period set to %d seconds.\n", mobile_period);
}

// Get the mobile session ID, which is reset at device boot
uint32_t sensor_get_mobile_session_id() {
    return mobile_session;
}

// Set the operating mode
bool sensor_set_op_mode(uint16_t op_mode) {

    // If we're setting it back to normal, but we're in BURN mode, just interpret it as returning to burn
#ifdef BURN
    if (op_mode == OPMODE_NORMAL)
        op_mode = OPMODE_TEST_BURN;
#endif

    // Do special work if we're switching into mobile mode
    if (op_mode == OPMODE_MOBILE && operating_mode != op_mode) {

        if (storage()->gps_latitude != 0 || storage()->gps_longitude != 0) {
            DEBUG_PRINTF("Mobile mode doesn't make sense with static GPS configuration.\n");
            return false;
        }

        // Tell others that we are beginning a new "drive" session
        mobile_session++;

        // Tell the GPS module to improve its location
        comm_gps_update();

        // Accelerate enabling the mobile modules
        sensor_group_schedule_now(GPS_SENSOR_GROUP);

        // Initiate a service update, so that we send a new stamp value
        // to the service so as to initiate a new "drive"
        comm_initiate_service_update(true);

    }

    // Set the mode
    operating_mode = op_mode;
    return true;

}

// Set temporary mode for N seconds, or turn it off with seconds == 0
void sensor_set_temporary_op_mode(uint16_t op_mode, uint32_t seconds) {
    temporary_op_mode = op_mode;
    temporary_op_mode_set_at = get_seconds_since_boot();
    temporary_op_mode_duration_seconds = seconds;
}

// Get the operating mode.  Note that this is an extremely low level routine, so
// don't modify it to call anything that might potentially go recursive.
uint16_t sensor_op_mode() {

    // Make sure we act in normal mode if battery is truly dead, no matter what mode
    // we are actually in.  This blocks aggressive comms and sensor measurement
    // behavior in many of the test modes.
    if (battery_soc() < 10.0)
        return OPMODE_NORMAL;

    // Allow it to be temporarily overridden
    if (temporary_op_mode_duration_seconds != 0) {
        if (ShouldSuppress(&temporary_op_mode_set_at, temporary_op_mode_duration_seconds))
            return temporary_op_mode;
        temporary_op_mode_duration_seconds = 0;
    }

    // Return the actual mode
    return(operating_mode);

}

// Skip handler for sensors that shouldn't ever be active if in mobile mode
bool g_mobile_skip(void *g) {
    if (sensor_op_mode() == OPMODE_MOBILE)
        return true;
    return false;
}

// Return true if any test mode is turned on except for battery test
// mode, during which we want communications to happen.
bool sensor_test_mode() {
    return(fTestModeRequested || sensor_op_mode() == OPMODE_TEST_SENSOR);
}

// Abort all in-progress measurements
void sensor_abort(sensor_t *s) {
    if (s->state.is_configured)
        sensor_measurement_completed(s);
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
        DEBUG_PRINTF("%s measured\n", s->name);
}

// Mark a sensor as being permanently unconfigured because of an error
void sensor_unconfigure(sensor_t *s) {
    // Note that we support calling of sensor routines directly in absence
    // of the sensor packaging having been involved, in which case this
    // will be null
    if (s == NULL)
        return;
    s->state.is_completed = true;
    s->state.is_polling_valid = false;
    if (sensor_op_mode() == OPMODE_TEST_BURN) {
        DEBUG_PRINTF("Would have deconfigured if not in burn-in test mode: %s\n", s->name);
    } else {
        s->state.is_requesting_deconfiguration = true;
        DEBUG_PRINTF("DECONFIGURING %s\n", s->name);
    }
}

// Determine whether or not polling is valid right now
bool sensor_group_is_polling_valid(group_t *g) {
    if (debug(DBG_SENSOR_SUPERDUPERMAX)) {
        if (g->state.is_polling_valid)
            DEBUG_PRINTF("%s poll\n", g->name);
    }
    return(g->state.is_polling_valid);
}

// Determine whether or not polling is valid right now
bool sensor_is_polling_valid(sensor_t *s) {
    if (debug(DBG_SENSOR_SUPERDUPERMAX)) {
        if (s->state.is_polling_valid)
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
    return((sensor_op_mode() == OPMODE_TEST_SENSOR) && s->state.is_being_tested);
}

// Look up a sensor group by name
void sensor_test(char *name) {
    group_t **gp, *g;
    sensor_t **sp, *s;
    sensor_set_op_mode(OPMODE_NORMAL);
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
        if (name[0] != '\0')
            DEBUG_PRINTF("Sensor not found\n");
    }
}

// Mark all but the GPS groups as needing to be measured NOW, for debugging
bool sensor_schedule_now() {
    group_t **gp, *g;
    if (!fInit) {
        DEBUG_PRINTF("Sensor package not yet initialized - try again.\n");
        return false;
    }
    for (gp = &sensor_groups[0]; (g = *gp) != END_OF_LIST; gp++) {
        if (g->state.is_configured) {
            if (strcmp(g->name, GPS_SENSOR_GROUP) == 0)
                continue;
            g->state.last_repeated = 0;
        }
    }
    DEBUG_PRINTF("Sensor timings have been accelerated.\n");
    return true;
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
    if (somethingCompleted && debug(DBG_SENSOR_MAX))
        DEBUG_PRINTF("%s is completed.\n", g->name);
    return (somethingCompleted);
}

// Mark all sensors within an entire group as having been unconfigured
void sensor_group_unconfigure(group_t *g) {
    // Note that we support calling of sensor group routines directly in absence
    // of the sensor packaging having been involved, in which case this
    // will be null
    if (g == NULL)
        return;
    if (sensor_op_mode() == OPMODE_TEST_BURN) {
        DEBUG_PRINTF("Would have deconfigured if not in burn-in test mode: %s\n", g->name);
    } else {
        g->state.is_requesting_deconfiguration = false;
        DEBUG_PRINTF("DECONFIGURING %s\n", g->name);
    }
}

// Determine if group is powered on
bool sensor_group_powered_on(group_t *g) {
    return(g->state.is_powered_on);
}

// Test to see if any sensor is running
bool sensor_group_busy() {
    group_t **gp, *g;
    if (!fInit)
        return false;
    if (sensor_op_mode() == OPMODE_TEST_SENSOR)
        return false;
    // If any sensor is busy, don't allow it
    for (gp = &sensor_groups[0]; (g = *gp) != END_OF_LIST; gp++)
        if (g->state.is_configured && g->state.is_processing)
            return true;
    // If we're in oneshot mode and comms is busy, it should be the same as a sensor being busy
    if (storage()->oneshot_minutes != 0 && !comm_is_deselected())
        return true;
    return false;
}

// Test to see if any sensor is running
bool sensor_group_any_exclusive_busy() {
    group_t **gp, *g;
    if (!fInit)
        return false;
    if (sensor_op_mode() == OPMODE_TEST_SENSOR)
        return false;
    // If any sensor is busy, don't allow it
    for (gp = &sensor_groups[0]; (g = *gp) != END_OF_LIST; gp++)
        if (g->state.is_configured && g->state.is_processing && g->exclusive)
            return true;
    return false;
}

// Test to see if any sensor is powered on
bool sensor_group_any_exclusive_powered_on() {
    group_t **gp, *g;
    if (!fInit)
        return false;
    if (sensor_op_mode() == OPMODE_TEST_SENSOR)
        return false;
    for (gp = &sensor_groups[0]; (g = *gp) != END_OF_LIST; gp++) {
        if (g->state.is_configured && g->power_set != NO_HANDLER && g->power_exclusive && g->state.is_powered_on)
            return true;
    }
    return false;
}

// Test to see if any sensor's TWI is in use
bool sensor_group_any_exclusive_twi_on() {
    group_t **gp, *g;
    if (!fInit)
        return false;
    for (gp = &sensor_groups[0]; (g = *gp) != END_OF_LIST; gp++) {
        if (g->state.is_configured && g->state.is_processing && g->twi_exclusive)
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
    if (sensor_op_mode() == OPMODE_TEST_SENSOR)
        return false;

    for (gp = &sensor_groups[0]; (g = *gp) != END_OF_LIST; gp++) {
        if (g->state.is_configured) {
            for (sp = &g->sensors[0]; (s = *sp) != END_OF_LIST; sp++)
                if (s->state.is_configured && s->upload_needed != NO_HANDLER)
                    if (s->upload_needed(s))
                        return true;
        }
    }
    if (debug(DBG_SENSOR_SUPERDUPERMAX))
        DEBUG_PRINTF("No sensors have pending measurements.\n");
    return false;
}

// Get a group's repeat minutes, adjusted for debugging
uint16_t group_repeat_seconds(group_t *g) {
    uint16_t bat_status = battery_status();
    uint16_t repeat_seconds = 0;
    repeat_t *r;

    // If overridden, use it
    if (g->state.repeat_seconds_override != 0) {
        return g->state.repeat_seconds_override;
    }

    // Loop, finding the appropriate battery status
    for (r = g->repeat;; r++) {
        if ((bat_status & r->active_battery_status) != 0) {
            repeat_seconds = r->repeat_seconds;
            break;
        }
    }

    // Bug check
    if (repeat_seconds == 0)
        DEBUG_PRINTF("%s repeat seconds not found for %s\n", g->name, battery_status_name());

    // If we're testing, just double it
    if (bat_status == BAT_TEST)
        return (repeat_seconds/2);

    return(repeat_seconds);
}

// Show the entire sensor state
void sensor_show_values(bool fReset) {
    static uint32_t when = 0;
    char buffer[256];
    group_t **gp, *g;
    sensor_t **sp, *s;
    if (fReset)
        when = get_seconds_since_boot();
    if (when == 0)
        return;
    for (gp = &sensor_groups[0]; (g = *gp) != END_OF_LIST; gp++)
        if (g->state.is_configured)
            for (sp = &g->sensors[0]; (s = *sp) != END_OF_LIST; sp++)
                if (s->state.is_configured)
                    if (s->show_value != NO_HANDLER)
                        if (s->show_value(when, buffer, sizeof(buffer)-1))
                            DEBUG_PRINTF("%s\n", buffer);

}

// Show the entire sensor state
void sensor_show_state(bool fVerbose) {
    group_t **gp, *g;
    sensor_t **sp, *s;
    uint32_t seconds_since_boot = get_seconds_since_boot();
    char buffp[512];

    if (!fInit) {
        DEBUG_PRINTF("Not yet initialized.\n");
        return;
    }

    if (fVerbose)
        DEBUG_PRINTF("%s UART:%s\n", battery_status_name(battery_status()), gpio_uart_name(gpio_current_uart()));

    buffp[0] = '\0';
    for (gp = &sensor_groups[0]; (g = *gp) != END_OF_LIST; gp++) {
        strcat(buffp, g->name);
        strcat(buffp, "[");
        if (!g->state.is_configured) {
            if (fVerbose)
                DEBUG_PRINTF("%s UNCONFIGURED\n", g->name);
            strcat(buffp, "X] ");
            continue;
        }
        if (((battery_status() & g->active_battery_status) != 0)) {
            bool fSkip = false;
            if (g->skip_handler != NO_HANDLER)
                if (g->skip_handler(g))
                    fSkip = true;
            bool fOverdue = false;
            int nextsecs = (group_repeat_seconds(g)) - (seconds_since_boot-g->state.last_repeated);
            if (nextsecs < 0) {
                nextsecs = -nextsecs;
                fOverdue = true;
            }
            int nextmin = nextsecs/60;
            nextsecs -= nextmin*60;
            char buff[128];
            if (fOverdue) {
                sprintf(buff, "is overdue to resample by %dm%ds", nextmin, nextsecs);
            }
            else if (g->state.last_repeated == 0) {
                sprintf(buff, "next up");
            } else {
                char buff2[16];
                if (nextmin == 0)
                    sprintf(buff2, "%ds", nextsecs);
                else
                    sprintf(buff2, "%dm", nextmin);
                strcat(buffp, buff2);
                sprintf(buff, "next up %dm%ds", nextmin, nextsecs);
            }
            if (fSkip) {
                strcat(buff, " when !skip");
                strcat(buffp, "S");
            } else {
                if (g->exclusive && sensor_group_busy()) {
                    strcat(buff, " when all idle");
                    strcat(buffp, "B");
                }
                if (g->power_exclusive && sensor_group_any_exclusive_powered_on()) {
                    strcat(buff, " when power avail");
                    strcat(buffp, "P");
                }
                if (g->twi_exclusive && sensor_group_any_exclusive_twi_on()) {
                    strcat(buff, " when twi avail");
                    strcat(buffp, "T");
                }
                if (g->uart_required != UART_NONE && gpio_current_uart() != UART_NONE) {
                    strcat(buff, " when UART avail");
                    strcat(buffp, "U");
                }
                if (comm_uart_switching_allowed() && g->uart_requested != UART_NONE && gpio_current_uart() != UART_NONE) {
                    strcat(buff, " when UART avail");
                    strcat(buffp, "U");
                }
            }
            if (g->state.is_being_tested) {
                strcat(buff, " (being tested)");
                strcat(buffp, "T");
            }

            if (fVerbose)
                DEBUG_PRINTF("%s %s\n", g->name, g->state.is_processing ? (g->state.is_settling ? "now settling" : "now sampling") : buff);

            strcat(buffp, "] ");
            for (sp = &g->sensors[0]; (s = *sp) != END_OF_LIST; sp++) {
                strcat(buffp, s->name);
                strcat(buffp, "(");
                if (!s->state.is_configured) {
                    if (fVerbose)
                        DEBUG_PRINTF("   %s UNCONFIGURED\n", s->name);
                    strcat(buffp, "X");
                } else {
                    bool fUploadNeeded = false;
                    if (s->upload_needed != NO_HANDLER)
                        fUploadNeeded = s->upload_needed(s);
                    char buff[40];
                    if (s->state.is_processing) {
                        sprintf(buff, "%s for %ds", s->state.is_settling ? "now settling" : "now sampling", (int) (seconds_since_boot - s->state.last_settled));
                        strcat(buffp, "s");
                    } else {
                        strcpy(buff, "waiting");
                        strcat(buffp, "w");
                    }
                    if (s->state.is_being_tested) {
                        strcat(buff, " (being tested)");
                        strcat(buffp, "t");
                    }
                    if (s->state.is_processing || s->state.is_being_tested || fUploadNeeded) {
                        if (fVerbose)
                            DEBUG_PRINTF("   %s %s%s\n", s->name, s->state.is_completed ? "completed" : buff, fUploadNeeded ? ", waiting to upload" : "");
                        strcat(buffp, fUploadNeeded ? "u" : "c");
                    }
                }
                strcat(buffp, ") ");
            }

        }

    }
    if (!fVerbose)
        DEBUG_PRINTF("%s\n", buffp);

}

// Standard power on/off handler
void sensor_set_pin_state(uint16_t pin, bool enable) {
    if (pin != SENSOR_PIN_UNDEFINED)
        gpio_power_set(pin, enable);
}

// Poll, advancing the state machine
void sensor_poll() {
    static int inside_poll = 0;
    bool groups_currently_active;
    int pending;
    group_t **gp, *g;
    sensor_t **sp, *s;

    // Exit if we haven't yet initialized GPS, which is a big signal that we're not yet ready to proceed,
    // except for the case of UGPS when we need sensor processing to acquire GPS
#ifndef UGPS
    uint16_t status = comm_gps_get_value(NULL, NULL, NULL);
    if (status != GPS_NOT_CONFIGURED && status != GPS_LOCATION_ABORTED)
        if (status != GPS_LOCATION_FULL && status != GPS_LOCATION_PARTIAL)
            return;
#endif

    // Initialize if we haven't yet done so
    if (!fInit) {
        sensor_init();
        fInit = true;
    }

    // Debug sensors
    if (debug(DBG_SENSOR_POLL)) {
        sensor_show_values(false);
    }

    // Debug TWI
#ifdef TWIX
    twi_status_check(false);
#endif

    // Exit if we're already inside the poller.  This DOES happen if one of the handlers (such as an
    // init handler) takes an incredibly long time because of, say, a retry loop.
    if (inside_poll++ != 0) {
        inside_poll--;
        DEBUG_PRINTF("sensor_poll REENTRANCY!\n");
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
        if (sensor_op_mode() == OPMODE_TEST_SENSOR && !g->state.is_being_tested)
            continue;

        // Are we completely idle?
        if (!g->state.is_processing && !g->state.is_settling) {

            if (debug(DBG_SENSOR_SUPERDUPERMAX))
                DEBUG_PRINTF("%s !processing !settling\n", g->name);

            // If we've requested test mode, don't schedule anything new
            if (fTestModeRequested)
                continue;

            // Skip if this group doesn't need to be processed right now
            if (g->skip_handler != NO_HANDLER && sensor_op_mode() != OPMODE_TEST_SENSOR)
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
            if (fSkipGroup && sensor_op_mode() != OPMODE_TEST_SENSOR) {
                if (debug(DBG_SENSOR_SUPERDUPERMAX)) {
                    if (Sensors == 0)
                        DEBUG_PRINTF("Skipping %s because no sensors are found.\n", g->name);
                    else
                        DEBUG_PRINTF("Skipping %s because all its sensors' uploads are pending.\n", g->name);
                }
                continue;
            }

            // If this sensor group can only be run when nothing else is running, skip if busy
            if (g->exclusive) {
                if (sensor_group_busy()) {
                    if (debug(DBG_SENSOR_SUPERDUPERMAX))
                        DEBUG_PRINTF("Skipping %s because something else is being processed.\n", g->name);
                    continue;
                }
            } else {
                if (sensor_group_any_exclusive_busy()) {
                    if (debug(DBG_SENSOR_SUPERDUPERMAX))
                        DEBUG_PRINTF("Skipping %s because exclusive is being processed.\n", g->name);
                    continue;
                }
            }

            // If this sensor group is a particular power hog and needs to be run
            // only when other exclusives aren't powered on, skip the group if anyone else
            // is currently powered on.
            if (g->power_exclusive && sensor_group_any_exclusive_powered_on()) {
                if (debug(DBG_SENSOR_SUPERDUPERMAX))
                    DEBUG_PRINTF("Skipping %s because something else is powered on.\n", g->name);
                continue;
            }

            // If this sensor group requires TWI and can only run when other exclusives
            // aren't using TWI, skip the group if anyone else is currently using TWI.
            if (g->twi_exclusive && sensor_group_any_exclusive_twi_on()) {
                if (debug(DBG_SENSOR_SUPERDUPERMAX))
                    DEBUG_PRINTF("Skipping %s because something else is using TWI.\n", g->name);
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
            if ((battery_status() & g->active_battery_status) == 0) {
                if (debug(DBG_SENSOR_SUPERDUPERMAX))
                    DEBUG_PRINTF("Skipping %s because 0x%04x doesn't map to %s\n", g->name, g->active_battery_status, battery_status_name());
                continue;
            }

            // If we're not in the right comms mode, skip it
            if ((comm_mode() & g->active_comm_mode) == 0) {
                if (debug(DBG_SENSOR_SUPERDUPERMAX))
                    DEBUG_PRINTF("Skipping %s because %04x doesn't map to active comm mode (%04x).\n", g->name, g->active_comm_mode, comm_mode());
                continue;
            }

            // If we're in the repeat idle period for this group, just go to next group
            if (sensor_op_mode() != OPMODE_TEST_SENSOR)
                if (ShouldSuppressConsistently(&g->state.last_repeated, group_repeat_seconds(g)))
                    continue;

            // Initialize sensor state and refresh configuration state
            for (sp = &(*gp)->sensors[0]; (s = *sp) != END_OF_LIST; sp++) {
                if (s->state.is_configured) {
                    s->state.is_settling = false;
                    s->state.is_processing = false;
                    s->state.is_completed = false;
                }
            }

            // Begin processing
            g->state.is_processing = true;

            if (debug(DBG_SENSOR_SUPERDUPERMAX))
                DEBUG_PRINTF("%s processing\n", g->name);

            // Power ON the module
            if (g->power_set != NO_HANDLER) {
                g->power_set(g->power_set_parameter, true);
                g->state.is_powered_on = true;
                // Delay a bit before proceeding to do anything at all
                nrf_delay_ms(MAX_NRF_DELAY_MS);
                if (debug(DBG_SENSOR_MAX))
                    DEBUG_PRINTF("%s power ON\n", g->name);
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
                    if (!s->init_power(s, s->init_parameter))
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

            if (g->settling_seconds != 0 && debug(DBG_SENSOR_MAX))
                DEBUG_PRINTF("Begin %s settling for %ds\n", g->name, g->settling_seconds);

            // Start the group app timer when the power is applied, because a core part of the
            // settling period for certain TWI devices (i.e. GPS) is that you need to
            // "warm them up" by pulling data out of them on a continuous basis.
            if (g->poll_handler != NO_HANDLER && !g->poll_continuously && g->poll_during_settling) {
                if (debug(DBG_SENSOR_MAX))
                    DEBUG_PRINTF("%s timer ON\n", g->name);
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
                    if (debug(DBG_SENSOR_MAX))
                        DEBUG_PRINTF("%s timer ON\n", s->name);
                    app_timer_start(s->state.sensor_timer.timer_id, APP_TIMER_TICKS(s->poll_repeat_milliseconds, APP_TIMER_PRESCALER), s);
                    // Release the poller so that it's ok to proceed
                    s->state.is_polling_valid = true;
                }

            }

        }

        // Are we in the settling period?
        if (g->state.is_processing && g->state.is_settling) {
            // groups_currently_active++;

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
                if (debug(DBG_SENSOR_MAX))
                    DEBUG_PRINTF("%s timer ON\n", g->name);
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
                    if (debug(DBG_SENSOR_MAX))
                        DEBUG_PRINTF("%s timer ON\n", s->name);
                    app_timer_start(s->state.sensor_timer.timer_id, APP_TIMER_TICKS(s->poll_repeat_milliseconds, APP_TIMER_PRESCALER), s);
                    // Release the poller so that it's ok to proceed
                    s->state.is_polling_valid = true;
                }

            }

        }

        // Is it time to do some processing?
        if (g->state.is_processing && !g->state.is_settling) {
            // groups_currently_active++;

            if (debug(DBG_SENSOR_SUPERDUPERMAX))
                DEBUG_PRINTF("%s processing !settling\n", g->name);

            // Loop over all sensors in this group, looking for work to do
            for (sp = &(*gp)->sensors[0]; (s = *sp) != END_OF_LIST; sp++) {

                // Skip unconfigured sensors
                if (!s->state.is_configured)
                    continue;

                // If test mode and not being tested, bail
                if (sensor_op_mode() == OPMODE_TEST_SENSOR && !s->state.is_being_tested)
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

                }

                // Keep measuring the sensor until it reports that it has "completed"
                if (s->state.is_processing && !s->state.is_completed && !s->state.is_settling) {

                    // Initiate the measurement.
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
                    if (sensor_op_mode() != OPMODE_TEST_SENSOR || s->state.is_being_tested)
                        pending++;

        if (debug(DBG_SENSOR_SUPERDUPERMAX))
            DEBUG_PRINTF("%s still not completed\n", g->name);

        // If none of the sensors has any work pending, we're done with this group.
        if (pending == 0) {

            // Stop the app timer if one had been requested
            if (g->poll_handler != NO_HANDLER && !g->poll_continuously) {
                app_timer_stop(g->state.group_timer.timer_id);
                if (debug(DBG_SENSOR_MAX))
                    DEBUG_PRINTF("%s timer OFF\n", g->name);
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
                    if (debug(DBG_SENSOR_MAX))
                        DEBUG_PRINTF("%s timer OFF\n", s->name);
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
                g->power_set(g->power_set_parameter, false);
                g->state.is_powered_on = false;
                if (debug(DBG_SENSOR_MAX))
                    DEBUG_PRINTF("%s power OFF\n", g->name);
            }

            // Clear our own state, setting us to idle.
            g->state.is_processing = false;

            // At the very end of group processing, satisfy any sensor deconfiguration requests
            int configured_sensors = 0;
            for (sp = &(*gp)->sensors[0]; (s = *sp) != END_OF_LIST; sp++)
                if (s->state.is_configured) {
                    if (s->state.is_requesting_deconfiguration)
                        s->state.is_configured = false;
                    else
                        configured_sensors++;
                }

            // If there are no sensors left to process, request deconfiguration of the group
            if (configured_sensors == 0)
                g->state.is_requesting_deconfiguration = true;

            // Now that the group is quiescent, deconfiguration is requested, do it
            if (g->state.is_requesting_deconfiguration)
                g->state.is_configured = false;

            // Done
            if (debug(DBG_SENSOR_SUPERDUPERMAX))
                DEBUG_PRINTF("%s !processing\n", g->name);
            if (debug(DBG_SENSOR_MAX))
                DEBUG_PRINTF("%s completed\n", g->name);

        }

    } // Looping across groups

    // If no groups are currently active and test mode was requested, we
    // can now enter it.
    if (fTestModeRequested) {
        if (groups_currently_active == 0) {
            fTestModeRequested = false;
            sensor_set_op_mode(OPMODE_TEST_SENSOR);
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
        g->state.is_requesting_deconfiguration = false;
        g->state.is_configured = (g->storage_product == c->product);
        if (!g->state.is_configured)
            continue;

        // Modify the sensor parameters to reflect what's in the storage parameters
        g->state.repeat_seconds_override = 0;
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
                        DEBUG_PRINTF("%s override repeat with %d minutes\n", g->name, v);
                    g->state.repeat_seconds_override = (uint16_t) v*60;
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

        // If it's to be sensed immediately, do it, else base repeats on when init started
        if (g->sense_at_boot)
            g->state.last_repeated = 0;
        else {
            g->state.last_repeated = init_time;
#if defined(FAST)
            g->state.last_repeated = 0;
#endif
        }
        
        // Power OFF the module as its initial state
        if (g->power_set == NO_HANDLER)
            g->state.is_powered_on = true;
        else {
            g->power_set(g->power_set_parameter, false);
            g->state.is_powered_on = false;
            if (debug(DBG_SENSOR_MAX))
                DEBUG_PRINTF("%s power OFF\n", g->name);
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
            if (g->settling_seconds != 0 && g->settling_seconds < min_settling_seconds)
                g->settling_seconds = min_settling_seconds;
        }

        // Loop over all sensors in the group
        uint16_t configured_sensors = 0;
        for (sp = &(*gp)->sensors[0]; (s = *sp) != END_OF_LIST; sp++) {

            // If not configured, don't bother initializing anything else
            s->state.is_requesting_deconfiguration = false;
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
                if (s->settling_seconds != 0 && s->settling_seconds < min_settling_seconds)
                    s->settling_seconds = min_settling_seconds;
            }

            // If there's an init handler, call it
            if (s->init_once != NO_HANDLER) {
                if (!s->init_once(s, s->init_parameter))
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
