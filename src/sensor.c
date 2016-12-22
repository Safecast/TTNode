// Sensor measurement processing

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
#include "bme.h"
#include "comm.h"
#include "send.h"
#include "ina.h"
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

// Default this to TRUE so that we charge up to MAX at boot before starting to draw down
static bool fullBatteryRecoveryMode = true;

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

// Returns a value based on the battery status.  Note that these values
// are defined in sensor.h as a bit mask, so they can be tested via "==" or switch
// OR via bitwise-&, as opposed to *needing* to do == or a switch statement.
uint16_t sensor_get_battery_status() {

#if !BATTERY_AUTOADJUST
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
        if (lastKnownBatterySOC < 60.0)
            return BAT_EMERGENCY;
        batteryRecoveryMode = false;
        return BAT_NORMAL;
    } else {
        if (lastKnownBatterySOC < 20.0) {
            batteryRecoveryMode = true;
            return BAT_EMERGENCY;
        }
    }

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

// Mark a sensor which is being processed as being completed
void sensor_measurement_completed(sensor_t *s) {
    // Note that we support calling of sensor routines directly in absence
    // of the sensor packaging having been involved, in which case this
    // will be null
    if (s == NULL)
        return;
    s->state.is_completed = true;
    if (debug(DBG_SENSOR))
        DEBUG_PRINTF("%s is measured.\n", s->name);
}

// Mark a sensor as being permanently unconfigured because of an error
void sensor_unconfigure(sensor_t *s, uint32_t err_code) {
    // Note that we support calling of sensor routines directly in absence
    // of the sensor packaging having been involved, in which case this
    // will be null
    if (s == NULL)
        return;
    s->state.is_completed = true;
    s->state.is_configured = false;
    if (debug(DBG_SENSOR))
        DEBUG_PRINTF("%s unconfigured (0x%04x)\n", s->name, err_code);
}

// Determine whether or not polling is valid right now
bool sensor_is_polling_valid(group_t *g) {
    if (!g->state.is_polling_valid)
        if (debug(DBG_SENSOR))
            DEBUG_PRINTF("%s spurious poll ignored\n", g->name);
    return(g->state.is_polling_valid);
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
    if (g->state.is_settling)
        return false;
    for (sp = &g->sensors[0]; (s = *sp) != END_OF_LIST; sp++)
        if (s->state.is_configured && !s->state.is_completed) {
            s->state.is_completed = true;
            somethingCompleted = true;
        }
    if (somethingCompleted && debug(DBG_SENSOR))
        DEBUG_PRINTF("%s is completed.\n", g->name);
    return (somethingCompleted);
}

// Mark all sensors within an entire group as having been completed
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
            s->state.is_configured = false;
        }
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

    for (gp = &sensor_groups[0]; (g = *gp) != END_OF_LIST; gp++) {
        if (g->state.is_configured) {
            for (sp = &g->sensors[0]; (s = *sp) != END_OF_LIST; sp++)
                if (s->state.is_configured && s->upload_needed != NO_HANDLER)
                    if (s->upload_needed(s)) {
                        if (gpio_motion_sense(MOTION_QUERY)) {
                            if (debug(DBG_SENSOR))
                                DEBUG_PRINTF("SENSOR: upload pending, but device is currently in-motion\n");
                            return false;
                        }
                        return true;
                    }
        }
    }
    if (debug(DBG_SENSOR_SUPERMAX))
        DEBUG_PRINTF("No sensors have pending measurements.\n");
    return false;
}

// Show the entire sensor state
void sensor_show_state() {
    group_t **gp, *g;
    sensor_t **sp, *s;

    if (!fInit) {
        DEBUG_PRINTF("Not yet initialized.\n");
        return;
    }

    DEBUG_PRINTF("Battery status: %d\n", sensor_get_battery_status());
    DEBUG_PRINTF("Motion status: %d\n", gpio_motion_sense(MOTION_QUERY));

    for (gp = &sensor_groups[0]; (g = *gp) != END_OF_LIST; gp++) {
        if (g->state.is_configured && ((sensor_get_battery_status() & g->active_battery_status) != 0)) {
            bool fOverdue = false;
            int nextsecs = (g->repeat_minutes*60) - (get_seconds_since_boot()-g->state.last_repeated);
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
            DEBUG_PRINTF("== %s %s\n", g->name, g->state.is_processing ? (g->state.is_settling ? "now settling" : "now sampling") : buff);

            for (sp = &g->sensors[0]; (s = *sp) != END_OF_LIST; sp++) {
                if (s->state.is_configured) {
                    bool fUploadNeeded = false;
                    if (s->upload_needed != NO_HANDLER)
                        fUploadNeeded = s->upload_needed(s);
                    char buff[40];
                    if (s->state.is_processing)
                        sprintf(buff, "%s for %ds", g->state.is_settling ? "now settling" : "now sampling", (int) (get_seconds_since_boot() - g->state.last_settled));
                    else
                        strcpy(buff, "waiting");
                    DEBUG_PRINTF("--- %s %s%s\n", s->name, s->state.is_completed ? "completed" : buff, fUploadNeeded ? ", waiting to upload" : "");
                }
            }

        }

    }

}

// Standard power on/off handler
void sensor_set_pin_state(uint16_t pin, bool init, bool enable) {
    if (pin != PIN_UNDEFINED) {
        if (init)
            gpio_power_init(pin, enable);
        else
            gpio_power_set(pin, enable);
    }
}

// Poll, advancing the state machine
void sensor_poll() {
    static int inside_poll = 0;
    int pending, configured_sensors;
    group_t **gp, *g;
    sensor_t **sp, *s;
    STORAGE *c = storage();

    // Exit if we haven't yet initialized GPS, which is a big signal that we're not yet ready to proceed
    uint16_t status = comm_gps_get_value(NULL, NULL, NULL);
    if (status != GPS_NOT_CONFIGURED)
        if (status != GPS_LOCATION_FULL && status != GPS_LOCATION_PARTIAL)
            return;

    // Initialize if we haven't yet done so
    if (!fInit) {
        sensor_init();
        fInit = true;
    }

    // Exit if we're already inside the poller.  This DOES happen if one of the handlers (such as an
    // init handler) takes an incredibly long time because of, say, a retry loop.
    if (inside_poll++ != 0) {
        inside_poll--;
        return;
    }

    if (debug(DBG_SENSOR_SUPERMAX))
        DEBUG_PRINTF("sensor_poll enter\n");

    // Loop over all configured sensors in all configured groups
    for (gp = &sensor_groups[0]; (g = *gp) != END_OF_LIST; gp++) {

        // If not configured, skip this group
        if (!g->state.is_configured)
            continue;

        // Are we completely idle?
        if (!g->state.is_processing && !g->state.is_settling) {

            if (debug(DBG_SENSOR_SUPERMAX))
                DEBUG_PRINTF("%s !processing !settling\n", g->name);

            // If ALL of the sensors in this group already have pending measurements,
            // skip the group because it's senseless to keep measuring.
            bool fSkipGroup = true;
            for (sp = &(*gp)->sensors[0]; (s = *sp) != END_OF_LIST; sp++) {
                if (s->state.is_configured) {
                    if (s->upload_needed == NO_HANDLER || !s->upload_needed(s)) {
                        fSkipGroup = false;
                        break;
                    }
                }
            }
            if (fSkipGroup) {
                if (debug(DBG_SENSOR_SUPERMAX))
                    DEBUG_PRINTF("Skipping %s because all its sensors' uploads are pending.\n", g->name);
                continue;
            }

            // If this sensor group is a particular power hog and needs to be run
            // only when other exclusives aren't powered on, skip the group if anyone else
            // is currently powered on.
            if (g->power_exclusive && sensor_group_any_exclusive_powered_on()) {
                if (debug(DBG_SENSOR_SUPERMAX))
                    DEBUG_PRINTF("Skipping %s because something else is powered on.\n", g->name);
                continue;
            }

            // If this sensor group requires a uart, but the UART is busy, skip the group
            if (g->uart_required != UART_NONE && gpio_current_uart() != UART_NONE) {
                if (debug(DBG_SENSOR_SUPERMAX))
                    DEBUG_PRINTF("Skipping %s because the required UART is busy.\n", g->name);
                continue;
            }

            // If this sensor group requests a uart (but is allowed to run without it if
            // it CAN'T be granted), but the UART is busy, skip the group
            if (comm_uart_switching_allowed() && g->uart_requested != UART_NONE && gpio_current_uart() != UART_NONE) {
                if (debug(DBG_SENSOR_SUPERMAX))
                    DEBUG_PRINTF("Skipping %s because the requested UART is busy.\n", g->name);
                continue;
            }

            // If we're not in the right battery status, skip it
            if ((sensor_get_battery_status() & g->active_battery_status) == 0) {
                if (debug(DBG_SENSOR_SUPERMAX))
                    DEBUG_PRINTF("Skipping %s because %04x doesn't map to active battery status (%04x).\n", g->name, g->active_battery_status, sensor_get_battery_status());
                continue;
            }

            // If we're not in the right comms mode, skip it
            if ((comm_mode() & g->active_comm_mode) == 0) {
                if (debug(DBG_SENSOR_SUPERMAX))
                    DEBUG_PRINTF("Skipping %s because %04x doesn't map to active comm mode (%04x).\n", g->name, g->active_comm_mode, comm_mode());
                continue;
            }

            // If we're in the repeat idle period for this group, just go to next group
            if (ShouldSuppressConsistently(&g->state.last_repeated, g->repeat_minutes*60))
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
            g->state.last_repeated = get_seconds_since_boot();

            if (debug(DBG_SENSOR_SUPERMAX))
                DEBUG_PRINTF("%s processing\n", g->name);

            // Power ON the module
            if (g->power_set != NO_HANDLER) {
                g->power_set(g->power_set_parameter, false, true);
                g->state.is_powered_on = true;
                // Delay a bit before proceeding to do anything at all
                nrf_delay_ms(MAX_NRF_DELAY_MS);
                if (debug(DBG_SENSOR))
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
                    if (!s->init_power())
                        s->state.init_failures++;
                    else
                        s->state.init_failures = 0;
                }

            }

            // Begin the settling period
            g->state.last_settled = get_seconds_since_boot();
            g->state.is_settling = true;

            if (debug(DBG_SENSOR_SUPERMAX))
                DEBUG_PRINTF("%s settling\n", g->name);

            if (g->settling_seconds != 0 && debug(DBG_SENSOR))
                DEBUG_PRINTF("Begin %s settling for %ds\n", g->name, g->settling_seconds);

            // Start the app timer when the power is applied, because a core part of the
            // settling period for certain TWI devices (i.e. GPS) is that you need to
            // "warm them up" by pulling data out of them on a continuous basis.
            if (g->poll_handler != NO_HANDLER && !g->poll_continuously && g->poll_during_settling) {
                if (debug(DBG_SENSOR))
                    DEBUG_PRINTF("Starting %s timer\n", g->name);
                app_timer_start(g->state.group_timer.timer_id, APP_TIMER_TICKS(g->poll_repeat_milliseconds, APP_TIMER_PRESCALER), g);
                // Release the poller so that it's ok to proceed
                g->state.is_polling_valid = true;
            }

        }

        // Are we in the settling period?
        if (g->state.is_processing && g->state.is_settling) {

            if (debug(DBG_SENSOR_SUPERMAX))
                DEBUG_PRINTF("%s processing settling\n", g->name);

            // If we're in the settling idle period for this group, just go to the next group
            if (g->settling_seconds != 0)
                if (ShouldSuppress(&g->state.last_settled, g->settling_seconds))
                    continue;

            // Stop the settling period.
            g->state.is_settling = false;

            // If there's a handler to be called after settling, call it
            if (g->done_settling != NO_HANDLER)
                g->done_settling();

            // Start the app timer when settling is over, if that's what was requested
            if (g->poll_handler != NO_HANDLER && !g->poll_continuously && !g->poll_during_settling) {
                if (debug(DBG_SENSOR))
                    DEBUG_PRINTF("Starting %s timer\n", g->name);
                app_timer_start(g->state.group_timer.timer_id, APP_TIMER_TICKS(g->poll_repeat_milliseconds, APP_TIMER_PRESCALER), g);
                // Release the poller so that it's ok to proceed
                g->state.is_polling_valid = true;
            }
        }

        // Is it time to do some processing?
        if (g->state.is_processing && !g->state.is_settling) {

            if (debug(DBG_SENSOR_SUPERMAX))
                DEBUG_PRINTF("%s processing !settling\n", g->name);

            // Loop over all sensors in this group, looking for work to do
            for (sp = &(*gp)->sensors[0]; (s = *sp) != END_OF_LIST; sp++) {

                // Skip unconfigured sensors
                if (!s->state.is_configured)
                    continue;

                // Is this a candidate for initiating work?
                if (!s->state.is_processing && !s->state.is_completed) {

                    // Begin processing
                    s->state.is_processing = true;

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
                    pending++;

        if (debug(DBG_SENSOR_SUPERMAX))
            DEBUG_PRINTF("%s still not completed\n", g->name);

        // If none of the sensors has any work pending, we're done.
        if (pending == 0) {

            // Stop the app timer if one had been requested
            if (g->poll_handler != NO_HANDLER && !g->poll_continuously) {
                app_timer_stop(g->state.group_timer.timer_id);
                if (debug(DBG_SENSOR))
                    DEBUG_PRINTF("Stopped %s timer\n", g->name);
                // Stop the poller so that it doesn't do any more processing
                g->state.is_polling_valid = false;
            }

            // Call the sensor power-off preparation functions
            for (sp = &g->sensors[0]; (s = *sp) != END_OF_LIST; sp++) {

                // If not configured, don't bother initializing anything else
                s->state.is_configured = ((s->storage_sensor_mask & c->sensors) != 0);
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
                if (debug(DBG_SENSOR))
                    DEBUG_PRINTF("%s power pin #%d OFF\n", g->name, g->power_set_parameter);
            }

            // Clear our own state, setting us to idle.
            g->state.is_processing = false;

            if (debug(DBG_SENSOR_SUPERMAX))
                DEBUG_PRINTF("%s !processing\n", g->name);

            if (debug(DBG_SENSOR))
                DEBUG_PRINTF("%s completed\n", g->name);

        }

    } // Looping across groups

    // Done

    if (debug(DBG_SENSOR_SUPERMAX))
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
                        DEBUG_PRINTF("%s override repeat_minutes %d with %d\n", g->name, g->repeat_minutes, v);
                    g->repeat_minutes = (uint16_t) v;
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
        g->state.last_repeated = init_time;
        g->state.is_polling_valid = false;

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
        for (sp = &(*gp)->sensors[0]; (s = *sp) != END_OF_LIST; sp++) {

            // If not configured, don't bother initializing anything else
            s->state.is_configured = ((s->storage_sensor_mask & c->sensors) != 0);
            if (!s->state.is_configured)
                continue;

            // Initialize sensor state
            s->state.is_settling = false;
            s->state.is_processing = false;
            s->state.is_completed = false;
            s->state.init_failures = 0;
            s->state.term_failures = 0;

            // If there's an init handler, call it
            if (s->init_once != NO_HANDLER) {
                if (!s->init_once())
                    s->state.init_failures++;
                else
                    s->state.init_failures = 0;
            }

        }
    }

}
