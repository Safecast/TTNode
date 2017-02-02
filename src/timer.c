// App-level timers

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "debug.h"
#include "nrf_delay.h"
#include "config.h"
#include "timer.h"
#include "storage.h"
#include "sensor.h"
#include "comm.h"
#include "gpio.h"
#include "phone.h"
#include "io.h"
#include "serial.h"
#include "bt.h"
#include "app_scheduler.h"
#include "app_timer_appsh.h"

// Primary app-level timer
#define TELETYPE_TIMER_INTERVAL         APP_TIMER_TICKS((TELETYPE_TIMER_SECONDS*1000), APP_TIMER_PRESCALER)
APP_TIMER_DEF(teletype_timer);

// Serial timers
APP_TIMER_DEF(serial_flush_timer);
APP_TIMER_DEF(serial_wakeup_timer);

// Primary clock, maintained by the primary app timer
// Initialize non-zero because zero is the default init value of all counters, and we want to look later than that.
static uint32_t seconds_since_boot = 1;
static uint32_t ticks_at_measurement = 0;

// Date/time
static uint32_t dt_seconds_since_boot_when_set = 0;
static uint32_t dt_date = 0;
static uint32_t dt_time = 0;

// Initialization
static bool comm_was_initialized = false;

// Access to our app-maintained system clock
uint32_t get_seconds_since_boot() {
    uint32_t ticks = app_timer_cnt_get();

    // If the clock has wrapped, just return the clock in TELETYPE_TIMER_SECONDS granularity
    if (ticks < ticks_at_measurement)
        return seconds_since_boot;

    // Compute seconds since last increment of our clock
    uint32_t elapsed_ticks = ticks - ticks_at_measurement;
    uint32_t elapsed_seconds = elapsed_ticks / APP_TIMER_TICKS_PER_SECOND;
    
    // Return finer-grained time
    return (seconds_since_boot + elapsed_seconds);    

}

// Set the date/time
void set_timestamp(uint32_t ddmmyy, uint32_t hhmmss) {
    // http://aprs.gids.nl/nmea/#rmc
    // 225446 Time of fix 22:54:46 UTC
    // 191194 Date of fix  19 November 1994
    dt_date = ddmmyy;
    dt_time = hhmmss;
    dt_seconds_since_boot_when_set = get_seconds_since_boot();
}

// Get the date/time
bool get_current_timestamp(uint32_t *date, uint32_t *time, uint32_t *offset) {

    // Exit if we've never acquired date/time from gps
    if (dt_seconds_since_boot_when_set == 0)
        return false;
    
    // Return this to the caller. This is obviously crufty, however it's best
    // to allow the server to do the offset calculation on the dates.  The other
    // benefit is that this is more compressable than a textual date/time.
    if (date != NULL)
        *date = dt_date;
    if (time != NULL)
        *time = dt_time;
    if (offset != NULL)
        *offset = (get_seconds_since_boot() - dt_seconds_since_boot_when_set);

    return true;
    
}

// Send a welcome message to the phone upon connect
void send_welcome_message(void) {
    static uint32_t btSessionIDLast = 0;
    uint32_t btSessionID = bluetooth_session_id();
    if (can_send_to_bluetooth() && btSessionID != btSessionIDLast) {
        btSessionIDLast = btSessionID;

        // Send a welcome message
        char message[64];
        uint32_t secs = get_seconds_since_boot();
        uint32_t mins = secs / 60;
        sprintf(message, "%lu alive %lum %lus on build %s", io_get_device_address(), mins, secs - mins * 60, app_build());
        phone_send(message);

        // Flag that we do NOT want to optimize power by shutting down
        // listens, even if this connection happens to drop.
        io_power_stay_suboptimal();
    }
}

// Primary app timer
void teletype_timer_handler(void *p_context) {
    
    // Bump the number of seconds since boot.
    // It's up to the users to anticipate overflow.
    seconds_since_boot += TELETYPE_TIMER_SECONDS;
    ticks_at_measurement = app_timer_cnt_get();

    // Restart if it's been requested
    io_restart_if_requested();

    // Exit if power is turned off - as when we're programming or charging the unit via USB
#ifdef SENSE_POWER_PIN
    if (!gpio_power_sensed()) {
        DEBUG_PRINTF("Power switch (GPIO P%d) OFF. This device %lu will remain idle.\n", SENSE_POWER_PIN, io_get_device_address());
        return;
    }
#endif

    // Update the status of whether or not the device is currently in-motion
    gpio_motion_sense(MOTION_UPDATE);

    // Terminate solicitations if we're optimizing power
    static bool fDropped = false;
    if (!fDropped && io_optimize_power()) {
        fDropped = true;
        drop_bluetooth();
        gpio_indicators_off();
    }

    // Say hello if we're just now connecting to BT
    send_welcome_message();

    // Poll and advance our communications state machine
    comm_poll();

    // If we've initialized at least once, poll and advance our sensor state machine,
    // except in the case of UGPS where we need the sensor polling to acquire GPS
#if (defined(LORA) || defined(CELLX)) && !defined(UGPS)
    if (!comm_was_initialized)
        comm_was_initialized = comm_mode() != COMM_NONE && comm_can_send_to_service();
#else
    comm_was_initialized = true;
#endif

    if (comm_was_initialized)
        sensor_poll();

    // Report any UART errors, but only after comm_poll had a chance to check
    serial_uart_error_check(false);

}

// Initialize our app timers
void timer_init() {

    // Init the completed task scheduler that lets us handle command
    // processing outside the interrupt handlers, and instead via app_sched_execute()
    // called from the main loop in main.c.
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);

    // Init app timer support for the entire app, in a way such that it utilizes
    // the app scheduler.  We do this so that we don't execute our (sometimes-lengthy)
    // timer functions at interrupt time, but rather via the app scheduler.
    APP_TIMER_APPSH_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, true);

    // If faking GPS time for debugging, set a timestamp for testing purposes
#ifdef FAKEGPSTIME
    set_timestamp(201116, 123456);
#endif

}

// Start our primary app timer
void timer_start() {

    // Create and start our primary app timer
    app_timer_create(&teletype_timer, APP_TIMER_MODE_REPEATED, teletype_timer_handler);
    app_timer_start(teletype_timer, TELETYPE_TIMER_INTERVAL, NULL);

}
