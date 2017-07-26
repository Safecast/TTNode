// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

// App-level timers

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "nrf52.h"
#include "debug.h"
#include "nrf_delay.h"
#include "config.h"
#include "timer.h"
#include "storage.h"
#include "sensor.h"
#include "geiger.h"
#include "comm.h"
#include "gpio.h"
#include "phone.h"
#include "io.h"
#include "serial.h"
#include "bt.h"
#include "btdebug.h"
#include "app_scheduler.h"
#include "app_timer_appsh.h"
#include "stats.h"
#include "misc.h"
#include "ssd.h"

// Primary app-level timers
#define TT_SLOW_TIMER_INTERVAL APP_TIMER_TICKS((TT_SLOW_TIMER_SECONDS*1000), APP_TIMER_PRESCALER)
#define TT_FAST_TIMER_INTERVAL APP_TIMER_TICKS((TT_FAST_TIMER_SECONDS*1000), APP_TIMER_PRESCALER)
APP_TIMER_DEF(tt_timer);

// Primary clock, maintained by the primary app timer
// Initialize non-zero because zero is the default init value of all counters, and we want to look later than that.
static uint32_t seconds_since_boot = 1;
static uint32_t ticks_at_measurement = 0;
static bool tt_fast_timer_mode = false;
static bool tt_request_timer_mode_reset = false;

// Date/time
static uint32_t dt_seconds_since_boot_when_set = 0;
static uint32_t dt_date = 0;
static uint32_t dt_time = 0;

// For convenience, a time display buffer that can be returned to callers
static char timebuf[32];;

// Forwards
void timer_refresh_mode();

// Access to our app-maintained system clock
uint32_t get_seconds_since_boot() {
    uint32_t ticks;

#if defined(NSDKV10) || defined(NSDKV11)
    app_timer_cnt_get(&ticks);
#else
    ticks = app_timer_cnt_get();
#endif

    // If the clock has wrapped, just return the clock in our clock's granularity
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

    // If we're in mobile mode AND we already have it, don't set it again
    // or else it will cause our stamps to be blown away on every resampling
    if (dt_seconds_since_boot_when_set != 0 && sensor_op_mode() == OPMODE_MOBILE)
        return;

    // http://aprs.gids.nl/nmea/#rmc
    // 225446 Time of fix 22:54:46 UTC
    // 191194 Date of fix  19 November 1994
    dt_date = ddmmyy;
    dt_time = hhmmss;
    dt_seconds_since_boot_when_set = get_seconds_since_boot();

    // Just for debugging, so we can see when we actually acquire a timestamp
    uint16_t yr = (ddmmyy % 100) + 2000;
    uint16_t mo = (ddmmyy/100) % 100;
    uint16_t dy = (ddmmyy/10000) % 100;
    uint16_t ss = hhmmss % 100;
    uint16_t mm = (hhmmss/100) % 100;
    uint16_t hh = (hhmmss/10000) % 100;
    DEBUG_PRINTF("%02d/%02d/%02d %02d:%02d:%02d UTC\n", mo, dy, yr, hh, mm, ss);

    // Force a service update so that the new timestamp is uploaded ASAP
    comm_initiate_service_update(false);

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

// Get time since boot
char *time_since_boot() {
    uint32_t secs = get_seconds_since_boot();
    uint32_t mins = secs / 60;
    uint32_t hrs = mins / 60;
    uint32_t days = hrs / 24;
    secs -= mins * 60;
    mins -= hrs * 60;
    hrs -= days * 24;
    timebuf[0] = '\0';
    if (days)
        sprintf(timebuf, "%lud ", days);
    if (hrs)
        sprintf(&timebuf[strlen(timebuf)], "%luh ", hrs);
    if (mins)
        sprintf(&timebuf[strlen(timebuf)], "%lum ", mins);
    sprintf(&timebuf[strlen(timebuf)], "%lus", secs);
    return timebuf;
}

// Send a welcome message to the phone upon connect
void welcome_message(void) {
    static uint32_t btSessionIDLast = 0;
    uint32_t btSessionID = bluetooth_session_id();
    if (btSessionID != btSessionIDLast && can_send_to_bluetooth()) {
        btSessionIDLast = btSessionID;

        // Send a welcome message
#ifdef BURN
#define TESTBUILD " (BURN-IN BUILD)"
#else
#ifndef DFU
#define TESTBUILD " (NON-DFU TEST BUILD)"
#else
#define TESTBUILD ""
#endif
#endif
        DEBUG_PRINTF("%lu alive %s on %s build %s%s\n", io_get_device_address(), time_since_boot(), STRINGIZE_VALUE_OF(FIRMWARE), app_build(), TESTBUILD);

        // Display processor version number
#if 0
        uint32_t nrf_part = NRF_FICR->INFO.PART;
        DEBUG_PRINTF("nrf_part: 0x%08lx\n", nrf_part);
        uint32_t nrf_variant = NRF_FICR->INFO.VARIANT;
        DEBUG_PRINTF("nrf_variant: 0x%08lx\n", nrf_variant);
        uint32_t nrf_package = NRF_FICR->INFO.PACKAGE;
        DEBUG_PRINTF("nrf_package: 0x%08lx\n", nrf_package);
        uint32_t nrf_ram = NRF_FICR->INFO.RAM;
        DEBUG_PRINTF("nrf_ram: 0x%08lx\n", nrf_ram);
        uint32_t nrf_flash = NRF_FICR->INFO.FLASH;
        DEBUG_PRINTF("nrf_flash: 0x%08lx\n", nrf_flash);
#endif

        // Flag that we do NOT want to optimize power by shutting down
        // listens, even if this connection happens to drop.
        io_power_stay_suboptimal();
    }
}

// Primary app timer
void tt_timer_handler(void *p_context) {
    static bool overcurrent = false;
    static uint32_t overcurrent_report = 0;

    // Remember the number of ticks the last time we set seconds_since_boot
    uint32_t ticks;
#if defined(NSDKV10) || defined(NSDKV11)
    app_timer_cnt_get(&ticks);
#else
    ticks = app_timer_cnt_get();
#endif

    // Bump the number of seconds since boot, leaving overflow to be dealt with by users
    seconds_since_boot += tt_fast_timer_mode ? TT_FAST_TIMER_SECONDS : TT_SLOW_TIMER_SECONDS;
    ticks_at_measurement = ticks;

    // Exit if we've somehow gone re-entrant
    static int inside_timer = 0;
    if (inside_timer++ != 0) {
        inside_timer--;
        DEBUG_PRINTF("tt_timer REENTRANCY!\n");
        return;
    }

    // Refresh the timer operating mode, if necessary
    timer_refresh_mode();

    // Update the status of whether or not the device is currently in-motion
    gpio_motion_sense(MOTION_UPDATE);

    // Display handling
#ifdef SSD

    // Turn on the display after boot or if we've detected movement
    if (!ssd1306_active())
        if (get_seconds_since_boot() < (DROP_DISPLAY_MINUTES*60))
            ssd1306_init();

    // Remember when we've first noticed the display having been turned on, for whatever reason
    static uint32_t ssd_enabled = 0;
    if (ssd_enabled == 0 && ssd1306_active())
        ssd_enabled = get_seconds_since_boot();

    // IF we're armed, so we can turn it on again, turn off the display if it has been on for too long
#ifndef SSDKEEPALIVE
    if (gpio_motion_sense(MOTION_QUERY_ARMED) && ssd_enabled != 0)
        if (!ShouldSuppress(&ssd_enabled, DROP_DISPLAY_MINUTES*60)) {
            while (ssd1306_active())
                ssd1306_term();
            ssd_enabled = 0;
        }
#endif
    
#endif

    // Notifiy if overcurrent is sensed
    if (gpio_power_overcurrent_sensed()) {
        overcurrent = true;
        DEBUG_PRINTF("Overcurrent sensed!\n");
    }

    // Report overcurrent only ever 15m
    if (!ShouldSuppress(&overcurrent_report, 60 * 15))
        if (overcurrent) {
            overcurrent = false;
            stats()->overcurrent_events++;
        }

    // Checkpoint deferred NVRAM I/O if serial I/O is not in progress.  In the case of
    // oneshot mode, this means that neither comms nor PMS are using the uart.  In the case
    // of non-oneshot mode where the uart is always busy, this just means when comms is not active.
    if (comm_uart_switching_allowed()) {
        if (gpio_current_uart() == UART_NONE)
            storage_checkpoint();
    } else {
        if (!comm_is_busy())
            storage_checkpoint();
    }

    // Restart if it's been requested
    io_restart_if_requested();

    // Terminate solicitations if we're optimizing power
    static bool fDropped = false;
    if (!fDropped && io_optimize_power()) {
        fDropped = true;
        drop_bluetooth();
    }

    // This is just defensive programming; they should shut down themselves WAY before this
    if (seconds_since_boot > 15*60)
        gpio_indicators_off();

    // Poll geiger counters
#ifdef GEIGERX
    if (tt_fast_timer_mode)
        geiger_poll();
#endif

    // Poll the sensor package BEFORE polling comms, so that if there is anything
    // marked as "completed" by the sensor package it will be immediately communicated
    sensor_poll();

    // Poll and advance our communications state machine
    comm_poll();

    // Say hello if we're just now connecting to BT
    welcome_message();

    // Report any UART errors, but only after comm_poll had a chance to check
    serial_uart_error_check(false);

    // Exit
    inside_timer--;

}

// Initialize our app timers
void timer_init() {

    // Initialize the clock to a small random number of seconds, so that
    // if someone intentionally tries to synchronize the power-on of multiple
    // devices the Lora uploads won't also necessarily be synchronized
    seconds_since_boot += io_get_random(60);

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

    // Create our primary app timer
    app_timer_create(&tt_timer, APP_TIMER_MODE_REPEATED, tt_timer_handler);

    // Create our debug output timer
    btdebug_create_timer();

}

// Start our primary app timer
void timer_start() {

    // Enable the slow timer
    tt_fast_timer_mode = false;
    app_timer_start(tt_timer, TT_SLOW_TIMER_INTERVAL, NULL);

    // Turn on the display immediately if it's available
#ifdef SSD
    ssd1306_init();
#endif

}

// Start or stop timers based on mode
void timer_refresh_mode() {
    bool fast_timer_mode_needed;

    // Determine whether or not we're in 'fast mode' based on mobile mode,
    // which requires the fast clock for geiger sensing.  Otherwise, we'll
    // keep the clock slow for power-savings reasons.
    if (sensor_op_mode() == OPMODE_MOBILE)
        fast_timer_mode_needed = true;
    else
        fast_timer_mode_needed = false;

    // Switch from one clock to the other if we're in the wrong mode
    if (tt_fast_timer_mode != fast_timer_mode_needed) {
        tt_fast_timer_mode = fast_timer_mode_needed;
        tt_request_timer_mode_reset = true;
    };

}

// Process timer change requests from the main scheduling loop, because we can't
// stop or start a timer from within the timer handler itself.
void timer_update_mode() {

    if (tt_request_timer_mode_reset) {
        tt_request_timer_mode_reset = false;
        app_timer_stop(tt_timer);
        app_timer_start(tt_timer, tt_fast_timer_mode ? TT_FAST_TIMER_INTERVAL :  TT_SLOW_TIMER_INTERVAL, NULL);
    }

}
