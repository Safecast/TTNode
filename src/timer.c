// App-level timers

#include <string.h>
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

// True if we've ever seen comm having been initialized
static bool have_initialized = false;

// Access to our app-maintained system clock
uint32_t get_seconds_since_boot() {
    return (seconds_since_boot);
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
        sprintf(message, "%lu alive for %lum %lus", io_get_device_address(), mins, secs - mins * 60);
        phone_send(message);

        // Flag that we do NOT want to optimize power by shutting down
        // listens, even if this connection happens to drop.
        io_power_stay_suboptimal();
    }
}

// Force expiration of all timers
void force_all_timer_expiration() {
    seconds_since_boot += FORCE_EXPIRATION_SECONDS;
    comm_watchdog_reset();
}

// Primary app timer
void teletype_timer_handler(void *p_context) {

    // Bump the number of seconds since boot.
    // It's up to the users to anticipate overflow.
    seconds_since_boot += TELETYPE_TIMER_SECONDS;

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

    // If we've initialized at least once, poll and advance our sensor state machine
#if defined(LORA) || defined(CELLX)
    if (!have_initialized)
        have_initialized = comm_mode() != COMM_NONE && comm_can_send_to_service();
#else
    have_initialized = true;
#endif

    if (have_initialized)
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

}

// Start our primary app timer
void timer_start() {

    // Create and start our primary app timer
    app_timer_create(&teletype_timer, APP_TIMER_MODE_REPEATED, teletype_timer_handler);
    app_timer_start(teletype_timer, TELETYPE_TIMER_INTERVAL, NULL);

}
