// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

// Debugging support, customized for debug-via-Bluetooth

#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#include "nrf_delay.h"
#include "nrf_error.h"
#include "app_timer.h"
#include "timer.h"
#include "debug.h"
#include "bt.h"
#include "io.h"
#include "gpio.h"
#include "btdebug.h"

#if defined(LED_RED) && !defined(SCHEDDEBUG)
#define INDICATORS
#endif

static bool init = false;
static bool timer_started = false;
static int recursion = 0;
static uint8_t output_buffer[1024];
static uint16_t output_buffer_fill_next = 0;
static uint16_t output_buffer_drain_next = 0;
static uint16_t output_buffer_used = 0;
static uint16_t timer_debounce_count = 0;

#define BTDEBUG_TIMER_MILLISECONDS          50
#define BTDEBUG_TIMER_DEBOUNCE_MILLISECONDS 2000
#define BTDEBUG_TIMER_INTERVAL APP_TIMER_TICKS(BTDEBUG_TIMER_MILLISECONDS, APP_TIMER_PRESCALER)
APP_TIMER_DEF(btdebug_timer);

void btdebug_send_byte(uint8_t databyte) {
    char strbuf[2];
    strbuf[0] = databyte;
    strbuf[1] = '\0';
    btdebug_send_string(strbuf);
}

void btdebug_send_string(char *str) {

    // Exit if not yet initialized
    if (!init)
        return;

    // Short circuit all of this if we can't even send to BT
    if (!can_send_to_bluetooth())
        return;

    // Send welcome message immediately (yes, this goes recursive)
    welcome_message();

    // If we're optimizing power, we don't even want to come down this path
    // because we're wasting time.  Just swallow the character.
    if (io_optimize_power())
        return;

    // This is completely defensive programming.  There is no way that
    // we should ever go recursive, but this is for future-proofing.
    if (recursion++ != 0) {
        --recursion;
        return;
    }

    // Loop over input chars
    while (*str != '\0') {

        // Exit if output buffer overrun
        if (output_buffer_used >= sizeof(output_buffer)) {
            break;
        }

        // Append to output buffer
        output_buffer[output_buffer_fill_next++] = *str++;
        if (output_buffer_fill_next >= sizeof(output_buffer))
            output_buffer_fill_next = 0;
        output_buffer_used++;

    }

    // If we need the timer, start it.
    if (output_buffer_used != 0 && !timer_started) {
        timer_started = true;
        app_timer_start(btdebug_timer, BTDEBUG_TIMER_INTERVAL, NULL);
#ifdef INDICATORS
        if (!gpio_indicators_are_active())
            gpio_pin_set(LED_PIN_RED, true);
#endif
    }

    // Done
    --recursion;

}

// Timer
void btdebug_timer_handler(void *p_context) {

    // If nothing is left after debouncing, shut down the timer
    if (output_buffer_used == 0) {

        // Exit if we're still debouncing
        if (timer_debounce_count != 0)
            if (--timer_debounce_count != 0)
                return;

        // Disable the timer
        app_timer_stop(btdebug_timer);
#ifdef INDICATORS
        if (!gpio_indicators_are_active())
            gpio_pin_set(LED_PIN_RED, false);
#endif
        timer_started = false;
        return;

    }

    // Since we've got something to output, reset the debounce timer
    timer_debounce_count = BTDEBUG_TIMER_DEBOUNCE_MILLISECONDS / BTDEBUG_TIMER_MILLISECONDS;

    // Drain until the earlier of empty or asked to pause
    while (output_buffer_used) {

        // Grab the next byte
        uint8_t databyte = output_buffer[output_buffer_drain_next++];
        if (output_buffer_drain_next >= sizeof(output_buffer))
            output_buffer_drain_next = 0;
        output_buffer_used--;

        // Send it, but (and this is defensive coding) don't allow any
        // output to be appended while we are inside the bluetooth subsystem.
        recursion++;
        bool fPause = send_byte_to_bluetooth(databyte);
        recursion--;

        // Force a pause if instructed to do so
        if (!fPause)
            break;

    }

}

// One-time init
void btdebug_create_timer(void) {
    app_timer_create(&btdebug_timer, APP_TIMER_MODE_REPEATED, btdebug_timer_handler);
    init = true;
}
