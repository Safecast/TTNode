// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

// GPIO support

#ifdef GEIGERX
#define ENABLE_GPIOTE
#endif

#include <stdint.h>
#include "debug.h"
#include "nrf.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "boards.h"
#include "storage.h"
#include "config.h"
#include "serial.h"
#include "timer.h"
#include "sensor.h"
#include "misc.h"
#include "stats.h"
#include "io.h"
#include "gpio.h"

#ifdef ENABLE_GPIOTE
#include "nrf_gpiote.h"
#include "nrf_drv_gpiote.h"
#include "app_gpiote.h"
#endif

#ifdef MOTIONX
#include "comm.h"
#endif

#ifdef GEIGERX
#include "geiger.h"
#endif

#if defined(LED_COLOR) && !defined(BOOTLOADERX) && !defined(POWERDEBUG)
#define INDICATORS
#endif

// Maximum number of app users of the GPIOTE handler
#ifdef ENABLE_GPIOTE
#define APP_GPIOTE_MAX_USERS        1
static app_gpiote_user_id_t m_gpiote_user_id;
#endif

// Power debugging
static bool fSensorPowerForceOn = false;
static bool fSensorPowerForceOff = false;

// GPIO initialization info
#ifdef GEIGERX
#if defined(NSDKV10) || defined(NSDKV11) || defined(NSDKV121)
static uint32_t m_geiger0_low_to_high_mask = 0;
static uint32_t m_geiger1_low_to_high_mask = 0;
static uint32_t m_geigers_low_to_high_mask = 0;
static uint32_t m_geigers_high_to_low_mask = 0;
#else
static uint32_t m_geiger0_low_to_high_mask[GPIO_COUNT] = {0};
static uint32_t m_geiger1_low_to_high_mask[GPIO_COUNT] = {0};
static uint32_t m_geigers_low_to_high_mask[GPIO_COUNT] = {0};
static uint32_t m_geigers_high_to_low_mask[GPIO_COUNT] = {0};
#endif
#endif

// Indicator-related
#ifdef INDICATORS
#define BLINK_MILLISECONDS 750
APP_TIMER_DEF(indicator_timer);
static uint32_t indicator_comm_color;
static uint32_t indicator_gps_color;
static uint8_t indicator_toggle = 0;
static bool indicator_app_timer_started = false;
static bool indicator_blinky = false;
static bool indicator_shutdown = false;
static uint32_t indicators_not_needed = 0L;
static bool lastKnown = false;
static bool lastRed;
static bool lastYel;
#endif

// UART-related
static uint16_t last_uart_selected = UART_NONE;

// Configure a pin for input, explicitly with no pull-up or pulldown
void gpio_cfg_input(uint16_t pin) {
    nrf_gpio_cfg_input(pin,  NRF_GPIO_PIN_NOPULL);
}

// Configure a pin for output
void gpio_cfg_output(uint16_t pin) {
    nrf_gpio_cfg_output(pin);
#ifdef DEBUGGPIO
    DEBUG_PRINTF("GPIO Pin %d now configured to state %d\n", nrf_gpio_pin_out_read(pin));
#endif
}

// Init power management for a pin
void gpio_power_init (uint16_t pin, bool fEnable) {
    gpio_cfg_output(pin);
    gpio_power_set(pin, fEnable);
}

// Turn a pin on or off
void gpio_pin_set (uint16_t pin, bool fOn) {
    if (fOn) {
        nrf_gpio_pin_set(pin);
    } else {
        nrf_gpio_pin_clear(pin);
    }
#ifdef DEBUGGPIO
    DEBUG_PRINTF("GPIO pin %d now state %d\n", nrf_gpio_pin_out_read(pin));
#endif
}

// Power Debug Modes
void gpio_power_debug_mode (bool fForceOn, bool fForceOff) {
    fSensorPowerForceOn = fForceOn;
    fSensorPowerForceOff = fForceOff;
}

// Turn power on or off for a pin
void gpio_power_set (uint16_t pin, bool fOn) {

    // If we're debugging power, just hammer everything modified
    // below to be ON.
#ifdef POWERDEBUG_OUTPUTS_ON
    gpio_pin_set(pin, true);
    gpio_cfg_output(POWER_PIN_PS_5V);
    gpio_pin_set(POWER_PIN_PS_5V, true);
    gpio_cfg_output(POWER_PIN_PS_BAT);
    gpio_pin_set(POWER_PIN_PS_BAT, true);
    gpio_cfg_output(POWER_PIN_TWI);
    gpio_pin_set(POWER_PIN_TWI, true);
    return;
#endif

    // Debug modes to force the power of sensors on or off
    if (fOn && fSensorPowerForceOff)
        return;
    if (!fOn && fSensorPowerForceOn)
        return;

    // On solarcast, turn on or off the _PS_ pins based on those pins that depend upon them
#ifdef scv1
    static uint32_t pins_enabled = 0L;  // All are cleared in gpio_init()
    static uint32_t pin_mask_5V = POWER_PINS_REQUIRING_PS_5V;
    static uint32_t pin_mask_BAT = POWER_PINS_REQUIRING_PS_BAT;
    static uint32_t pin_mask_TWI = POWER_PINS_REQUIRING_TWI;
    uint32_t pin_mask_before = pins_enabled;

    // Remember the state of which pins are powered on
    if (fOn)
        pins_enabled |=  (1 << pin);
    else
        pins_enabled &= ~(1 << pin);

    // Do special processing where, in order to deliver power, more than one pin is necessary
    if ((pin_mask_before & pin_mask_TWI) == 0 && (pins_enabled & pin_mask_TWI) != 0) {
        gpio_pin_set(POWER_PIN_TWI, true);
    } else if ((pin_mask_before & pin_mask_TWI) != 0 && (pins_enabled & pin_mask_TWI) == 0) {
        gpio_pin_set(POWER_PIN_TWI, false);
    }
    if ((pin_mask_before & pin_mask_5V) == 0 && (pins_enabled & pin_mask_5V) != 0) {
        gpio_cfg_output(POWER_PIN_PS_5V);
        gpio_pin_set(POWER_PIN_PS_5V, true);
    } else if ((pin_mask_before & pin_mask_5V) != 0 && (pins_enabled & pin_mask_5V) == 0) {
        gpio_pin_set(POWER_PIN_PS_5V, false);
        gpio_cfg_input(POWER_PIN_PS_5V);
    }
    if ((pin_mask_before & pin_mask_BAT) == 0 && (pins_enabled & pin_mask_BAT) != 0) {
        gpio_cfg_output(POWER_PIN_PS_BAT);
        gpio_pin_set(POWER_PIN_PS_BAT, true);
    } else if ((pin_mask_before & pin_mask_BAT) != 0 && (pins_enabled & pin_mask_BAT) == 0) {
        gpio_pin_set(POWER_PIN_PS_BAT, false);
        gpio_cfg_input(POWER_PIN_PS_BAT);
    }

#endif // scv1

    // Turn the actual power pin on or off
    gpio_pin_set(pin, fOn);
}

// See if the power overcurrent has been detected
bool gpio_power_overcurrent_sensed() {
#if defined(OCSENSE) && defined(SENSE_PIN_OVERCURRENT)
    return (nrf_gpio_pin_read(SENSE_PIN_OVERCURRENT) == 0);
#else
    return false;
#endif
}

// Handle motion detection
bool gpio_motion_sense(uint16_t command) {
#if !defined(MOTIONX)
    return false;
#else
    static bool fMotionArmed = false;
    static bool fMotionSensed = false;
    static uint32_t MotionDetectedTime = 0L;

    switch (command) {

        // Arm the motion detection
    case MOTION_ARM:
        // If (and ONLY if) there had previously been motion sensed,
        // refresh the GPS position.
        if (fMotionSensed) {
            // If we're in a mode supporting auto-motion, switch operating mode
            if (storage()->wan == WAN_FONA_PLUS_MOBILE)
                sensor_set_op_mode(OPMODE_NORMAL);
            // Update the GPS plus stats
            comm_gps_update();
            stats()->motion_events++;
        } else if (!fMotionArmed) {
            DEBUG_PRINTF("Motion sensing NOW ARMED\n");
        }
        // Prepare for sensing
        fMotionSensed = false;
        fMotionArmed = true;
        break;

        // Disarm so that we won't pay any attention to motion
    case MOTION_DISARM:
        fMotionArmed = false;
        break;

    case MOTION_QUERY_PIN:
#if defined(SENSE_PIN_MOTION) && defined(TWILIS3DH)
        return(nrf_gpio_pin_read(SENSE_PIN_MOTION));
#else
        return(false);
#endif

        // Update the state of the motion sensing, based on the motion pin
    case MOTION_UPDATE:
        if (fMotionArmed) {
            bool fMotionNowSensed = false;

            // Sense the state of the pin if we're configured for it
            fMotionNowSensed = gpio_motion_sense(MOTION_QUERY_PIN);
#ifdef MOTIONDEBUGMAX
            if (fMotionNowSensed && debug(DBG_SENSOR))
                DEBUG_PRINTF("Motion detected by GPIO\n");
#endif

            // If we've sensed motion, refresh the debounce timer
            if (fMotionNowSensed)
                MotionDetectedTime = get_seconds_since_boot();

            // Handle state changes
            if (!fMotionSensed && fMotionNowSensed) {

                // Remember that it was sensed
                fMotionSensed = true;
                DEBUG_PRINTF("MOTION SENSED\n");

                // If we're in a mode supporting auto-motion, set operating mode
                if (storage()->wan == WAN_FONA_PLUS_MOBILE)
                    sensor_set_op_mode(OPMODE_MOBILE);

            } else if (fMotionSensed && !fMotionNowSensed) {

                // Debounce.  When it truly has stabilized, re-arm.
                if (!ShouldSuppress(&MotionDetectedTime, MOTION_STABLE_MINUTES * 60L))
                    gpio_motion_sense(MOTION_ARM);
            }

        }
        break;

        // See if motion has occurred
    case MOTION_QUERY:
        if (!fMotionArmed)
            return false;
        break;

    }
    return fMotionSensed;
#endif
}

// Event handler that handles our GPIO interrupt events
#ifdef ENABLE_GPIOTE

#if defined(NSDKV10) || defined(NSDKV11) || defined(NSDKV121)

void gpiote_event_handler (uint32_t event_pins_low_to_high, uint32_t event_pins_high_to_low) {
#ifdef GEIGERX
    if ((event_pins_low_to_high & m_geiger0_low_to_high_mask) != 0)
        geiger0_event();
    if ((event_pins_low_to_high & m_geiger1_low_to_high_mask) != 0)
        geiger1_event();
#endif
}

#else

void gpiote_event_handler (uint32_t const *event_pins_low_to_high, uint32_t const *event_pins_high_to_low) {
#ifdef GEIGERX
    if ((event_pins_low_to_high[0] & m_geiger0_low_to_high_mask[0]) != 0)
        geiger0_event();
    if ((event_pins_low_to_high[0] & m_geiger1_low_to_high_mask[0]) != 0)
        geiger1_event();
#endif
}

#endif  // SDK version

#endif  // ENABLE_GPIOTE

// Primary app timer
#ifdef INDICATORS
void indicator_timer_handler(void *p_context) {
    uint32_t color, mask;

    if (indicator_toggle == 1) {
        indicator_toggle = 0;
        mask = 0x01010100;
    } else {
        indicator_toggle = 1;
        mask = 0x02020200;
    }

    // Give priority to the COMM color unless it's still unknown
    if (indicator_blinky)
        color = BOTH_ALTERNATE;
    else if ((indicator_comm_color & ~MODE) != BLACK && (indicator_comm_color & ~MODE) != BOTH_SOLID)
        color = indicator_comm_color;
    else
        color = indicator_gps_color;
    color &= mask;

    // Turn on or off the LEDs based on the mask, efficiently
    bool newRed = (color & 0x0000ff00) != 0;
    bool newYel = (color & 0x00ff0000) != 0;
    if (!lastKnown) {
        gpio_pin_set(LED_PIN_RED, newRed);
        gpio_pin_set(LED_PIN_YEL, newYel);
    } else {
        if (newRed != lastRed)
            gpio_pin_set(LED_PIN_RED, newRed);
        if (newYel != lastYel)
            gpio_pin_set(LED_PIN_YEL, newYel);
    }
    lastRed = newRed;
    lastYel = newYel;
    lastKnown = true;


}
#endif

// Turn indicators to a certain state
void gpio_indicate(uint32_t what) {
#ifdef INDICATORS

    // If this is a special request to flash so that we can identify the device, do it
    if (what == INDICATE_BLINKY) {
        indicator_blinky = true;
        if (indicator_app_timer_started)
            app_timer_stop(indicator_timer);
        app_timer_start(indicator_timer, APP_TIMER_TICKS(150, APP_TIMER_PRESCALER), NULL);
        indicator_app_timer_started = true;
        indicator_shutdown = false;
    }

    // Set the color so the next poll changes it
    if ((what & COMM) != 0)
        indicator_comm_color = what;
    if ((what & GPS) != 0)
        indicator_gps_color = what;

    // Turn things off if no longer needed
    if (what == INDICATE_GPS_CONNECTED)
        gpio_indicator_no_longer_needed(GPS);
    else if (what == INDICATE_CELL_CONNECTED || what == INDICATE_LORA_CONNECTED || what == INDICATE_LORAWAN_CONNECTED)
        gpio_indicator_no_longer_needed(COMM);

#endif
}

// Are indicators on?
bool gpio_indicators_are_active() {
#ifdef INDICATORS
    return indicator_app_timer_started;
#else
    return false;
#endif
}

// Turn indicators off
void gpio_indicators_off() {

#ifdef INDICATORS

    if (!indicator_shutdown && !indicator_blinky) {
        indicator_shutdown = true;

        // Turn off the timer, for power savings
        if (indicator_app_timer_started) {
            app_timer_stop(indicator_timer);
            indicator_app_timer_started = false;
        }

        // Turn them off, for power savings
        gpio_pin_set(LED_PIN_RED, false);
        gpio_pin_set(LED_PIN_YEL, false);

        // Indicate high-overhead timer has stopped
        DEBUG_PRINTF("LED shutdown\n");

    }

#endif

}

// Flag that indicator no longer needed
void gpio_indicator_no_longer_needed(uint32_t what) {

#ifdef INDICATORS

    indicators_not_needed |= what;

    // If neither gps nor comms indicators are needed, turn them off
    if ( ((indicators_not_needed & GPS) != 0) && ((indicators_not_needed & COMM) != 0) )
        gpio_indicators_off();

#endif

}

// Which is currently selected?
uint16_t gpio_current_uart() {
    return last_uart_selected;
}

// Get uart name
char *gpio_uart_name(uint16_t which) {
    switch (which) {
    case UART_NONE:
        return "none";
    case UART_LORA:
        return "LORA";
    case UART_FONA:
        return "FONA";
    case UART_PMS:
        return "PMS";
    case UART_GPS:
        return "GPS";
    }
    return "?";
}

// UART Selector
void gpio_uart_select(uint16_t which) {
    uint32_t speed = UART_BAUDRATE_BAUDRATE_Baud57600;
    bool hwfc = HWFC;
    uint16_t prev_uart_selected = last_uart_selected;
    last_uart_selected = which;

#ifdef DEBUGSELECT
    DEBUG_PRINTF("UART SELECT %s\n", gpio_uart_name(which));
#endif

    // Deconfigure the UART on the Nordic driver
    serial_init(0, false);

    // Allow settling
    nrf_delay_ms(500);

    // Power-off modules as appropriate
#if defined(LORA) && defined(POWER_PIN_LORA)
    if (which != UART_LORA)
        gpio_power_set(POWER_PIN_LORA, false);
#endif
#ifdef CELLX
    if (which != UART_FONA)
        gpio_power_set(POWER_PIN_CELL, false);
#endif
#ifdef UGPS
    if (which != UART_GPS) {
        // Note that we NEVER turn off the GPS while in mobile mode,
        // so we don't lose our fix.
        if (sensor_op_mode() != OPMODE_MOBILE)
            gpio_power_set(POWER_PIN_GPS, false);
    }
#endif

    // Disable all serial input coming through the mux
#ifdef USX
    gpio_pin_set(UART_DESELECT, true);
#endif

    // Select the appropriate port on the (still-disabled) uart mux
#ifdef LORA
    if (which == UART_LORA) {
        hwfc = HWFC;
        speed = UART_BAUDRATE_BAUDRATE_Baud57600;
#if defined(USX) && defined(USLORA)
        gpio_pin_set(UART_SELECT_A, (UART_SELECT_PIN_A & USLORA) != 0);
        gpio_pin_set(UART_SELECT_B, (UART_SELECT_PIN_B & USLORA) != 0);
#endif
    }
#endif
#ifdef CELLX
    if (which == UART_FONA) {
        hwfc = HWFC;
        speed = UART_BAUDRATE_BAUDRATE_Baud9600;
#if defined(USX) && defined(USFONA)
        gpio_pin_set(UART_SELECT_A, (UART_SELECT_PIN_A & USFONA) != 0);
        gpio_pin_set(UART_SELECT_B, (UART_SELECT_PIN_B & USFONA) != 0);
#endif
    }
#endif
#if defined(PMSX) && PMSX==IOUART
    if (which == UART_PMS) {
        speed = UART_BAUDRATE_BAUDRATE_Baud9600;
        hwfc = false;
#if defined(USX) && defined(USPMS)
        gpio_pin_set(UART_SELECT_A, (UART_SELECT_PIN_A & USPMS) != 0);
        gpio_pin_set(UART_SELECT_B, (UART_SELECT_PIN_B & USPMS) != 0);
#endif
    }
#endif
#ifdef UGPS
    if (which == UART_GPS) {
        speed = UART_BAUDRATE_BAUDRATE_Baud9600;
        hwfc = false;
#if defined(USX) && defined(USGPS)
        gpio_pin_set(UART_SELECT_A, (UART_SELECT_PIN_A & USGPS) != 0);
        gpio_pin_set(UART_SELECT_B, (UART_SELECT_PIN_B & USGPS) != 0);
#endif
    }
#endif

    // Re-enable comms
    if (which != UART_NONE) {

        // Initialize the UART
        serial_init(speed, hwfc);

        // Allow serial traffic to stabilize after selecting speed
        nrf_delay_ms(1000);

        // Enable the uart mux, which starts data flowing
#ifdef USX
        gpio_pin_set(UART_DESELECT, false);
#endif

    }

    // Power-on modules as appropriate
#if defined(LORA) && defined(POWER_PIN_LORA)
    if (which == UART_LORA)
        gpio_power_set(POWER_PIN_LORA, true);
#endif
#ifdef CELLX
    if (which == UART_FONA)
        gpio_power_set(POWER_PIN_CELL, true);
#endif
#ifdef UGPS
    if (which == UART_GPS)
        gpio_power_set(POWER_PIN_GPS, true);
#endif

    // Allow a stabilization period before we start transmitting to it
    nrf_delay_ms(500);

    // Clear UART error count
    serial_uart_error_check(true);

    // Indicate what we just selected
    if (prev_uart_selected != UART_NONE || last_uart_selected != UART_NONE)
        DEBUG_PRINTF("UART from %s to %s\n", gpio_uart_name(prev_uart_selected), gpio_uart_name(last_uart_selected));

}

// Initialize everything related to GPIO
void gpio_init() {

#ifdef ENABLE_GPIOTE
    uint32_t err_code;

    APP_GPIOTE_INIT(APP_GPIOTE_MAX_USERS);

#ifdef GEIGERX

#if defined(NSDKV10) || defined(NSDKV11) || defined(NSDKV121)
    m_geiger0_low_to_high_mask = (1L << PIN_GEIGER0);
    m_geiger1_low_to_high_mask = (1L << PIN_GEIGER1);
    m_geigers_low_to_high_mask = m_geiger0_low_to_high_mask | m_geiger1_low_to_high_mask;
#else
    m_geiger0_low_to_high_mask[0] = (1L << PIN_GEIGER0);
    m_geiger1_low_to_high_mask[0] = (1L << PIN_GEIGER1);
    m_geigers_low_to_high_mask[0] = m_geiger0_low_to_high_mask[0] | m_geiger1_low_to_high_mask[0];
#endif

    nrf_gpio_cfg_input(PIN_GEIGER0,  NRF_GPIO_PIN_NOPULL);
    nrf_gpio_cfg_input(PIN_GEIGER1,  NRF_GPIO_PIN_NOPULL);

    nrf_gpio_cfg_sense_input(PIN_GEIGER0,
                             NRF_GPIO_PIN_NOPULL,
                             NRF_GPIO_PIN_SENSE_HIGH);
    nrf_gpio_cfg_sense_input(PIN_GEIGER1,
                             NRF_GPIO_PIN_NOPULL,
                             NRF_GPIO_PIN_SENSE_HIGH);

#endif // GEIGERX

    err_code = app_gpiote_user_register(&m_gpiote_user_id,
                                        m_geigers_low_to_high_mask,
                                        m_geigers_high_to_low_mask,
                                        gpiote_event_handler);
    DEBUG_CHECK(err_code);

    err_code = app_gpiote_user_enable(m_gpiote_user_id);
    DEBUG_CHECK(err_code);

#endif // ENABLE_GPIOTE

    // Initialize LEDs
#ifdef LED_COLOR
    gpio_power_init(LED_PIN_RED, false);
    gpio_power_init(LED_PIN_YEL, false);
#endif

    // Init the LED indicator support
#ifdef INDICATORS
#if defined(LORA) || defined(FONA) || defined(TWIUBLOXM8)
    indicator_toggle = 0;
    indicator_shutdown = false;
    gpio_power_init(LED_PIN_RED, true);
    gpio_power_init(LED_PIN_YEL, true);
    gpio_indicate(INDICATE_GPS_STATE_UNKNOWN);
    gpio_indicate(INDICATE_COMMS_STATE_UNKNOWN);
    app_timer_create(&indicator_timer, APP_TIMER_MODE_REPEATED, indicator_timer_handler);
    app_timer_start(indicator_timer, APP_TIMER_TICKS(BLINK_MILLISECONDS, APP_TIMER_PRESCALER), NULL);
    indicator_app_timer_started = true;
#else
    indicator_shutdown = true;
#endif
#endif

    // Init power sensor
#ifdef SENSE_PIN_OVERCURRENT
    nrf_gpio_cfg_input(SENSE_PIN_OVERCURRENT, NRF_GPIO_PIN_NOPULL);
#endif

    // Init motion sensor
#ifdef SENSE_PIN_MOTION
    nrf_gpio_cfg_input(SENSE_PIN_MOTION, NRF_GPIO_PIN_NOPULL);
#endif

    // Init these as inputs, and thereafter let gpio_power_set decide
    // what state they should be in.
#ifdef scv1
    gpio_cfg_input(POWER_PIN_PS_5V);
    gpio_cfg_input(POWER_PIN_PS_BAT);
#endif

    // Init power selectors to the appropriate defaults
#ifdef POWER_PIN_GEIGER
    gpio_power_init(POWER_PIN_GEIGER, false);
#endif
#ifdef POWER_PIN_GPS
    gpio_power_init(POWER_PIN_GPS, false);
#endif
#ifdef POWER_PIN_TWI
    gpio_power_init(POWER_PIN_TWI, false);
#endif
#ifdef POWER_PIN_CELL
    gpio_power_init(POWER_PIN_CELL, false);
#endif
#ifdef POWER_PIN_LORA
    gpio_power_init(POWER_PIN_LORA, false);
#endif
#ifdef POWER_PIN_AIR
    gpio_power_init(POWER_PIN_AIR, false);
#endif
#ifdef POWER_PIN_ROCK
    gpio_power_init(POWER_PIN_ROCK, false);
#endif

    // Init UART selector, and set it to talk to the "null" input via UART_NONE
#ifdef UART_DESELECT
    gpio_cfg_output(UART_DESELECT);
#endif
#ifdef UART_SELECT_A
    gpio_cfg_output(UART_SELECT_A);
#endif
#ifdef UART_SELECT_B
    gpio_cfg_output(UART_SELECT_B);
#endif
    gpio_uart_select(UART_NONE);

}
