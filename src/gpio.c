// GPIO support

#ifdef GEIGER
#define ENABLE_GPIOTE
#endif

#include <stdint.h>
#include "debug.h"
#include "nrf.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "boards.h"
#include "softdevice_handler.h"
#include "storage.h"
#include "config.h"
#include "serial.h"
#include "timer.h"
#include "sensor.h"
#include "misc.h"
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

#ifdef GEIGER
#include "geiger.h"
#endif

// Maximum number of app users of the GPIOTE handler
#ifdef ENABLE_GPIOTE
#define APP_GPIOTE_MAX_USERS        1
static app_gpiote_user_id_t m_gpiote_user_id;
#endif

// GPIO initialization info
#ifdef GEIGER
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
#ifdef LED_COLOR
#define BLINK_MILLISECONDS 750
APP_TIMER_DEF(indicator_timer);
static uint32_t indicator_comm_color;
static uint32_t indicator_gps_color;
static uint8_t indicator_toggle = 0;
static bool indicator_shutdown = false;
static uint32_t indicators_not_needed = 0L;
static bool lastKnown = false;
static bool lastRed;
static bool lastYel;
#endif

// UART-related
static uint16_t last_uart_selected = UART_NONE;

// Init power management for a pin
void gpio_power_init (uint16_t pin, bool fEnable) {
    nrf_gpio_cfg_output(pin);
    gpio_power_set(pin, fEnable);
}

// Turn a pin on or off
void gpio_pin_set (uint16_t pin, bool fOn) {
    if (fOn) {
        nrf_gpio_pin_set(pin);
    } else {
        nrf_gpio_pin_clear(pin);
    }
}

// Turn power on or off for a pin
void gpio_power_set (uint16_t pin, bool fOn) {
    gpio_pin_set(pin, fOn);
}

// See if the power is enabled to all sensors on the devic
bool gpio_power_sensed() {
#if defined(POWERSENSE) && defined(SENSE_POWER_PIN)
    return (nrf_gpio_pin_read(SENSE_POWER_PIN) != 0);
#else
    return true;
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
            DEBUG_PRINTF("Motion sensed will cause GPS to be refreshed!\n");
            comm_gps_update();
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
#if defined(SENSE_MOTION_PIN) && defined(TWILIS3DH)
        return(nrf_gpio_pin_read(SENSE_MOTION_PIN));
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
#ifdef GEIGER
    if ((event_pins_low_to_high & m_geiger0_low_to_high_mask) != 0)
        geiger0_event();
    if ((event_pins_low_to_high & m_geiger1_low_to_high_mask) != 0)
        geiger1_event();
#endif
}

#else

void gpiote_event_handler (uint32_t const *event_pins_low_to_high, uint32_t const *event_pins_high_to_low) {
#ifdef GEIGER
    if ((event_pins_low_to_high[0] & m_geiger0_low_to_high_mask[0]) != 0)
        geiger0_event();
    if ((event_pins_low_to_high[0] & m_geiger1_low_to_high_mask[0]) != 0)
        geiger1_event();
#endif
}

#endif  // SDK version

#endif  // ENABLE_GPIOTE

// Primary app timer
#ifdef LED_COLOR
void indicator_timer_handler(void *p_context) {
    uint32_t color, mask;

    if (indicator_toggle == 1) {
        indicator_toggle = 0;
        mask = 0x01010100;
    } else {
        indicator_toggle = 1;
        mask = 0x02020200;
    }

    // Give priority to the COMM color
    if ((indicator_comm_color & ~MODE) != BLACK)
        color = indicator_comm_color;
    else
        color = indicator_gps_color;
    color &= mask;

    // Turn on or off the LEDs based on the mask, efficiently
    bool newRed = (color & 0x0000ff00) != 0;
    bool newYel = (color & 0x00ff0000) != 0;
    if (!gpio_power_sensed()) {
        gpio_pin_set(LED_PIN_RED, false);
        gpio_pin_set(LED_PIN_YEL, false);
    } else if (!lastKnown) {
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

// Turn indicators off
void gpio_indicate(uint32_t what) {

    // Set the color so the next poll changes it
#ifdef LED_COLOR
    if ((what & COMM) != 0)
        indicator_comm_color = what;
    if ((what & GPS) != 0)
        indicator_gps_color = what;
#endif

}

void gpio_indicators_off() {

#ifdef LED_COLOR

    if (!indicator_shutdown) {
        indicator_shutdown = true;

        // Turn off the timer, for power savings
        app_timer_stop(indicator_timer);

        // Turn them off, for power savings
        nrf_gpio_pin_clear(LED_PIN_RED);
        nrf_gpio_pin_clear(LED_PIN_YEL);
    }

#endif

}

// Flag that indicator no longer needed
void gpio_indicator_no_longer_needed(uint32_t what) {

#ifdef LED_COLOR

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
char *uart_name(uint16_t which) {
    switch (which) {
    case UART_NONE:
        return "none";
    case UART_LORA:
        return "LORA";
    case UART_FONA:
        return "FONA";
    case UART_PMS:
        return "PMS";
    }
    return "?";
}

// UART Selector
void gpio_uart_select(uint16_t which) {
    uint32_t speed = UART_BAUDRATE_BAUDRATE_Baud57600;
    bool hwfc = HWFC;
    uint16_t prev_uart_selected = last_uart_selected;
    last_uart_selected = which;

    // The delay that we'll use here for settling between stages of switching
#define SETTLING_DELAY 100

    // Begin by disabling the UART so nothing confuses the MCU during powerdown
    // or during sequential switching of the two UART select lines
#ifdef UART_SELECT
    gpio_pin_set(UART_DESELECT, true);
#endif

    // Disable power to all that aren't controlled by the sensor package
#if defined(LORA) && defined(POWER_PIN_LORA)
    if (which != UART_LORA)
        gpio_power_set(POWER_PIN_LORA, false);
#endif
#ifdef CELLX
    if (which != UART_FONA)
        gpio_power_set(POWER_PIN_CELL, false);
#endif

    // Select UART and initialize serial
#ifdef LORA
    if (which == UART_LORA) {
        hwfc = HWFC;
        speed = UART_BAUDRATE_BAUDRATE_Baud57600;
#if UART_SELECT
        gpio_pin_set(UART_SELECT0, (UART_SELECT_PIN0 & UART_SELECT_LORA) != 0);
        gpio_pin_set(UART_SELECT1, (UART_SELECT_PIN1 & UART_SELECT_LORA) != 0);
        gpio_pin_set(UART_DESELECT, false);
#endif
    }
#endif
#ifdef CELLX
    if (which == UART_FONA) {
        hwfc = HWFC;
        speed = UART_BAUDRATE_BAUDRATE_Baud9600;
#if UART_SELECT
        gpio_pin_set(UART_SELECT0, (UART_SELECT_PIN0 & UART_SELECT_CELL) != 0);
        gpio_pin_set(UART_SELECT1, (UART_SELECT_PIN1 & UART_SELECT_CELL) != 0);
        gpio_pin_set(UART_DESELECT, false);
#endif
    }
#endif
#if defined(PMSX) && PMSX==IOUART
    if (which == UART_PMS) {
        speed = UART_BAUDRATE_BAUDRATE_Baud9600;
        hwfc = false;
#if UART_SELECT
        gpio_pin_set(UART_SELECT0, (UART_SELECT_PIN0 & UART_SELECT_PMS) != 0);
        gpio_pin_set(UART_SELECT1, (UART_SELECT_PIN1 & UART_SELECT_PMS) != 0);
        gpio_pin_set(UART_DESELECT, false);
#endif
    }
#endif

    // Allow UART to stabilize after selecting it
    nrf_delay_ms(SETTLING_DELAY);

    // Power-on as appropriate
#if defined(LORA) && defined(POWER_PIN_LORA)
    if (which == UART_LORA)
        gpio_power_set(POWER_PIN_LORA, true);
#endif
#ifdef CELLX
    if (which == UART_FONA)
        gpio_power_set(POWER_PIN_CELL, true);
#endif

    // Allow hardware to stabilize before selecting new speed
    nrf_delay_ms(SETTLING_DELAY);

    // Initialize serial I/O on the port
    if (which == UART_NONE)
        serial_init(0, false);
    else
        serial_init(speed, hwfc);

    // Allow serial traffic to stabilize after selecting speed
    nrf_delay_ms(SETTLING_DELAY);

    // Clear UART error count
    serial_uart_error_check(true);

    // Indicate what we just selected
    if (prev_uart_selected != UART_NONE || last_uart_selected != UART_NONE)
        DEBUG_PRINTF("Routing UART from %s to %s\n", uart_name(prev_uart_selected), uart_name(last_uart_selected));

}

// Initialize everything related to GPIO
void gpio_init() {

#ifdef ENABLE_GPIOTE
    uint32_t err_code;

    APP_GPIOTE_INIT(APP_GPIOTE_MAX_USERS);

#ifdef GEIGER

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

    // Make sure that regardless of what the bootloader did, we configure this
    // in a way that doesn't draw any current
#ifdef SPARE_PIN
    nrf_gpio_cfg_input(SPARE_PIN,  NRF_GPIO_PIN_NOPULL);
#endif

#endif // GEIGER

    err_code = app_gpiote_user_register(&m_gpiote_user_id,
                                        m_geigers_low_to_high_mask,
                                        m_geigers_high_to_low_mask,
                                        gpiote_event_handler);
    DEBUG_CHECK(err_code);

    err_code = app_gpiote_user_enable(m_gpiote_user_id);
    DEBUG_CHECK(err_code);

#endif // ENABLE_GPIOTE

    // Init UART selector, and set it to talk to the "null" input via UART_NONE
#ifdef UART_DESELECT
    nrf_gpio_cfg_output(UART_DESELECT);
#endif
#ifdef UART_SELECT0
    nrf_gpio_cfg_output(UART_SELECT0);
#endif
#ifdef UART_SELECT1
    nrf_gpio_cfg_output(UART_SELECT1);
#endif
    gpio_uart_select(UART_NONE);

    // Init the LED indicator support
#ifdef LED_COLOR
#if defined(LORA) || defined(FONA) || defined(TWIUBLOXM8)
    indicator_toggle = 0;
    indicator_shutdown = false;
    gpio_power_init(LED_PIN_RED, true);
    gpio_power_init(LED_PIN_YEL, true);
    gpio_indicate(INDICATE_COMMS_STATE_UNKNOWN);
    app_timer_create(&indicator_timer, APP_TIMER_MODE_REPEATED, indicator_timer_handler);
    app_timer_start(indicator_timer, APP_TIMER_TICKS(BLINK_MILLISECONDS, APP_TIMER_PRESCALER), NULL);
#else
    indicator_shutdown = true;
    gpio_power_init(LED_PIN_RED, false);
    gpio_power_init(LED_PIN_YEL, false);
#endif
#endif

    // Init power sensor
#ifdef SENSE_POWER_PIN
    nrf_gpio_cfg_input(SENSE_POWER_PIN, NRF_GPIO_PIN_NOPULL);
#endif

    // Init motion sensor
#ifdef SENSE_MOTION_PIN
    nrf_gpio_cfg_input(SENSE_MOTION_PIN, NRF_GPIO_PIN_NOPULL);
#endif

    // Init power selectors
#ifdef POWER_PIN_GEIGER
    gpio_power_init(POWER_PIN_GEIGER, false);
#endif
#ifdef POWER_PIN_GPS
    gpio_power_init(POWER_PIN_GPS, false);
#endif
#ifdef POWER_PIN_BASICS
    gpio_power_init(POWER_PIN_BASICS, false);
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

}
