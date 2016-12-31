//
// TT Bootloader
//

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "boards.h"
#include "nrf_delay.h"
#include "nrf_mbr.h"
#include "nrf_bootloader.h"
#include "nrf_bootloader_app_start.h"
#include "nrf_dfu.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "app_error.h"
#include "app_error_weak.h"
#include "nrf_bootloader_info.h"
#include "app_scheduler.h"
#include "app_timer_appsh.h"

#ifdef DFUFONA
#include "fona-dfu.h"
#endif

// Function for initialization of LEDs, used for DFU feedback
#ifndef DFUFONA

static void leds_init(void) {
    nrf_gpio_range_cfg_output(LED_START, LED_STOP);
    nrf_gpio_pins_clear(LEDS_MASK);
}

// For debugging of the bootloader with the standard button-based DFU
static void buttons_init(void) {
    #ifndef FONA
    nrf_gpio_cfg_sense_input(BOOTLOADER_BUTTON,
                             BUTTON_PULL,
                             NRF_GPIO_PIN_SENSE_LOW);
    #endif
}

#endif

// Main bootloader entry
int main(void) {
    uint32_t ret_val;

    // Init the log package
    (void) NRF_LOG_INIT(NULL);
    NRF_LOG_INFO("TTBOOT: Inside main!\r\n");

    // Initialize our fona transport
#ifdef DFUFONA
    fona_dfu_init();
#endif

    // Init other board stuff for compatibility
#ifndef DFUFONA
    leds_init();
    buttons_init();
#endif

    NRF_LOG_INFO("TTBOOT: About to init bootloader\r\n");

    ret_val = nrf_bootloader_init();
    APP_ERROR_CHECK(ret_val);

    // Either there was no DFU functionality enabled in this project or the DFU module detected
    // no ongoing DFU operation and found a valid main application.
    // Boot the main application.
    NRF_LOG_INFO("TTBOOT: No DFU requested, so starting main application\r\n");
    nrf_bootloader_app_start(MAIN_APPLICATION_START_ADDR);

    // Should never be reached.
    NRF_LOG_INFO("TTBOOT: Can't happen\r\n");
}
