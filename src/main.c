// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

// Main TTNode init & scheduling loop

#include "debug.h"
#include "config.h"
#include "comm.h"
#include "timer.h"
#include "bt.h"
#include "io.h"
#include "gpio.h"
#include "serial.h"
#include "storage.h"
#include "softdevice_handler.h"
#include "app_scheduler.h"

#if !defined(NSDKV10) && !defined(NSDKV11)
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#endif

// Primary power-sensitive work dispatcher
void power_manage(void) {

#ifdef NRF52
    // From nRF5 SDK v11.0.0 release_notes.txt Release Date: Week 10, 2016
    // Found in our code because the very first TWI transaction caused the
    // FPU exception, thus causing massive battery drain.
    // The release_notes say:
    // - When the FPU is in use, it triggers the FPU_IRQn interrupt when one of
    //   the six exception flags (IDC, IXC, UFC, OFC, DZC, IOC) is set.
    //   The FPU interrupt will always set the pending flag (even if the
    //   interrupt is not enabled), irrespective of whether the user is
    //   interested in the exception bit.
    //   The pending flag then prevents the SoftDevice from going into low
    //   power mode when sd_app_evt_wait() is called.
    //   Therefore, always clear the exception bits and the pending interrupt
    //   before calling sd_app_evt_wait().
#define FPU_EXCEPTION_MASK 0x0000009F
    __set_FPSCR(__get_FPSCR()  & ~(FPU_EXCEPTION_MASK));
    (void) __get_FPSCR();
    NVIC_ClearPendingIRQ(FPU_IRQn);
#endif

#if defined(SCHEDDEBUG)
#ifdef LED_PIN_YEL
    gpio_pin_set(LED_PIN_YEL, true);
#endif
#ifdef LED_PIN_RED
    gpio_pin_set(LED_PIN_RED, false);
#endif
#endif

    sd_app_evt_wait();

#if defined(SCHEDDEBUG)
#ifdef LED_PIN_YEL
    gpio_pin_set(LED_PIN_YEL, false);
#endif
#ifdef LED_PIN_RED
    gpio_pin_set(LED_PIN_RED, true);
#endif
#endif

    // Do work enqueued during interrupt service routines
    app_sched_execute();

#if defined(SCHEDDEBUG)
#ifdef LED_PIN_RED
    gpio_pin_set(LED_PIN_RED, false);
#endif
#ifdef LED_PIN_YEL
    gpio_pin_set(LED_PIN_YEL, false);
#endif
#endif

}

// Application main entry
int main(void) {

    // Init debug flags
    debug_init();
    
    // Init app timers & scheduler, which must be done before other things
    // which use app sched or timers such as serial
    timer_init();

    // Init serial very early, for debugging, at the default baud rate
    serial_init(UART_BAUDRATE_BAUDRATE_Baud57600, false);

    // Init the log package after serial, if we are doing extreme debugging
#if NRF_LOG_ENABLED
    NRF_LOG_INIT(NULL);
#endif

    // Indicate when debugging
#ifdef DEBUG_USES_UART
    DEBUG_PRINTF("\n%s\n%s\n", app_version(), STRINGIZE_VALUE_OF(FIRMWARE));
#endif

    // Init the softdevice before anything else. It's our OS.
    bluetooth_softdevice_init();

    // Initialize persistent storage, which must happen before
    // io and timer init because they rely upon config values
    storage_init();

    // Init misc I/O
    io_init();

    // Init our command state machine
    comm_init();

    // Init bluetooth
    bluetooth_init();

    // Start the app timers
    timer_start();

    // Enter main loop.
    for (;;) {
#if NRF_LOG_ENABLED
        if (NRF_LOG_PROCESS() == false)
            power_manage();
#else
        power_manage();
#endif
        // Ensure that the app timer frequency is up to date
        timer_update_mode();
    }

}
