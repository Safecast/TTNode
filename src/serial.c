// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

// UART I/O support

#include <stdint.h>
#include <string.h>
#include "debug.h"
#include "app_error.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "custom_board.h"
#include "app_uart.h"
#include "serial.h"
#include "gpio.h"

#ifdef LORA
#include "lora.h"
#endif

#ifdef FONA
#include "fona.h"
#endif

#ifdef UGPS
#include "ugps.h"
#endif

#if defined(PMSX) && PMSX==IOUART
#include "pms.h"
#endif

// Disable serial when debugging
#if defined(DEBUG_USES_UART) && !( defined(NSDKV10) || defined(NSDKV11) )
#define DISABLE_UART
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#endif

// Serial-related
#ifdef BOOTLOADERX
#define UART_TX_BUF_SIZE 256
#define UART_RX_BUF_SIZE 1024
#else
#define UART_TX_BUF_SIZE 256
#define UART_RX_BUF_SIZE 256
#endif

static bool fSerialInit = false;
static bool fTransmitDisabled = false;
static bool fHWFC = false;
static uint32_t uart_errors = 0;
static uint32_t uart_init_err_code = 0;

#ifdef SERIALRECEIVEDEBUG
static int debug_total = 0;
static int debug_chars = 0;
static char debug_buff[250] = "";
#endif

// Get state of hwfc
bool serial_hwfc_enabled() {
    return fHWFC;
}

// Get state of transmit
bool serial_transmit_enabled() {
    return(fSerialInit || !fTransmitDisabled);
}

// Temporarily disable or enable transmit
void serial_transmit_enable(bool fEnable) {
    fTransmitDisabled = !fEnable;
}

void serial_send_string(char *str) {
    while (*str != '\0')
        serial_send_byte((uint8_t)(*str++));
    serial_send_byte('\r');
    serial_send_byte('\n');
}

// Transmit a byte to the LPWAN device
void serial_send_byte(uint8_t databyte) {
    bool fTransmitted = false;

    // Exit if not initialized
    if (!fSerialInit)
        return;

    // Exit if we're temporarily disabled because
    // we know that we'll cause a uart error if
    // we try sending to the other device.
    if (fTransmitDisabled)
        return;

#ifndef DISABLE_UART

    // Send it.  Note that if we are linking against the NON-fifo version of app_uart,
    // it is EXPECTED that this will come back with an error if we are busy.
    // As such, we'll do a retry for a reasonable period of time.  We don't want
    // to hang here forever, though, because we might just be transmitting at a time
    // when the target device is not selected by the uart multiplexer.
    int i;
    for (i=0; i<3; i++) {

        // It is expected that we will have material overrun problems on output when
        // tthere is a large block of stuff being output (such as during bootloader).
        // Since there's no app_uart equivalent of nrf_drv_uart_tx_in_progress(), we
        // just do a nominal delay on EVERY byte transmitted. Given how little we
        // transmit to Lora/Fona devices, this is not harmful even in the non-bootloader
        // case.  (Note that at 9600 baud, it takes roughly 1ms to transmit
        // a single character - so this is quite a generous delay at that or faster
        // baud rates.)
        nrf_delay_ms(2);

        // Transmit, and we're done if it's successful
        if (app_uart_put(databyte) == NRF_SUCCESS) {
            fTransmitted = true;
            break;
        }

    }

#endif // DISABLE_UART

    // Debugging
    if (!fTransmitted)
        DEBUG_PRINTF("SSB Error\n");

#if defined(DEBUG_USES_UART) && !( defined(NSDKV10) || defined(NSDKV11) )
    NRF_LOG_RAW_INFO("%c", databyte);
#endif

}

// Check and clear uart errors
bool serial_uart_error_check(bool fClearOnly) {
    bool wereErrors;

    // Silently clear out if that's what is being requested
    if (fClearOnly) {
#ifdef SERIALRECEIVEDEBUG
        debug_buff[0] = '\0';
        debug_chars = 0;
#endif
        uart_errors = 0;
        return false;
    }

    // Debug

#ifdef SERIALRECEIVEDEBUG
    if (debug_buff[0] != '\0') {
        DEBUG_PRINTF("%d %s\n", debug_total, debug_buff);
        debug_buff[0] = '\0';
        debug_chars = 0;
    }
#endif

    // We can't really do much without a UART, so be persistent about reporting
    if (uart_init_err_code != 0)
        DEBUG_PRINTF("UART init error: 0x%04x\n", uart_init_err_code);

    // If no uart is selected, ignore reporting the expected large # of errors
    if (gpio_current_uart() == UART_NONE)
        wereErrors = false;
    else
        wereErrors = (uart_errors > 5);

#ifdef SERIALRECEIVEDEBUG
    if (wereErrors)
        DEBUG_PRINTF("%d more UART(%d) errors\n", uart_errors, gpio_current_uart());
#endif

    uart_errors = 0;
    return(wereErrors);
}

#ifdef SERIALRECEIVEDEBUG
void add_to_debug_log(uint16_t databyte) {
    if (debug_chars >= (sizeof(debug_buff)-2))
        debug_chars = 0;
    if (databyte < ' ')
        databyte = '.';
    debug_buff[debug_chars++] = databyte;
    debug_buff[debug_chars] = '\0';
    debug_total++;
}
#endif

// Handle serial input events
#ifndef DISABLE_UART
void uart_event_handler(app_uart_evt_t *p_event) {
    uint8_t databyte;

    // Exit if we're in the middle of switching uarts
    if (!fSerialInit)
        return;

    switch (p_event->evt_type) {

    case APP_UART_DATA_READY: {
        int i;
        // We're called here at interrupt level, with serial
        // interrupts disabled.  On the one hand, if we only process
        // a single byte per interrupt, we may never catch up
        // if in fact the fifo starts to fill up and we are behind.
        // On the other hand, if we stay here too long (such as a
        // "while app_uart_get()==success"), we could cause significant
        // problems staying in the interrupt handler for too long.
        // This number is chosen as a "reasonable" number that
        // enables us to consume several bytes from the rx queue
        // without hanging us in here for too too long.
        for (i=0; i<4; i++) {
            if (app_uart_get(&databyte) != NRF_SUCCESS)
                break;
#ifdef SERIALRECEIVEDEBUG
            if (gpio_current_uart() != UART_NONE)
                add_to_debug_log(databyte);
#endif
            switch (gpio_current_uart()) {
#if defined(PMSX) && PMSX==IOUART
            case UART_PMS:
                pms_received_byte(databyte);
                break;
#endif
#ifdef LORA
            case UART_LORA:
                lora_received_byte(databyte);
                break;
#endif
#ifdef FONA
            case UART_FONA:
                fona_received_byte(databyte);
                break;
#endif
#ifdef UGPS
            case UART_GPS:
                s_ugps_received_byte(databyte);
                break;
#endif
            }
        }
        break;
    }

    case APP_UART_COMMUNICATION_ERROR:
        if (gpio_current_uart() != UART_NONE)
            uart_errors++;
        break;

    case APP_UART_FIFO_ERROR:
        DEBUG_CHECK(p_event->data.error_code);
        break;

    default:
        break;
    }
}
#endif // DISABLE_UART

// Shut down serial I/O
void serial_term() {
    serial_init(0, false);
}

// Init the serial I/O subsystem
void serial_init(uint32_t speed, bool hwfc) {

    // Close it and re-open it if reinitializing
    if (fSerialInit) {

        // If we are doing uart debugging, don't mess things up by doing a reset or init
#ifdef DEBUG_USES_UART
        return;
#endif

        // Close the UART.
        fSerialInit = false;
        fTransmitDisabled = true;
#ifndef DISABLE_UART
        app_uart_close();
#endif

        // Disable the pins, to ensure there is no leakage path
        gpio_cfg_input(RX_PIN);
        gpio_cfg_input(TX_PIN);
#if HWFC
        gpio_cfg_input(RTS_PIN);
        gpio_cfg_input(CTS_PIN);
#endif

    }

    // If we're just shutting down the UART, exit
    if (speed == 0)
        return;

    // Init
#ifndef DISABLE_UART

    // If not configured for HWFC, turn it off
#if !HWFC
    hwfc = false;
#endif

#if HWFC
    app_uart_comm_params_t comm_params = {
        RX_PIN,
        TX_PIN,
#if ( defined(NSDKV10) || defined(NSDKV11) )
        (uint8_t)RTS_PIN,
        (uint8_t)CTS_PIN,
#else
        RTS_PIN,
        CTS_PIN,
#endif
        APP_UART_FLOW_CONTROL_ENABLED,
        false,                              // NO PARITY
        speed
    };
#else
    app_uart_comm_params_t comm_params = {
        RX_PIN,
        TX_PIN,
#if ( defined(NSDKV10) || defined(NSDKV11) )
        (uint8_t)UART_PIN_DISCONNECTED,
        (uint8_t)UART_PIN_DISCONNECTED,
#else
        UART_PIN_DISCONNECTED,
        UART_PIN_DISCONNECTED,
#endif
        APP_UART_FLOW_CONTROL_DISABLED,
        false,                              // NO PARITY
        speed
    };
#endif
    
    if (!hwfc) {
        comm_params.flow_control = APP_UART_FLOW_CONTROL_DISABLED;
#if defined(NSDKV10) || defined(NSDKV11)
        comm_params.rts_pin_no = (uint8_t) UART_PIN_DISCONNECTED;
        comm_params.rts_pin_no = (uint8_t) UART_PIN_DISCONNECTED;
#else
        comm_params.rts_pin_no = UART_PIN_DISCONNECTED;
        comm_params.cts_pin_no = UART_PIN_DISCONNECTED;
#endif
    }

    // Save this so the higher levels can query it
    fHWFC = hwfc;

    // Init the uart
    app_uart_buffers_t buffer_params;
    app_irq_priority_t priority;
    static uint8_t rx_buf[UART_RX_BUF_SIZE];
    static uint8_t tx_buf[UART_TX_BUF_SIZE];
    buffer_params.rx_buf      = rx_buf;
    buffer_params.rx_buf_size = sizeof (rx_buf);
    buffer_params.tx_buf      = tx_buf;
    buffer_params.tx_buf_size = sizeof (tx_buf);
    priority = APP_IRQ_PRIORITY_LOW;
    uart_init_err_code = app_uart_init(&comm_params, &buffer_params, uart_event_handler, priority);

#ifdef SERIALRECEIVEDEBUG
    char *baudstr, *flowstr;
    switch (comm_params.flow_control) {
    case APP_UART_FLOW_CONTROL_DISABLED:
        flowstr = "disabled";
        break;
    case APP_UART_FLOW_CONTROL_ENABLED:
        flowstr = "enabled";
        break;
    default:
        flowstr = "?";
        break;
    }
    switch (comm_params.baud_rate) {
    case UART_BAUDRATE_BAUDRATE_Baud9600:
        baudstr = "9600";
        break;
    case UART_BAUDRATE_BAUDRATE_Baud19200:
        baudstr = "19200";
        break;
    case UART_BAUDRATE_BAUDRATE_Baud57600:
        baudstr = "57600";
        break;
    case UART_BAUDRATE_BAUDRATE_Baud115200:
        baudstr = "115200";
        break;
    default:
        baudstr = "?";
        break;
    }
    DEBUG_PRINTF("UART=%s hwfc=%s err=%d\n", baudstr, flowstr, uart_init_err_code);
#endif

#endif // DISABLE_UART

    fSerialInit = true;
    fTransmitDisabled = false;

}
