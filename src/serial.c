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
static uint32_t uart_errors = 0;
static uint32_t uart_init_err_code = 0;

#ifdef SERIALRECEIVEDEBUG
static int debug_total = 0;
static int debug_chars = 0;
static char debug_buff[250] = "";
#endif

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

    // Exit if not initialized
    if (!fSerialInit)
        return;

    // Exit if we're temporarily disabled because
    // we know that we'll cause a uart error if
    // we try sending to the other device.
    if (fTransmitDisabled)
        return;

#ifndef DISABLE_UART

    // Send it
    app_uart_put(databyte);

    // We are having serious overrun problems on output when there is a large block
    // of stuff being output (such as during bootloader).  Since there's no app_uart
    // equivalent of nrf_drv_uart_tx_in_progress(), we just do a nominal delay on
    // each byte transmitted.
    nrf_delay_ms(1);

#endif // DISABLE_UART

    // Debugging
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

    switch (p_event->evt_type) {

    case APP_UART_DATA_READY:
        while (app_uart_get(&databyte) == NRF_SUCCESS) {
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
            }
        }
        break;

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

    }

    // If we're just shutting down the UART, exit
    if (speed == 0)
        return;
    
    // Init
#ifndef DISABLE_UART

#if HWFC
    app_uart_comm_params_t comm_params = {
        RX_PIN,
        TX_PIN,
        RTS_PIN,
        CTS_PIN,
        APP_UART_FLOW_CONTROL_ENABLED,
        false,                              // NO PARITY
        speed
    };
#else
    app_uart_comm_params_t comm_params = {
        RX_PIN,
        TX_PIN,
        PIN_UNDEFINED,
        PIN_UNDEFINED,
        APP_UART_FLOW_CONTROL_DISABLED,
        false,                              // No parity check
        speed
    };
#endif

#ifdef HWFC
    if (hwfc)
        comm_params.flow_control = APP_UART_FLOW_CONTROL_ENABLED;
    else
        comm_params.flow_control = APP_UART_FLOW_CONTROL_DISABLED;
#endif

    app_uart_buffers_t buffer_params;
    static uint8_t rx_buf[UART_RX_BUF_SIZE];
    static uint8_t tx_buf[UART_TX_BUF_SIZE];
    buffer_params.rx_buf      = rx_buf;
    buffer_params.rx_buf_size = sizeof (rx_buf);
    buffer_params.tx_buf      = tx_buf;
    buffer_params.tx_buf_size = sizeof (tx_buf);
    uart_init_err_code = app_uart_init(&comm_params, &buffer_params, uart_event_handler, APP_IRQ_PRIORITY_LOW);

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
