// Teletype Bootloader

#include <stdint.h>
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
#include "gpio.h"
#include "serial.h"

// For app scheduler, used by our serial package
#define SCHED_MAX_EVENT_DATA_SIZE       MAX(APP_TIMER_SCHED_EVT_SIZE, 0)
#define SCHED_QUEUE_SIZE                20

// Serial I/O Line Buffers
#define MAXLINE 250
#define LINES 2
static uint16_t linesize[LINES] = {0};
static char linebuf[LINES][MAXLINE];
static bool line_complete = false;
static uint16_t line_completed = 0;
static uint16_t line_building = 0;

// Wait for a reply
#define REPLY_TIMEOUT   0
#define REPLY_1         1
#define REPLY_2         2
#define REPLY_3         3
bool send_and_wait_for_reply(char *cmd, char *r1, char *r2, char *r3) {

#ifdef FONA
    int i;
    int timeout_count = 30;
    int timeout_delay_ms = 100;

    line_complete = false;
    serial_send_string(cmd);

    for (i=timeout_count;; --i) {
        if (i == 0)
            break;

        if (line_complete) {

            if (r1 != NULL)
                if (strcmp(r1, linebuf[line_completed]) == 0)
                    return REPLY_1;

            if (r2 != NULL)
                if (strcmp(r2, linebuf[line_completed]) == 0)
                    return REPLY_2;

            if (r3 != NULL)
                if (strcmp(r3, linebuf[line_completed]) == 0)
                    return REPLY_3;

        }

        // Skip this unknown reply
        line_complete = false;

        // Reschedule
        app_sched_execute();
        nrf_delay_ms(timeout_delay_ms);

    }
#endif

    // Not found in allotted time
    return REPLY_TIMEOUT;

}

// A hack to display something harmlessly for debugging
void send_debug_message(char *msg) {
    char buffer[100];
    sprintf(buffer, "at+cftppw=\"%s\"", msg);
    send_and_wait_for_reply(buffer, "OK", "ERROR", NULL);
}

// Function for initialization of LEDs, used for DFU feedback
static void leds_init(void) {
    nrf_gpio_range_cfg_output(LED_START, LED_STOP);
    nrf_gpio_pins_clear(LEDS_MASK);
}


// For debugging of the bootloader with the standard button-based DFU
#ifndef FONA
static void buttons_init(void) {
    nrf_gpio_cfg_sense_input(BOOTLOADER_BUTTON,
                             BUTTON_PULL,
                             NRF_GPIO_PIN_SENSE_LOW);
}
#endif

// Main bootloader entry
int main(void) {
    uint32_t ret_val;

    // This is done by dfu_init, however we also need app scheduler for UART
    // which is active prior to even getting to dfu.
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);

    // Init the log package
    (void) NRF_LOG_INIT(NULL);
    NRF_LOG_INFO("TTBOOT: Inside main!\r\n");

    // Init our board, plus serial, and start reliably talking to the Fona
    gpio_init();
#ifdef FONA
    gpio_uart_select(UART_FONA);
#endif
    send_and_wait_for_reply("at+cgfunc=11,0", "OK", "ERROR", NULL);
    send_and_wait_for_reply("ate0", "OK", "ERROR", NULL);
    send_and_wait_for_reply("at+catr=1", "OK", "ERROR", NULL);

    // Init other board stuff for compatibility
    leds_init();

#ifndef FONA
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

// Implemenation of nrf_dfu_check_enter for when FONA
// is present.  If not present, we fall into default behavior
// as defined by the _WEAK method in the SDK.
#ifdef FONA
bool nrf_dfu_enter_check(void) {
    static bool fChecked = false;
    static bool fResult;
    uint16_t response;

    // Exit if we've already been here
    if (fChecked)
        return fResult;

    fChecked = true;

    // See if the special file exists, whose presence we use to drive DFU
    response = send_and_wait_for_reply("at+fsattri=dfu.zip", "OK", "ERROR", NULL);
    if (response == REPLY_1) {
        // OK means that the file existed, and that we should drop into DFU mode.
        fResult = true;
    } else {
        // Any other reply means that we do NOT want to start DFU
        fResult = false;
    }

    // Done
    send_debug_message(fResult ? "ENTER DFU MODE" : "No DFU requested");

    // This is what you use to initiate the transfer
    //    serial_send_string("at+cftrantx=\"c:/dfu.zip\"");

    return fResult;

}
#endif // FONA

// Process a line-completion event
void line_completion_event_handler(void *p_event_data, uint16_t event_size) {
    uint32_t type = * (uint32_t *) p_event_data;
    UNUSED_VARIABLE(type);

}

// Process byte received from modem
void fona_received_byte(uint8_t databyte) {

    // If this is an EOL, process it
    if (databyte == '\n' || databyte == '\r') {

        // If it's just a blank line (or the second char of \r\n), skip it
        if (linesize[line_building] == 0)
            return;

        // Null terminate the line, which is fine because it IS a string
        linebuf[line_building][linesize[line_building]] = '\0';

        // Bump to the next buffer
        line_completed = line_building;
        line_complete = true;
        if (++line_building >= LINES)
            line_building = 0;
        linesize[line_building] = 0;

        uint32_t type = 0;
        uint32_t err_code;
        // Notify the app that it should process the line
        err_code = app_sched_event_put(&type, sizeof(type), line_completion_event_handler);
        if (err_code != NRF_SUCCESS) {
            // so we can spot it in debug output on serial port
            serial_send_string("app_sched err\n");
            NRF_LOG_ERROR("Can't put into app queue 0x%08x\n", err_code);
        }

        return;
    }

    // Exit if line overflow, leaving one byte for a null terminator
    if (linesize[line_building] >= MAXLINE-2)
        return;

    // Add the char to the line buffer
    linebuf[line_building][linesize[line_building]++] = (char) databyte;

}
