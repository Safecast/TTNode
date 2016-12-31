//
// Fona Bootloader Support
//

#ifdef DFUFONA

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
#include "gpio.h"
#include "serial.h"

// Serial I/O Buffers
#define IOBUFFERS 8
#define MAXLINE 250
#define MAXDATA 550
typedef struct {
    uint16_t linesize;
    char linebuf[MAXLINE];
    uint32_t dataoffset;
    uint16_t datasize;
    uint8_t databuf[MAXDATA];
} iobuf_t;

static bool initializing;
static bool dfu_check_done;
static iobuf_t iobuf[IOBUFFERS];
static int completed_iobufs_available;
static uint16_t iobuf_completed;
static uint16_t iobuf_filling;
static uint16_t iobuf_filling_data_left;
static uint32_t iobuf_total_data;

// Init I/O buffers
void iobuf_init() {
    completed_iobufs_available = 0;
    iobuf_filling = 0;
    iobuf_filling_data_left = 0;
    iobuf[0].linesize = 0;
    iobuf[0].datasize = 0;
}

// Is one available?
bool iobuf_is_completed() {
    return (completed_iobufs_available > 0);
}

// Drain the completed I/O buffer and point to the next
void iobuf_pop() {
    if (completed_iobufs_available > 0) {
        if (++iobuf_completed >= IOBUFFERS)
            iobuf_completed = 0;
        completed_iobufs_available--;
    }
}

// Bump to the next I/O buffer, dropping the line if we overflow I/O buffers
void iobuf_push() {
    bool fDrop = false;

    // A particular pecularity of the SIMCOM chip is that if it
    // is doing a TRANTX for a very long buffer, it will be
    // offline from a radio perspective, which generates a massive
    // block of spurious NO CARRIER messages nearing the end
    // of the transfer.
    if (strcmp("NO CARRIER", iobuf[iobuf_completed].linebuf) == 0)
        fDrop = true;

    // If not dropping the message, close it out and move on
    if (!fDrop) {
        if (completed_iobufs_available < IOBUFFERS) {
            iobuf_total_data += iobuf[iobuf_filling].datasize;
            completed_iobufs_available++;
            if (++iobuf_filling >= IOBUFFERS)
                iobuf_filling = 0;
        } else {
#ifdef DFUDEBUG
            // Overrun
            serial_send_string("$");
#endif
        }
    }
    
    // Initialize the buffer
    iobuf[iobuf_filling].linesize = 0;
    iobuf[iobuf_filling].datasize = 0;
    iobuf[iobuf_filling].dataoffset = iobuf_total_data;
    iobuf_filling_data_left = 0;

}

// Wait for a reply
#define REPLY_TIMEOUT               0
#define REPLY_1                     1
#define REPLY_2                     2
#define REPLY_3                     3
uint16_t send_and_wait_for_reply(char *cmd, char *r1, char *r2, char *r3) {

    int i;
    int timeout_count = 30;
    int timeout_delay_ms = 100;

    // Send the command
    serial_send_string(cmd);

    for (i=timeout_count;; --i) {
        if (i == 0)
            break;

        if (iobuf_is_completed()) {

            // Display reply for debugging, so long as it's not a command being issued while still in Echo Mode,
            // and so long as it's not our debug message reply.
#ifdef DFUDEBUG
            if (iobuf[iobuf_completed].linebuf[2] != '+' && r3 != NULL && r3[0] != '$') {
                char buffer[100];
                sprintf(buffer, " %s", iobuf[iobuf_completed].linebuf);
                serial_send_string(buffer);
            }
#endif

            if (r1 != NULL)
                if (strcmp(r1, iobuf[iobuf_completed].linebuf) == 0) {
                    iobuf_pop();
                    return REPLY_1;
                }

            if (r2 != NULL)
                if (strcmp(r2, iobuf[iobuf_completed].linebuf) == 0) {
                    iobuf_pop();
                    return REPLY_2;
                }

            if (r3 != NULL)
                if (strcmp(r3, iobuf[iobuf_completed].linebuf) == 0) {
                    iobuf_pop();
                    return REPLY_3;
                }

            // Skip this unknown reply
            iobuf_pop();

        }

        // Force a reschedule
        app_sched_execute();
        nrf_delay_ms(timeout_delay_ms);

    }

    // Not found in allotted time
    return REPLY_TIMEOUT;

}

// A hack to display something harmlessly for debugging
void send_debug_message(char *msg) {
#ifdef DFUDEBUG
    char buffer[100];
    sprintf(buffer, "at+cftppw=\"%s\"", msg);
    send_and_wait_for_reply(buffer, "OK", "ERROR", "$");
#endif
}

// Initialize our transport
void fona_dfu_init() {

    // Note that we're initializing.  This is important because we've found that
    // some conditions cause us to re-enter the bootloader
    initializing = true;
    dfu_check_done = false;
    iobuf_total_data = 0;

    iobuf_init();
    gpio_init();
    gpio_uart_select(UART_FONA);

    // Establish basic connectivity to correct for differences in flow control
    serial_send_string("at+cgfunc=11,0");

    // Wait for any prior failed download to stabilize.
    int i;
    int timeout_count = 50;
    int timeout_delay_ms = 100;
    for (i=timeout_count;; --i) {
        if (i == 0)
            break;
        if (iobuf_is_completed()) {
            char *datastr = "+CFTRANTX:";
            uint16_t datastrlen = strlen(datastr);
            if (memcmp(iobuf[iobuf_completed].linebuf, datastr, datastrlen) == 0) {
                i = timeout_count;
#ifdef DFUDEBUG
                char buffer[100];
                sprintf(buffer, "wait: %ld %d %s", iobuf_total_data, iobuf[iobuf_completed].datasize, iobuf[iobuf_completed].linebuf);
                serial_send_string(buffer);
#endif
            }
            iobuf_pop();
        }
        app_sched_execute();
        nrf_delay_ms(timeout_delay_ms);
    }

    // Ready for true initialization
    initializing = false;
    iobuf_total_data = 0;

    // Initialize fona
    send_and_wait_for_reply("at+cgfunc=11,0", "OK", "ERROR", NULL);
    send_and_wait_for_reply("ate0", "OK", "ERROR", NULL);
    send_and_wait_for_reply("at+catr=1", "OK", "ERROR", NULL);

}


// Implemenation of nrf_dfu_check_enter for when FONA
// is present.  If not present, we fall into default behavior
// as defined by the _WEAK method in the SDK.
bool nrf_dfu_enter_check(void) {
    static uint16_t fResult;
    uint16_t response;

    // Exit if we've already been here
    if (dfu_check_done)
        return fResult;

    dfu_check_done = true;

    // See if the special file exists, whose presence we use to drive DFU
    response = send_and_wait_for_reply("at+fsattri=dfu.bin", "OK", "ERROR", NULL);
    if (response == REPLY_1) {
        // OK means that the file existed, and that we should drop into DFU mode.
        fResult = true;
    } else {
        // Any other reply means that we do NOT want to start DFU
        fResult = false;
    }

    // Done
    send_debug_message(fResult ? "ENTER DFU MODE" : "No DFU requested");

    //
    // Kick off a read of the DAT file
    //
    // This is temporary until I know the right way to bootstrap the transport
    //

#ifdef DFUDEBUG
    serial_send_string("at+cftrantx=\"c:/dfu.bin\"");
#endif

    return fResult;

}

// Process a data-completion event
void data_completion_event_handler(void *p_event_data, uint16_t event_size) {

#ifdef DFUDEBUG
    uint16_t completed_iobuf = * (uint16_t *) p_event_data;
    uint8_t *data = iobuf[completed_iobuf].databuf;
    uint16_t datasize = iobuf[completed_iobuf].datasize;
    uint32_t offset = iobuf[completed_iobuf].dataoffset;
    if (event_size == 0)
        serial_send_string("Data completed");
    else {
        char buffer[100];
        sprintf(buffer, "Data(%ld:%d) %02x %02x ... %02x %02x", offset, datasize, data[0], data[1], data[datasize-2], data[datasize-1]);
        serial_send_string(buffer);
    }
#endif

    iobuf_pop();

}

// Process byte received from modem
void fona_received_byte(uint8_t databyte) {

    // If we're filling binary that's associated with the command, do it.
    if (iobuf_filling_data_left) {

        if (iobuf[iobuf_filling].datasize < MAXDATA)
            iobuf[iobuf_filling].databuf[iobuf[iobuf_filling].datasize++] = databyte;

        if (--iobuf_filling_data_left == 0) {

            // Bump to the next I/O buffer so the next serial event doesn't overwrite this one
            iobuf_push();

            // Asynchronously notify the app that no more data will be forthcoming
            if (!initializing)
                if (app_sched_event_put(&iobuf_completed, sizeof(iobuf_completed), data_completion_event_handler) != NRF_SUCCESS) {
#ifdef DFUDEBUG
                    serial_send_string("ERR put 1");
#endif
                }
            
        }

        return;

    }

    // If this is the CR of the CRLF sequence, skip it.
    if (databyte == '\r')
        return;

    // If this the final byte of the CRLF sequence, process it
    if (databyte == '\n') {

        // If it's just a blank line (or the second char of \r\n), skip it
        if (iobuf[iobuf_filling].linesize == 0)
            return;

        // Null terminate the line because it's a string
        iobuf[iobuf_filling].linebuf[iobuf[iobuf_filling].linesize] = '\0';

        // See if this is the special command indicating that there is no more data
        if (strcmp(iobuf[iobuf_filling].linebuf, "+CFTRANTX: 0") == 0) {

            // Bump to the next I/O buffer so the next serial event doesn't overwrite this one
            iobuf_push();

            // Asynchronously notify the app that no more data will be forthcoming
            if (!initializing)
                if (app_sched_event_put(NULL, 0, data_completion_event_handler) != NRF_SUCCESS) {
#ifdef DFUDEBUG
                    serial_send_string("ERR put 2");
#endif
                }

            return;
        }

        // See if we should be switching to a mode in which we receive binary data
        char *datastr = "+CFTRANTX: DATA,";
        uint16_t datastrlen = strlen(datastr);
        if (memcmp(iobuf[iobuf_filling].linebuf, datastr, datastrlen) == 0) {
            iobuf_filling_data_left = atoi(&iobuf[iobuf_filling].linebuf[datastrlen]);
            return;
        }

        // Bump to the next I/O buffer so the next serial event doesn't overwrite this one
        iobuf_push();

        return;
    }

    // Exit if line overflow, leaving one byte for a null terminator
    if (iobuf[iobuf_filling].linesize >= MAXLINE-2)
        return;

    // If somehow a non-text character got here, substitute.
    if (databyte < 0x20 || databyte > 0x7f)
        databyte = '.';

    // Add the char to the line buffer
    iobuf[iobuf_filling].linebuf[iobuf[iobuf_filling].linesize++] = (char) databyte;

}

#endif // DFUFONA
