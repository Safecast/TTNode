// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

// Fona Bootloader Support
#ifdef DFUFONA

#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "sdk_common.h"
#include "boards.h"
#include "nrf_delay.h"
#include "nrf_mbr.h"
#include "nrf_bootloader.h"
#include "nrf_bootloader_info.h"
#include "nrf_dfu_req_handler.h"
#include "nrf_dfu_transport.h"
#include "nrf_dfu_mbr.h"
#include "nrf_bootloader_app_start.h"
#include "app_timer.h"
#include "softdevice_handler_appsh.h"
#include "nrf_dfu.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "app_error.h"
#include "app_error_weak.h"
#include "nrf_bootloader_info.h"
#include "app_scheduler.h"
#include "app_timer_appsh.h"
#include "nrf_dfu_req_handler.h"
#include "dfu_req_handling.h"
#include "gpio.h"
#include "serial.h"

#ifdef DFUDEBUG
#include "nrf_log_backend.h"
#endif

// Serial I/O Buffers
#define IOBUFFERS 8
#define MAXLINE 150
#define MAXDATA 550
typedef struct {
    uint16_t linesize;
    char linebuf[MAXLINE];
    uint32_t dataoffset;
    bool nomoredata;
    uint16_t datasize;
    uint8_t databuf[MAXDATA];
} iobuf_t;

static bool initializing;
static bool dfu_check_done;
static bool receiving_init_packet;
static bool ignore_nondata;
static iobuf_t iobuf[IOBUFFERS];
static int completed_iobufs_available;
static uint16_t iobuf_completed;
static uint16_t iobuf_filling;
static bool iobuf_receiving_data;
static uint32_t iobuf_receiving_data_target;
static uint32_t iobuf_receiving_data_received;
static uint32_t received_total;

// For raw communications debugging
static uint32_t raw_received = 0;
#ifdef DEBUGRAW
static char raw_databytes[6] = {'*', '*', '*', '*', '*', '\0'};
#endif

// Init packet
static uint8_t init_packet_buffer[1024];
static uint16_t init_packet_received;
#define FLASH_BUFFER_CHUNK_LENGTH 256       // Must be an even multiple of CODE_PAGE_SIZE
static uint8_t code_page_buffer[CODE_PAGE_SIZE];
static uint16_t code_page_received;

// Reset the buffer
void iobuf_reset() {
    iobuf[iobuf_filling].linesize = 0;
    iobuf[iobuf_filling].datasize = 0;
    iobuf[iobuf_filling].nomoredata = false;
    iobuf[iobuf_filling].dataoffset = received_total;
    iobuf_receiving_data = false;
}

// Init I/O buffers
void iobuf_init() {
    completed_iobufs_available = 0;
    iobuf_completed = 0;
    iobuf_filling = 0;
    iobuf_reset();
}

// NRF Scheduler Parameters
#define SCHED_MAX_EVENT_DATA_SIZE       MAX(APP_TIMER_SCHED_EVT_SIZE, sizeof(uint16_t))
#define SCHED_QUEUE_SIZE                20

// Timer stuff
#define TIMER_SECONDS                   5
#define TIMER_INTERVAL                  APP_TIMER_TICKS((TIMER_SECONDS*1000), APP_TIMER_PRESCALER)
#define APP_TIMER_TICKS_PER_SECOND      32768
#define APP_TIMER_PRESCALER             0
#define APP_TIMER_OP_QUEUE_SIZE         4
// APP_TIMER_DEF(timer);

// For the transport definition
uint32_t fona_dfu_transport_init(void);
uint32_t fona_dfu_transport_close(void);
DFU_TRANSPORT_REGISTER(nrf_dfu_transport_t const dfu_trans) =
{
    .init_func =        fona_dfu_transport_init,
    .close_func =       fona_dfu_transport_close
};

// Send a string without any AT characters in it
bool debug_clean(uint8_t *buff, uint16_t len) {
#ifdef DFUDEBUG
    int i;
    uint8_t lastbyte = '\0';
    for (i=0; i<len; i++) {
        uint8_t thisbyte = buff[i];
        if ((thisbyte >= 0x20 && thisbyte < 0x7f) || (thisbyte == '\r' || thisbyte == '\n')) {
            // Wicked hack, because the modem scans for and processes ANY sequence containing "AT"!!!
            if ((lastbyte == 'a' || lastbyte == 'A') && (thisbyte == 't' || thisbyte == 'T'))
                thisbyte = '*';
            serial_send_byte(thisbyte);
            lastbyte = thisbyte;
        }
    }
    return true;
#endif
}

// Transmit using any hack necessary to display harmlessly to somewhere where it might be seen
bool debug_tx(uint8_t *buff, uint16_t len) {
#ifdef DEBUGRAW
    if (buff[0] >= ' ') {
        char header[32];
        sprintf(header, "%06ld %s ", raw_received, raw_databytes);
        debug_clean((uint8_t *)header, strlen(header));
    }
#endif
    return(debug_clean(buff, len));
}

// Transmit a debug message
void debug_string(char *msg) {
#ifdef DFUDEBUG
    debug_tx((uint8_t *)msg, strlen(msg));
    debug_tx((uint8_t *)"\r\n", 2);
#endif
}

// Transmit a debug message with a single value
void debug_value(char *msg, uint32_t value) {
    char buffer[100];
    sprintf(buffer, "%s: 0x%08lx", msg, value);
    debug_string(buffer);
}

// Timer, just to know things are still alive
void timer_handler(void *p_context) {
    serial_send_string("** timer **");
}

// Drain the completed I/O buffer and point to the next
bool iobuf_pop(iobuf_t *piobuf) {
    if (completed_iobufs_available <= 0)
        return false;
#if DFUDEBUGMAX
    char buffer[100];
    sprintf(buffer, "pop '%s' (%d %d)", iobuf[iobuf_completed].linebuf, iobuf_completed, iobuf_filling);
    debug_string(buffer);
#endif
    *piobuf = iobuf[iobuf_completed];
    completed_iobufs_available--;
    if (++iobuf_completed >= IOBUFFERS)
        iobuf_completed = 0;
    return true;
}

// Bump to the next I/O buffer, dropping the line if we overflow I/O buffers
bool iobuf_push() {
    bool dropped = false;

    // A particular pecularity of the SIMCOM chip is that if it
    // is doing a TRANTX for a very long buffer, it will be
    // offline from a radio perspective, which generates a massive
    // block of spurious NO CARRIER messages nearing the end
    // of the transfer.
    if (strcmp("NO CARRIER", iobuf[iobuf_filling].linebuf) == 0)
        dropped = true;

    // If not dropping the message, close it out and move on
    if (!dropped) {
        if (completed_iobufs_available < IOBUFFERS) {
            received_total += iobuf[iobuf_filling].datasize;
            completed_iobufs_available++;
            if (++iobuf_filling >= IOBUFFERS)
                iobuf_filling = 0;
#ifdef LED_PIN_RED
            gpio_pin_set(LED_PIN_RED, (received_total & 0x00000200) != 0);
#endif
#ifdef LED_PIN_YEL
            gpio_pin_set(LED_PIN_YEL, (received_total & 0x00000400) != 0);
#endif
        } else {
            dropped = true;
        }
    }

    // Initialize the buffer
    iobuf_reset();

    return !dropped;
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

    // If we'd been ignoring nondata, turn off that mode now
    ignore_nondata = false;

    // Send the command
    serial_send_string(cmd);

    for (i=timeout_count;; --i) {
        iobuf_t iobuf_popped;

        if (i == 0)
            break;

        if (iobuf_pop(&iobuf_popped)) {

            if (r1 != NULL)
                if (memcmp(r1, iobuf_popped.linebuf, strlen(r1)) == 0)
                    return REPLY_1;

            if (r2 != NULL)
                if (memcmp(r2, iobuf_popped.linebuf, strlen(r2)) == 0)
                    return REPLY_2;

            if (r3 != NULL)
                if (memcmp(r3, iobuf_popped.linebuf, strlen(r3)) == 0)
                    return REPLY_3;

            // Debug
#ifdef DFUDEBUG
            char buffer[100];
            sprintf(buffer, "??? (%d) '%s' %d %d", strlen(iobuf_popped.linebuf), iobuf_popped.linebuf, iobuf_filling, iobuf_completed);
            debug_string(buffer);
#endif

        }

        // Force a reschedule
        app_sched_execute();
        nrf_delay_ms(timeout_delay_ms);

    }

    // Not found in allotted time
    return REPLY_TIMEOUT;

}

// Kickoff handler
void kickoff_dat_event_handler(void *p_event_data, uint16_t event_size) {
    received_total = 0;
    init_packet_received = 0;
    receiving_init_packet = true;
    ignore_nondata = true;
    serial_send_string("at+cftrantx=\"c:/dfu.dat\"");
}

// Kickoff handler
void kickoff_bin_event_handler(void *p_event_data, uint16_t event_size) {
    received_total = 0;
    code_page_received = 0;
    receiving_init_packet = false;
    ignore_nondata = true;
    serial_send_string("at+cftrantx=\"c:/dfu.bin\"");
}

// BLE event handler
#ifdef SOFTDEVICE_PRESENT
void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
}
#endif

// Softdevice initialization
#ifdef SOFTDEVICE_PRESENT
uint32_t softdevice_init()
{
    uint32_t         err_code;
    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

    debug_string("init_sd()");

    err_code = nrf_dfu_mbr_init_sd();
    if (err_code != NRF_SUCCESS) {
        debug_value("dfu mbr init sd err %d", err_code);
        return err_code;
    }

    debug_string("vector_table_base_set()");

    NRF_LOG_INFO("vector table: 0x%08x\r\n", BOOTLOADER_START_ADDR);
    err_code = sd_softdevice_vector_table_base_set(BOOTLOADER_START_ADDR);
    if (err_code != NRF_SUCCESS) {
        debug_value("sd vec tbl init %d", err_code);
        return err_code;
    }

    debug_string("appsh_init()");

    SOFTDEVICE_HANDLER_APPSH_INIT(&clock_lf_cfg, true);

    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(1, 1, &ble_enable_params);
    if (err_code != NRF_SUCCESS) {
        debug_value("sd get cfg err %d", err_code);
        return err_code;
    }

#if (NRF_SD_BLE_API_VERSION == 3)
    ble_enable_params.gatt_enable_params.att_mtu = GATT_MTU_SIZE_DEFAULT;
#endif

    debug_string("softdevice_enable()");

    err_code = softdevice_enable(&ble_enable_params);
    if (err_code != NRF_SUCCESS) {
        debug_value("sd enable err %d", err_code);
        return err_code;
    }

    debug_string("softdevice_handler_set()");

    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    if (err_code != NRF_SUCCESS) {
        debug_value("sd evt hdlr err %d", err_code);
        return err_code;
    }

    debug_string("Softdevice initialized.");
    return NRF_SUCCESS;
}
#endif // SOFTDEVICE_PRESENT

// Terminate our transport completely.
void fona_dfu_term() {

    // Turn off serial because we will never get control
    // back, and we will cause horrendous reentrancy if we call back to our
    // serial processing after we've decided to jump into the app
    serial_term();

}

// Initialize our transport
bool fona_dfu_init() {

    // Initialize the softdevice, because we can't seem to get the NRF flash operations
    // functioning without the softdevice being enabled.
#ifdef SOFTDEVICE_PRESENT

    if (softdevice_init() != NRF_SUCCESS)
        return false;

#else

    // Init the completed task scheduler that lets us handle command
    // processing outside the interrupt handlers, and instead via app_sched_execute()
    // called from the main loop in main.c.
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);

#endif

    // Note that we're initializing.  This is important because we've found that
    // some conditions cause us to re-enter the bootloader
    initializing = true;
    dfu_check_done = false;
    received_total = 0;
    ignore_nondata = false;

    // Initialize the TTNODE I/O system
    iobuf_init();
    gpio_init();

    // Give immediate feedback that the unit is powered on
#ifdef LED_PIN_RED
    gpio_pin_set(LED_PIN_RED, true);
#endif
#ifdef LED_PIN_YEL
    gpio_pin_set(LED_PIN_YEL, true);
#endif

    // Select the Fona
    gpio_uart_select(UART_FONA);

    // Init app timer support
#ifdef TESTING_APP_TIMER
    APP_TIMER_APPSH_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, true);
    app_timer_create(&timer, APP_TIMER_MODE_REPEATED, timer_handler);
    app_timer_start(timer, TIMER_INTERVAL, NULL);
#endif

    // Do initial setup of the comms mode, without waiting for a reply
#if HWFC
    serial_send_string("at+cgfunc=11,1");
    serial_send_string("at+cgfunc=11,1");
    serial_send_string("at+ifc=2,2");
#else
    serial_send_string("at+cgfunc=11,0");
    serial_send_string("at+cgfunc=11,0");
    serial_send_string("at+ifc=0,0");
#endif

    // Wait for any prior failed download to stabilize.
    int i;
    int timeout_count = 15;
    int timeout_delay_ms = 100;
    for (i=timeout_count;; --i) {
        iobuf_t iobuf_popped;
        if (i == 0)
            break;
        if (iobuf_pop(&iobuf_popped)) {
            char *datastr = "+CFTRANTX:";
            uint16_t datastrlen = strlen(datastr);
            if (memcmp(iobuf_popped.linebuf, datastr, datastrlen) == 0) {
                i = timeout_count;
#ifdef DFUDEBUG
                char buffer[100];
                sprintf(buffer, "wait: %ld %d %s", received_total, iobuf_popped.datasize, iobuf_popped.linebuf);
                debug_string(buffer);
#endif
            }
        }

        app_sched_execute();
        nrf_delay_ms(timeout_delay_ms);
    }

    iobuf_init();
    timeout_count = 50;
    timeout_delay_ms = 100;
    for (i=timeout_count;; --i) {

        // Exit if we can't initialize Fona
        if (i == 0)
            return false;

#if HWFC
        send_and_wait_for_reply("at+cgfunc=11,1", "OK", "ERROR", "+");
        send_and_wait_for_reply("at+cgfunc=11,1", "OK", "ERROR", "+");
        send_and_wait_for_reply("at+ifc=2,2", "OK", "ERROR", "+");
        send_and_wait_for_reply("at+ifc=2,2", "OK", "ERROR", "+");
#else
        send_and_wait_for_reply("at+cgfunc=11,0", "OK", "ERROR", "+");
        send_and_wait_for_reply("at+cgfunc=11,0", "OK", "ERROR", "+");
        send_and_wait_for_reply("at+ifc=0,0", "OK", "ERROR", "+");
        send_and_wait_for_reply("at+ifc=0,0", "OK", "ERROR", "+");
#endif

        // Send these commands several times, because it is critical that
        // they get through.  Specifically, if we don't turn off echo we won't
        // understand responses to commands because we'll be receiving echoes
        // of the commands themselves as replies.
        send_and_wait_for_reply("ate0", "OK", "ERROR", "+");
        send_and_wait_for_reply("ate0", "OK", "ERROR", "+");
        // And this is perhaps the most important of all commands, because if
        // it fails to be processed,
        // a) the subsequent at+cftrantx for the binary, which is very long,
        //    will terminate after two or three 512 byte blocks
        // b) we will hang waiting for the +CFTRANTX: 0
        // c) because we are doing single-bank updates because of lack of
        //    memory, we will render the device as bricked because we overwrote
        //    the first several blocks of our program.  UGH.
        send_and_wait_for_reply("at+catr=1", "OK", "ERROR", NULL);
        send_and_wait_for_reply("at+catr=1", "OK", "ERROR", NULL);
        send_and_wait_for_reply("at+catr=1", "OK", "ERROR", NULL);
        send_and_wait_for_reply("at+catr=1", "OK", "ERROR", NULL);
        send_and_wait_for_reply("at+catr=1", "OK", "ERROR", NULL);
        if (send_and_wait_for_reply("at+catr=1", "OK", "ERROR", NULL) == REPLY_1)
            break;

#if DFUDEBUGMAX
        char buffer[100];
        sprintf(buffer, "retry (%d %d %d %ld)", iobuf_completed, iobuf_filling, iobuf_receiving_data, raw_received);
        debug_string(buffer);
#endif

        app_sched_execute();
        nrf_delay_ms(timeout_delay_ms);

    }

    // Ready for true initialization
    initializing = false;
    received_total = 0;
    return true;
}


// Implemenation of nrf_dfu_check_enter for when FONA
// is present.  If not present, we fall into default behavior
// as defined by the _WEAK method in the SDK.
bool nrf_dfu_enter_check(void) {
    static uint16_t fResult = false;
    uint16_t response;

    // Exit if we've already been here
    if (dfu_check_done)
        return fResult;

    dfu_check_done = true;

    // See if the special file exists, whose presence we use to drive DFU
    response = send_and_wait_for_reply("at+fsattri=dfu.dat", "+FSATTRI:", "ERROR", NULL);
    if (response == REPLY_1) {
        // Attribs means that the file existed, and that we should drop into DFU mode.
        fResult = true;
    } else if (response == REPLY_2) {
        // ERROR means that the file didn't exist
        fResult = false;
    } else {
        // Anything else (such as SMS DONE, PB DONE, etc.) should be ignored
        // but for all practical purposes acts as "no DFU" because fresult defaults to false
    }

    // See if the binary file exists, just to make sure
    if (fResult) {
        response = send_and_wait_for_reply("at+fsattri=dfu.bin", "+FSATTRI:", "ERROR", NULL);
        if (response == REPLY_1) {
            // Attribs means that the file existed, and that we should drop into DFU mode.
            fResult = true;
        } else if (response == REPLY_2) {
            // ERROR means that the file didn't exist
            fResult = false;
        } else {
            // Anything else (such as SMS DONE, PB DONE, etc.) should be ignored
            // but for all practical purposes acts as "no DFU" because fresult defaults to false
        }
    }

    // Done
    debug_string(fResult ? "ENTER DFU MODE" : "No DFU requested");

    // If no DFU requested, turn off serial because we will never get control
    // back, and we will cause horrendous reentrancy if we call back to our
    // serial processing after we've decided to jump into the app
    if (!fResult)
        fona_dfu_term();

    return fResult;

}

// Processing for when we know we've received the entire init packet
void init_packet_completed() {
    nrf_dfu_res_code_t err;
    nrf_dfu_req_t req;
    nrf_dfu_res_t res;
    req.req_type = NRF_DFU_OBJECT_OP_CREATE;
    req.obj_type = NRF_DFU_OBJ_TYPE_COMMAND;
    req.object_size = init_packet_received;
    err = nrf_dfu_command_req(NULL, &req, &res);
    if (err != NRF_DFU_RES_CODE_SUCCESS)
        debug_value("ERR Init Pkt Create %ld", err);
    else {
        req.req_type = NRF_DFU_OBJECT_OP_WRITE;
        req.p_req = init_packet_buffer;
        req.req_len = init_packet_received;
        err = nrf_dfu_command_req(NULL, &req, &res);
        if (err != NRF_DFU_RES_CODE_SUCCESS)
            debug_value("ERR Init Pkt Write %ld", err);
        else {
            req.req_type = NRF_DFU_OBJECT_OP_EXECUTE;
            err = nrf_dfu_command_req(NULL, &req, &res);
            if (err != NRF_DFU_RES_CODE_SUCCESS)
                debug_value("ERR Init Pkt Exec %ld", err);
        }
    }

    // Whether success or failure, Delete the .dat file
    // so that we don't end up in an infinite DFU loop
    serial_send_string("at+fsdel=\"dfu.dat\"");
    nrf_delay_ms(1000);

    // If successful, kick off download of the image
    if (err == NRF_DFU_RES_CODE_SUCCESS) {

        // Kick off the binary event handler
        if (app_sched_event_put(NULL, 0, kickoff_bin_event_handler) != NRF_SUCCESS)
            debug_string("ERR put 4");

    }
}

// Process init packet data
void init_packet(iobuf_t *piobuf) {
    uint8_t *data = piobuf->databuf;
    uint16_t datasize = piobuf->datasize;

    if (piobuf->nomoredata)
        init_packet_completed();
    else {
        if (init_packet_received+datasize > sizeof(init_packet_buffer))
            debug_string("ERR: Init Pkt Overrun");
        else {
            memcpy(&init_packet_buffer[init_packet_received], data, datasize);
            init_packet_received += datasize;
        }
    }
}

// Delay waiting for I/O
void schedule() {
    app_sched_execute();
}

// Process data packet data
void data_packet(iobuf_t *piobuf) {
    uint8_t *data = piobuf->databuf;
    uint16_t datasize = piobuf->datasize;

#ifdef DFUDEBUG
    char *dbgmsg = "Rcvd";
#endif

    // Fill the code page buffer
    uint16_t left_in_iobuf;
    if ((sizeof(code_page_buffer) - code_page_received) >= datasize)
        left_in_iobuf = 0;
    else
        left_in_iobuf = (code_page_received + datasize) - sizeof(code_page_buffer);
    memcpy(&code_page_buffer[code_page_received], data, datasize-left_in_iobuf);
    code_page_received += datasize-left_in_iobuf;

    // Write it if we've filled the code page
    if (piobuf->nomoredata || code_page_received == sizeof(code_page_buffer)) {
        nrf_dfu_res_code_t err;
        nrf_dfu_req_t req;
        nrf_dfu_res_t res;
        req.req_type = NRF_DFU_OBJECT_OP_CREATE;
        req.obj_type = NRF_DFU_OBJ_TYPE_DATA;
        req.object_size = code_page_received;
        err = nrf_dfu_data_req(NULL, &req, &res);
        if (err != NRF_DFU_RES_CODE_SUCCESS)
            debug_value("ERR Data Create %ld", err);
        else {
            uint16_t chunk_left = code_page_received;
            uint8_t *chunk = code_page_buffer;
            while (chunk_left) {
                uint16_t chunk_len = chunk_left;
                if (chunk_len > FLASH_BUFFER_CHUNK_LENGTH)
                    chunk_len = FLASH_BUFFER_CHUNK_LENGTH;
                req.req_type = NRF_DFU_OBJECT_OP_WRITE;
                req.p_req = chunk;
                req.req_len = chunk_len;
                err = nrf_dfu_data_req(NULL, &req, &res);
                if (err != NRF_DFU_RES_CODE_SUCCESS)
                    debug_value("ERR Data Write %ld", err);
                chunk += chunk_len;
                chunk_left -= chunk_len;
                schedule();
            }
            schedule();
            req.req_type = NRF_DFU_OBJECT_OP_EXECUTE;
            err = nrf_dfu_data_req(NULL, &req, &res);
            if (err != NRF_DFU_RES_CODE_SUCCESS)
                debug_value("ERR Data Exec %ld", err);
        }

#ifdef DFUDEBUG
        if (err == NRF_DFU_RES_CODE_SUCCESS) {
            dbgmsg = piobuf->nomoredata ? "Stored FINAL" : "Stored";
            // Process final chunk
            if (piobuf->nomoredata) {
                // Wait until complete.  This will do an NVIC_SystemReset
                // after closing the transport.
                while (true) {
                    nrf_dfu_req_handler_reset_if_dfu_complete();
                    debug_string("Waiting for DFU completion...");
                    nrf_delay_ms(1000);
                }
            }
        } else
            dbgmsg = piobuf->nomoredata ? "Failed FINAL" : "Failed";
#endif

        // Move the odd amount that may be remaining into the code page buffer
        memcpy(code_page_buffer, &data[datasize-left_in_iobuf], left_in_iobuf);
        code_page_received = left_in_iobuf;

    }

#ifdef DFUDEBUG
    char buffer[100];
    sprintf(buffer, "%s(%ld:%d:%d/%d)", dbgmsg, piobuf->dataoffset, datasize, code_page_received, sizeof(code_page_buffer));
    debug_string(buffer);
#endif

}

// Process a data-received events
void packet_event_handler(void *p_event_data, uint16_t event_size) {
    static int in_here = 0;
    iobuf_t iobuf_popped;

    // Prevent recursion, which DOES happen because of our need
    // to call the app_scheduler to execute flash storage events.

    if (in_here)
        return;

    in_here++;

    // The cure to discarding events above is to always process
    // all pending events before exiting.
    while (iobuf_pop(&iobuf_popped)) {

        // Process the packet
        if (receiving_init_packet)
            init_packet(&iobuf_popped);
        else
            data_packet(&iobuf_popped);

    }

    // Done
    in_here--;

}

// Process byte received from modem
void fona_received_byte(uint8_t databyte) {

    // Raw debugging of input stream
    raw_received++;
#ifdef DEBUGRAW
    int r;
    for (r=0; r<sizeof(raw_databytes)-2; r++)
        raw_databytes[r] = raw_databytes[r+1];
    if (databyte <= ' ' || databyte >= 0x7f)
        raw_databytes[r] = '_';
    else
        raw_databytes[r] = (char) databyte;
#endif

    // If we're receiving binary that's associated with the command, do it.
    if (iobuf_receiving_data) {

        // Don't allow reception of a buffer larger than we'e statically allocated
        if (iobuf[iobuf_filling].datasize < MAXDATA)
            iobuf[iobuf_filling].databuf[iobuf[iobuf_filling].datasize++] = databyte;
        else
            debug_string("ERR data overrun");

        // Exit if we've still got more binary data to receive
        iobuf_receiving_data_received++;
        if (iobuf_receiving_data_received < iobuf_receiving_data_target)
            return;

        // Stop receiving any more binary data
        iobuf_receiving_data = false;

        // Bump to the next I/O buffer so the next serial event doesn't overwrite this one
        if (iobuf_push()) {

            // Asynchronously notify the app that no more data will be forthcoming
            if (!initializing)
                app_sched_event_put(NULL, 0, packet_event_handler);

        }

        return;

    }

    // If we get here, we're expecting to receive an ASCII text command terminated in \r\n.
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

            // Indicate that no further data is coming for this set of data
            iobuf[iobuf_filling].nomoredata = true;

            // Bump to the next I/O buffer so the next serial event doesn't overwrite this one
            if (iobuf_push()) {

                // Asynchronously notify the app that no more data will be forthcoming
                if (!initializing)
                    app_sched_event_put(NULL, 0, packet_event_handler);

            }

            return;
        }

        // See if we should be switching to a mode in which we receive binary data
        char *datastr = "+CFTRANTX: DATA,";
        uint16_t datastrlen = strlen(datastr);
        if (memcmp(iobuf[iobuf_filling].linebuf, datastr, datastrlen) == 0) {
            iobuf_receiving_data_target = atoi(&iobuf[iobuf_filling].linebuf[datastrlen]);
            iobuf_receiving_data_received = 0;
            iobuf_receiving_data = true;
            return;
        }

        // If we're still in a mode where we're paying attention to nondata, enqueue it
        if (ignore_nondata)
            iobuf_reset();
        else
            iobuf_push();

        return;
    }

    // If somehow a non-text character got here, substitute.
    if (databyte < 0x20 || databyte > 0x7f)
        databyte = '.';

    // Add the char to the line buffer if we can, else it's overrun
    if (iobuf[iobuf_filling].linesize < (MAXLINE-2))
        iobuf[iobuf_filling].linebuf[iobuf[iobuf_filling].linesize++] = (char) databyte;
    else
        iobuf_reset();

}

uint32_t fona_dfu_transport_init(void) {
    uint32_t err_code = NRF_SUCCESS;

    // Initialize our first event.  This is done asynchronously so that nrf_dfu_init() has a chance
    // to call nrf_dfu_req_handler_init() before we start jamming stuff into the request handler.
    // This event should get kicked off on the first wait_for_event().
    if (app_sched_event_put(NULL, 0, kickoff_dat_event_handler) != NRF_SUCCESS)
        debug_string("ERR put 3");

    return err_code;
}


uint32_t fona_dfu_transport_close(void) {
    uint32_t err_code = NRF_SUCCESS;

    // Clean up, especially because of serial I/O
    fona_dfu_term();

    return err_code;
}

#ifdef DFUDEBUG

// Statics for log backend
static bool m_initialized   = false;
static bool m_blocking_mode = false;

// Hex dump parameters
#define HEXDUMP_BYTES_PER_LINE               16
#define HEXDUMP_HEXBYTE_AREA                 3 // Two bytes for hexbyte and space to separate
#define HEXDUMP_MAX_STR_LEN (NRF_LOG_BACKEND_MAX_STRING_LENGTH -        \
                             (HEXDUMP_HEXBYTE_AREA*HEXDUMP_BYTES_PER_LINE + \
                              2)) /* Separators */

ret_code_t nrf_log_backend_init(bool blocking) {
    if (m_initialized && (blocking == m_blocking_mode)) {
        return NRF_SUCCESS;
    }
    m_initialized   = true;
    m_blocking_mode = blocking;
    return NRF_SUCCESS;
}

static void byte2hex(const uint8_t c, char * p_out) {
    uint8_t  nibble;
    uint32_t i = 2;
    while (i-- != 0) {
        nibble       = (c >> (4 * i)) & 0x0F;
        p_out[1 - i] = (nibble > 9) ? ('A' + nibble - 10) : ('0' + nibble);
    }
}

static uint32_t nrf_log_backend_serial_hexdump_handler(
    uint8_t                severity_level,
    const uint32_t * const p_timestamp,
    const char * const     p_str,
    uint32_t               offset,
    const uint8_t * const  p_buf0,
    uint32_t               buf0_length,
    const uint8_t * const  p_buf1,
    uint32_t               buf1_length) {
    char     str[NRF_LOG_BACKEND_MAX_STRING_LENGTH];
    uint32_t slen;
    char   * p_hex_part;
    char   * p_char_part;
    uint8_t  c;
    uint32_t byte_in_line;
    uint32_t buffer_len    = 0;
    uint32_t byte_cnt      = offset;
    uint32_t length        = buf0_length + buf1_length;

    // If it is the first part of hexdump print the header
    if (offset == 0) {
        slen = strlen(p_str);
        // Saturate string if it's too long.
        slen = (slen > HEXDUMP_MAX_STR_LEN) ? HEXDUMP_MAX_STR_LEN : slen;
        memcpy(&str[buffer_len], p_str, slen);
        buffer_len += slen;
    }

    do {

        uint32_t hex_part_offset  = buffer_len;
        uint32_t char_part_offset = hex_part_offset +
            (HEXDUMP_BYTES_PER_LINE * HEXDUMP_HEXBYTE_AREA + 1); // +1 - separator between hexdump and characters.

        p_hex_part  = &str[hex_part_offset];
        p_char_part = &str[char_part_offset];

        // Fill the blanks to align to timestamp print
        for (byte_in_line = 0; byte_in_line < HEXDUMP_BYTES_PER_LINE; byte_in_line++) {
            if (byte_cnt >= length) {
                // file the blanks
                *p_hex_part++  = ' ';
                *p_hex_part++  = ' ';
                *p_hex_part++  = ' ';
                *p_char_part++ = ' ';
            } else {
                if (byte_cnt < buf0_length) {
                    c = p_buf0[byte_cnt];
                } else {
                    c = p_buf1[byte_cnt - buf0_length];
                }
                byte2hex(c, p_hex_part);
                p_hex_part    += 2; // move the pointer since byte in hex was added.
                *p_hex_part++  = ' ';
                *p_char_part++ = isprint(c) ? c : '.';
                byte_cnt++;
            }
        }
        *p_char_part++ = '\r';
        *p_char_part++ = '\n';
        *p_hex_part++  = ' ';
        buffer_len    +=
            (HEXDUMP_BYTES_PER_LINE * HEXDUMP_HEXBYTE_AREA + 1) + // space for hex dump and separator between hexdump and string
            HEXDUMP_BYTES_PER_LINE +                              // space for stringS dump
            2;                                                    // space for new line

        if (!debug_tx((uint8_t *)str, buffer_len))
            return byte_cnt;

        buffer_len = 0;
    } while (byte_cnt < length);

    return byte_cnt;
}

bool nrf_log_backend_serial_std_handler(
    uint8_t                severity_level,
    const uint32_t * const p_timestamp,
    const char * const     p_str,
    uint32_t             * p_args,
    uint32_t               nargs) {
    char     str[NRF_LOG_BACKEND_MAX_STRING_LENGTH];
    int32_t  tmp_str_len     = 0;
    uint32_t buffer_len      = 0;
    bool     status          = true;

    switch (nargs) {
    case 0: {
        tmp_str_len = strlen(p_str);
        if ((tmp_str_len + buffer_len) < NRF_LOG_BACKEND_MAX_STRING_LENGTH) {
            memcpy(&str[buffer_len], p_str, tmp_str_len);
        }
        break;
    }

    case 1:
        tmp_str_len = snprintf(&str[buffer_len], NRF_LOG_BACKEND_MAX_STRING_LENGTH-buffer_len, p_str, p_args[0]);
        break;

    case 2:
        tmp_str_len = snprintf(&str[buffer_len], NRF_LOG_BACKEND_MAX_STRING_LENGTH-buffer_len, p_str, p_args[0], p_args[1]);
        break;

    case 3:
        tmp_str_len = snprintf(&str[buffer_len], NRF_LOG_BACKEND_MAX_STRING_LENGTH-buffer_len, p_str, p_args[0], p_args[1], p_args[2]);
        break;

    case 4:
        tmp_str_len = snprintf(&str[buffer_len], NRF_LOG_BACKEND_MAX_STRING_LENGTH-buffer_len, p_str, p_args[0], p_args[1], p_args[2], p_args[3]);
        break;

    case 5:
        tmp_str_len = snprintf(&str[buffer_len],
                               NRF_LOG_BACKEND_MAX_STRING_LENGTH-buffer_len,
                               p_str,
                               p_args[0],
                               p_args[1],
                               p_args[2],
                               p_args[3],
                               p_args[4]);
        break;

    case 6:
        tmp_str_len = snprintf(&str[buffer_len],
                               NRF_LOG_BACKEND_MAX_STRING_LENGTH-buffer_len,
                               p_str,
                               p_args[0],
                               p_args[1],
                               p_args[2],
                               p_args[3],
                               p_args[4],
                               p_args[5]);
        break;

    default:
        break;
    }

    // buf_len_update()
    if (tmp_str_len < 0)
        status = false;
    else {
        buffer_len += (uint32_t)tmp_str_len;
        status = true;
    }

    if (status && (buffer_len <= NRF_LOG_BACKEND_MAX_STRING_LENGTH)) {
        return debug_tx((uint8_t *)str, buffer_len);
    } else {
        // error, snprintf failed.
        return false;
    }

}

nrf_log_hexdump_handler_t nrf_log_backend_hexdump_handler_get(void) {
    return nrf_log_backend_serial_hexdump_handler;
}

nrf_log_std_handler_t nrf_log_backend_std_handler_get(void) {
    return nrf_log_backend_serial_std_handler;
}

#endif // DFUDEBUG

#endif // DFUFONA
