//
// Fona Bootloader Support
//

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

// Init packet
static uint8_t init_packet_buffer[1024];
static uint16_t init_packet_received;
#define FLASH_BUFFER_CHUNK_LENGTH 256       // Must be an even multiple of CODE_PAGE_SIZE
static uint8_t code_page_buffer[CODE_PAGE_SIZE];
static uint16_t code_page_received;

// Init I/O buffers
void iobuf_init() {
    completed_iobufs_available = 0;
    iobuf_filling = 0;
    iobuf_receiving_data = false;
    iobuf[0].linesize = 0;
    iobuf[0].datasize = 0;
}

// Reset the buffer
void iobuf_reset() {
    iobuf[iobuf_filling].linesize = 0;
    iobuf[iobuf_filling].datasize = 0;
    iobuf[iobuf_filling].dataoffset = received_total;
    iobuf_receiving_data = false;
}

// NRF Scheduler Parameters
#define SCHED_MAX_EVENT_DATA_SIZE       MAX(APP_TIMER_SCHED_EVT_SIZE, sizeof(uint16_t))
#define SCHED_QUEUE_SIZE                20

// For the transport definition
uint32_t fona_dfu_transport_init(void);
uint32_t fona_dfu_transport_close(void);
DFU_TRANSPORT_REGISTER(nrf_dfu_transport_t const dfu_trans) =
{
    .init_func =        fona_dfu_transport_init,
    .close_func =       fona_dfu_transport_close
};

// Transmit using any hack necessary to display harmlessly to somewhere where it might be seen
bool debug_tx(uint8_t *buff, uint16_t len) {
    int i;
    uint8_t lastbyte = '\0';
    for (i=0; i<len; i++) {
        uint8_t thisbyte = buff[i];
        if ((thisbyte >= 0x20 && thisbyte < 0x7f) || (thisbyte == '\r' || thisbyte == '\n')) {
            // Wicked hack, because the modem scans for and processes ANY sequence containing "AT"!!!
            if ((lastbyte == 'a' || lastbyte == 'A') && (thisbyte == 't' || thisbyte == 'T'))
                thisbyte = '*';
#ifdef DFUDEBUG
            serial_send_byte(thisbyte);
#endif
            lastbyte = thisbyte;
        }
    }
    return true;
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
    sprintf("%s: %ld/0x%08lx", msg, value, value);
    debug_string(buffer);
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
bool iobuf_push() {
    bool dropped = false;

    // A particular pecularity of the SIMCOM chip is that if it
    // is doing a TRANTX for a very long buffer, it will be
    // offline from a radio perspective, which generates a massive
    // block of spurious NO CARRIER messages nearing the end
    // of the transfer.
    if (strcmp("NO CARRIER", iobuf[iobuf_completed].linebuf) == 0)
        dropped = true;

    // If not dropping the message, close it out and move on
    if (!dropped) {
        if (completed_iobufs_available < IOBUFFERS) {
            received_total += iobuf[iobuf_filling].datasize;
            completed_iobufs_available++;
            if (++iobuf_filling >= IOBUFFERS)
                iobuf_filling = 0;
#ifdef LED_COLOR
            gpio_pin_set(LED_PIN_RED, (received_total & 0x00000200) != 0);
            gpio_pin_set(LED_PIN_YEL, (received_total & 0x00000400) != 0);
#endif
        } else {
            dropped = true;
            debug_string("$");
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
                debug_string(buffer);
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
    
// Initialize fona state
void fona_init() {
}

// Initialize our transport
void fona_dfu_init() {

    // Init the completed task scheduler that lets us handle command
    // processing outside the interrupt handlers, and instead via app_sched_execute()
    // called from the main loop in main.c.
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);

    // Note that we're initializing.  This is important because we've found that
    // some conditions cause us to re-enter the bootloader
    initializing = true;
    dfu_check_done = false;
    received_total = 0;
    ignore_nondata = false;

    iobuf_init();
    gpio_init();
    gpio_indicators_off();
    gpio_uart_select(UART_FONA);

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
                sprintf(buffer, "wait: %ld %d %s", received_total, iobuf[iobuf_completed].datasize, iobuf[iobuf_completed].linebuf);
                debug_string(buffer);
#endif
            }
            iobuf_pop();
        }
  
        app_sched_execute();
        nrf_delay_ms(timeout_delay_ms);
    }

    // Fully init the fona
#if HWFC
    send_and_wait_for_reply("at+cgfunc=11,1", "OK", "ERROR", NULL);
    send_and_wait_for_reply("at+cgfunc=11,1", "OK", "ERROR", NULL);
    send_and_wait_for_reply("at+ifc=2,2", "OK", "ERROR", NULL);
#else
    send_and_wait_for_reply("at+cgfunc=11,0", "OK", "ERROR", NULL);
    send_and_wait_for_reply("at+cgfunc=11,0", "OK", "ERROR", NULL);
    send_and_wait_for_reply("at+ifc=0,0", "OK", "ERROR", NULL);
#endif
    send_and_wait_for_reply("ate0", "OK", "ERROR", NULL);
    send_and_wait_for_reply("at+catr=1", "OK", "ERROR", NULL);

    // Ready for true initialization
    initializing = false;
    received_total = 0;

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
    response = send_and_wait_for_reply("at+fsattri=dfu.dat", "OK", "ERROR", NULL);
    if (response == REPLY_1) {
        // OK means that the file existed, and that we should drop into DFU mode.
        fResult = true;
    } else {
        // Any other reply means that we do NOT want to start DFU
        fResult = false;
    }

    // Done
    debug_string(fResult ? "ENTER DFU MODE" : "No DFU requested");

    return fResult;

}

// Process init packet data
void init_packet(uint16_t completed_iobuf) {
    uint8_t *data = iobuf[completed_iobuf].databuf;
    uint16_t datasize = iobuf[completed_iobuf].datasize;
    if (init_packet_received+datasize > sizeof(init_packet_buffer))
        debug_string("ERR: Init Pkt Overrun");
    else {
        memcpy(&init_packet_buffer[init_packet_received], data, datasize);
        init_packet_received += datasize;
    }
}

// Processing for when we know we've received the entire init packet
void init_packets_completed() {
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

    // If successful, kick off download of the image
    if (err == NRF_DFU_RES_CODE_SUCCESS) {
        if (app_sched_event_put(NULL, 0, kickoff_bin_event_handler) != NRF_SUCCESS)
            debug_string("ERR put 4");
    }
}

// Process data packet data
void data_packet(uint16_t completed_iobuf) {
    uint8_t *data = iobuf[completed_iobuf].databuf;
    uint16_t datasize = iobuf[completed_iobuf].datasize;

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
    if (code_page_received == sizeof(code_page_buffer)) {
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
            }
            req.req_type = NRF_DFU_OBJECT_OP_EXECUTE;
            err = nrf_dfu_data_req(NULL, &req, &res);
            if (err != NRF_DFU_RES_CODE_SUCCESS)
                debug_value("ERR Data Exec %ld", err);
        }

#ifdef DFUDEBUG
        if (err == NRF_DFU_RES_CODE_SUCCESS)
            dbgmsg = "Stored";
        else
            dbgmsg = "Failed";
#endif

        // Move the odd amount that may be remaining into the code page buffer
        memcpy(code_page_buffer, &data[datasize-left_in_iobuf], left_in_iobuf);
        code_page_received = left_in_iobuf;

    }

#ifdef DFUDEBUG
    char buffer[100];
    sprintf(buffer, "%s(%ld:%d:%d/%d)", dbgmsg, iobuf[completed_iobuf].dataoffset, datasize, code_page_received, sizeof(code_page_buffer));
    debug_string(buffer);
#endif

}

// Processing for when we know we've received the entire image
void data_packets_completed() {

    // Send the final chunk, which is less than a page
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
        }
        req.req_type = NRF_DFU_OBJECT_OP_EXECUTE;
        err = nrf_dfu_data_req(NULL, &req, &res);
        if (err != NRF_DFU_RES_CODE_SUCCESS)
            debug_value("ERR Data Exec %ld", err);
    }

    // If successful, say so
    if (err == NRF_DFU_RES_CODE_SUCCESS)
        debug_string("Successful FINAL Data");
    else
        debug_value("FINAL Data error %ld", err);

}

// Process a data-received events
void packet_event_handler(void *p_event_data, uint16_t event_size) {
    uint16_t completed_iobuf = * (uint16_t *) p_event_data;

    // Process the packet
    if (receiving_init_packet)
        init_packet(completed_iobuf);
    else
        data_packet(completed_iobuf);

    // Release this buffer, which can now be re-used
    iobuf_pop();

}

// Process data-completion events
void completion_event_handler(void *p_event_data, uint16_t event_size) {

    // Process the packet
    if (receiving_init_packet)
        init_packets_completed();
    else
        data_packets_completed();

    // Release this buffer, which can now be re-used
    iobuf_pop();

}

// Process byte received from modem
void fona_received_byte(uint8_t databyte) {

    // If we're receiving binary that's associated with the command, do it.
    if (iobuf_receiving_data) {

        // Don't allow reception of a buffer larger than we'e statically allocated
        if (iobuf[iobuf_filling].datasize < sizeof(iobuf[iobuf_filling].databuf))
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
                if (app_sched_event_put(&iobuf_completed, sizeof(iobuf_completed), packet_event_handler) != NRF_SUCCESS)
                    debug_string("ERR put 1");

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

            // Bump to the next I/O buffer so the next serial event doesn't overwrite this one
            if (iobuf_push()) {

                // Asynchronously notify the app that no more data will be forthcoming
                if (!initializing)
                    if (app_sched_event_put(NULL, 0, completion_event_handler) != NRF_SUCCESS) {
                        debug_string("ERR put 2");
                    }

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

    // Add the char to the line buffer
    if (iobuf[iobuf_filling].linesize < (sizeof(iobuf[iobuf_filling].linebuf)-2))
        iobuf[iobuf_filling].linebuf[iobuf[iobuf_filling].linesize++] = (char) databyte;
    else
        debug_string("ERR line overrun");

}

// Softdevice initialization
#ifdef SOFTDEVICE_PRESENT
uint32_t softdevice_init()
{
    uint32_t         err_code;
    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

    err_code = nrf_dfu_mbr_init_sd();
    if (err_code != NRF_SUCCESS) {
        debug_value("dfu mbr init sd err %d", err_code);
        return err_code;
    }

    NRF_LOG_INFO("vector table: 0x%08x\r\n", BOOTLOADER_START_ADDR);
    err_code = sd_softdevice_vector_table_base_set(BOOTLOADER_START_ADDR);
    if (err_code != NRF_SUCCESS) {
        debug_value("sd vec tbl init %d", err_code);
        return err_code;
    }

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

    err_code = softdevice_enable(&ble_enable_params);
    if (err_code != NRF_SUCCESS) {
        debug_value("sd enable err %d", err_code);
        return err_code;
    }

    debug_string("Softdevice initialized.");
    return NRF_SUCCESS;
}
#endif // SOFTDEVICE_PRESENT

uint32_t fona_dfu_transport_init(void) {
    uint32_t err_code = NRF_SUCCESS;

    // Initialize the softdevice, because we can't seem to get the NRF flash operations
    // functioning without the softdevice being enabled.
#ifdef SOFTDEVICE_PRESENT
    err_code = softdevice_init();
    if (err_code != NRF_SUCCESS)
        return err_code;
#endif

    // Initialize our first event.  This is done asynchronously so that nrf_dfu_init() has a chance
    // to call nrf_dfu_req_handler_init() before we start jamming stuff into the request handler.
    // This event should get kicked off on the first wait_for_event().
    if (app_sched_event_put(NULL, 0, kickoff_dat_event_handler) != NRF_SUCCESS)
        debug_string("ERR put 3");

    return err_code;
}


uint32_t fona_dfu_transport_close(void) {
    uint32_t err_code = NRF_SUCCESS;

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
