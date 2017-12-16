// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

#ifndef COMM_H__
#define COMM_H__

// States.  For device-specific state, start assigning at COMM_STATE_DEVICE_START
#define COMM_STATE_IDLE                 0
#define COMM_STATE_COMPLETE             1
#define COMM_STATE_DEVICE_START         100

// Maximum length of any command, including \r\n terminating it
// Most are small, but the largest commands tend to be the RX replies
#define CMD_MAX_LINELENGTH              250

// Types that enable introspective self-identification of command buffer
#define CMDBUF_TYPE_PHONE               0
#define CMDBUF_TYPE_BGEIGIE             1
#ifdef LORA
#define CMDBUF_TYPE_LORA                2
#endif
#ifdef FONA
#define CMDBUF_TYPE_FONA                3
#define CMDBUF_TYPE_FONA_DEFERRED       4
#endif

// Command buffer
typedef struct cmdbuf_s cmdbuf_t;
struct cmdbuf_s {
    // CMDBUF_TYPE
    uint16_t type;
    // COMM_STATE
    uint16_t state;
    // various flags for things that we'll try to recognize between state changes
    uint32_t recognized;
    // true if this is ready to be process()'ed
    bool complete;
    // length is +1 so we can always guarantee null termination of what's inside
    uint8_t buffer[CMD_MAX_LINELENGTH + 1];
    uint16_t length;
    // A buffer of stuff held in case we receive while we're busy processing
    uint8_t busy_buffer[250];
    uint16_t busy_nextget;
    uint16_t busy_nextput;
    uint16_t busy_length;
    // offset into buffer where completed command args begin
    uint16_t args;
    // offset to the next argument, after testing an arg via ThisArg()
    uint16_t nextarg;
};

// Why comm may be failing
#define CONNECT_STATE_UNKNOWN           0
#define CONNECT_STATE_LORA_MODULE       1
#define CONNECT_STATE_FONA_MODULE       2
#define CONNECT_STATE_WIRELESS_SERVICE  3
#define CONNECT_STATE_DATA_SERVICE      4
#define CONNECT_STATE_APP_SERVICE       5
#define CONNECT_STATE_LORA_GATEWAY      6
#define CONNECT_STATE_LORAWAN_GATEWAY   7
#define CONNECT_STATE_LORA_DESELECTED   8
#define CONNECT_STATE_FONA_DESELECTED   9
#define CONNECT_STATE_LORA_ACTIVE       10
#define CONNECT_STATE_LORAWAN_ACTIVE    11
#define CONNECT_STATE_FONA_ACTIVE       12
void comm_set_connect_state(uint16_t state);

// Public

bool comm_is_initialized();
char *comm_connect_state();
void comm_cmdbuf_init(cmdbuf_t *cmd, uint16_t type);
void comm_cmdbuf_reset(cmdbuf_t *cmd);
void comm_call_now(void);
void comm_request_state();
void comm_watchdog_reset();
void comm_reset(bool fForce);
void comm_force_cell();
void comm_flush_buffers();
void comm_initiate_service_update(bool fFull);
bool comm_update_service();
void comm_process_message_from_service(char *message);
void comm_cmdbuf_set(cmdbuf_t *cmd, char *Message);
void comm_cmdbuf_set_state(cmdbuf_t *cmd, uint16_t newstate);
bool comm_cmdbuf_append(cmdbuf_t *cmd, uint8_t databyte);
bool comm_cmdbuf_received_byte(cmdbuf_t *cmd, uint8_t databyte);
bool comm_cmdbuf_this_arg_is(cmdbuf_t *cmd, char *testCmd);
char *comm_cmdbuf_next_arg(cmdbuf_t *cmd);
void comm_enqueue_complete(uint16_t type);
bool comm_is_initialized();
bool comm_is_busy();
void comm_disable_oneshot_mode();
bool comm_uart_switching_allowed();
bool comm_oneshot_currently_enabled();
void comm_oneshot_completed();
bool comm_would_be_buffered(bool);
void comm_poll();
void comm_reinit();
void comm_init();
void comm_request_mode_on_reselect(uint16_t mode);
void comm_select_completed();
uint16_t comm_get_mtu();

#define GPS_NOT_CONFIGURED              0
#define GPS_NO_DATA                     1
#define GPS_NO_LOCATION                 2
#define GPS_LOCATION_PARTIAL            3
#define GPS_LOCATION_FULL               4
#define GPS_LOCATION_ABORTED            5
uint16_t comm_gps_get_value(float *lat, float *lon, float *alt);
void comm_gps_abort();
void comm_gps_update();
bool comm_gps_completed();
bool comm_gps_active();

// Only one mode is ever active, however this is defined bitwise so that
// we can test using a bitwise-AND operator rather than just == or switch.
#define COMM_NONE   0x0001
#define COMM_LORA   0x0002
#define COMM_FONA   0x0004
uint16_t comm_mode(void);
void comm_reselect();
void comm_deselect(char *reason);
bool comm_is_deselected();
bool comm_db_is_active();
void comm_show_state();
void comm_select(uint16_t which, char *reason);
bool comm_can_send_to_service();
bool comm_send_to_service(uint8_t *buffer, uint16_t length, uint16_t RequestType);

#define AUTOWAN_NORMAL          0
#define AUTOWAN_GPS_WAIT        1
#define AUTOWAN_FAILOVER        2
uint16_t comm_autowan_mode();

#define MSG_NOT_DECODED         0
#define MSG_TELECAST            1
#define MSG_SAFECAST            2
#define MSG_REPLY_TTGATE        3
#define MSG_REPLY_TTSERVE       4
uint16_t comm_decode_received_message(char *msg, void *message, uint8_t *buffer, uint16_t length, uint16_t *decodedBytes);

#endif // COMM_H__
