// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

#ifndef DEBUGX_H_
#define DEBUGX_H_

#include <stdint.h>
#include <stdarg.h>
#include <stdbool.h>

#define STRINGIZE(x) #x
#define STRINGIZE_VALUE_OF(x) STRINGIZE(x)

#ifndef UNUSED_VARIABLE
#define UNUSED_VARIABLE(X)  ((void)(X))
#endif

#ifndef UNUSED_PARAMETER
#define UNUSED_PARAMETER(X) UNUSED_VARIABLE(X)
#endif

// Set debug modes
#define DBG_RX              0x00000001L
#define DBG_TX              0x00000002L
#define DBG_AIR             0x00000004L
#define DBG_SENSOR          0x00000008L
#define DBG_GPS_MAX         0x00000010L
#define DBG_SENSOR_MAX      0x00000020L
#define DBG_SENSOR_SUPERMAX 0x00000040L
#define DBG_SENSOR_SUPERDUPERMAX 0x00000080L
#define DBG_SENSOR_POLL     0x00000100L
#define DBG_COMM_MAX        0x00000200L
#define DBG_BT              0x00000400L
#define DBG_NONE            0
#define DBG_COMMON          (DBG_RX|DBG_TX|DBG_AIR|DBG_SENSOR)
void debug_flags_set(uint32_t flags);
uint32_t debug_flags();
bool debug(uint32_t flag);
bool debug_flag_toggle(uint32_t flag);
void debug_init();

#define DBG_NUM_VA_ARGS(...) (sizeof((char*[]){ 0, ##__VA_ARGS__ })/sizeof(char*)-1)

void log_debug_printf(char *format_msg, ...);
void log_debug_write_char(char c);
void log_debug_write_string_many(int num_args, ...);
void log_debug_write_string(char *msg);
void log_debug_write_hex(uint32_t value);
void log_debug_write_hex_char(uint8_t c);

#define DEBUG_PRINTF(...)           log_debug_printf(__VA_ARGS__)
#define DEBUG_PRINTF_DEBUG(...)     log_debug_printf(__VA_ARGS__)
#define DEBUG_PRINTF_ERROR(...)     log_debug_printf(__VA_ARGS__)

#define DEBUG_STR(...)              log_debug_write_string_many(DBG_NUM_VA_ARGS(__VA_ARGS__), ##__VA_ARGS__)
#define DEBUG_DEBUG(...)            log_debug_write_string_many(DBG_NUM_VA_ARGS(__VA_ARGS__), ##__VA_ARGS__)
#define DEBUG_ERROR(...)            log_debug_write_string_many(DBG_NUM_VA_ARGS(__VA_ARGS__), ##__VA_ARGS__)

#define DEBUG_HEX(val)              log_debug_write_hex(val)
#define DEBUG_HEX_DEBUG(val)        log_debug_write_hex(val)
#define DEBUG_HEX_ERROR(val)        log_debug_write_hex(val)

#define DEBUG_HEX_CHAR(val)         log_debug_write_hex_char(val)
#define DEBUG_HEX_CHAR_DEBUG(val)   log_debug_write_hex_char(val)
#define DEBUG_HEX_CHAR_ERROR(val)   log_debug_write_hex_char(val)

void debug_check_handler(uint32_t error_code, uint32_t line_num, uint8_t *p_file_name);

#define DEBUG_HANDLER(ERR_CODE)                                         \
    do                                                                  \
    {                                                                   \
        debug_check_handler((ERR_CODE), __LINE__, (uint8_t*) __FILE__); \
    } while (0)

#define DEBUG_CHECK(ERR_CODE)                       \
    do                                              \
    {                                               \
        uint32_t LOCAL_ERR_CODE = (ERR_CODE); \
        if (LOCAL_ERR_CODE != NRF_SUCCESS)          \
        {                                           \
            DEBUG_HANDLER(LOCAL_ERR_CODE);          \
        }                                           \
    } while (0)

#define DEBUG_CHECK_BOOL(BOOLEAN_VALUE)                         \
    do                                                          \
    {                                                           \
        uint32_t LOCAL_BOOLEAN_VALUE = (BOOLEAN_VALUE);   \
        if (!LOCAL_BOOLEAN_VALUE)                               \
        {                                                       \
            DEBUG_HANDLER(0);                                   \
        }                                                       \
    } while (0)

#endif // DEBUGX_H_
