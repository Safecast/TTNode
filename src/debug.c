// Debugging support, customized for debug-via-Bluetooth

#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#include "nrf_delay.h"
#include "nrf_error.h"
#include "debug.h"
#include "io.h"
#include "serial.h"
#include "bt.h"

// Set program debug modes
static uint32_t the_debug_flags = DBG_NONE;

void debug_init() {
#ifdef COMMDEBUG
    the_debug_flags |= DBG_RX | DBG_TX | DBG_COMM_MAX;
#endif
#ifdef SENSORDEBUG
    the_debug_flags |= DBG_SENSOR;
#endif
#ifdef AIRDEBUG
    the_debug_flags |= DBG_AIR;
#endif
#ifdef GPSDEBUG
    the_debug_flags |= DBG_GPS_MAX;
#endif
}

void debug_flags_set(uint32_t flags) {
    if (flags == DBG_NONE)
        the_debug_flags = 0;
    else
        the_debug_flags |= flags;
}

uint32_t debug_flags() {
    return the_debug_flags;
}

bool debug(uint32_t flags) {
    return ((the_debug_flags & flags) != 0);
}

bool debug_flag_toggle(uint32_t flags) {
    the_debug_flags = the_debug_flags ^ flags;
    return (debug(flags));
}

void debug_putchar(char databyte) {
#if defined(DEBUG_USES_UART)
    serial_send_byte((uint8_t) databyte);
#else
#if !defined(BOOTLOADERX)
    send_byte_to_bluetooth((uint8_t) databyte);
#endif
#endif
}

// For softdevice debugging, add a "-DNRF_LOG_USES_RTT=1" to compiler flags
uint32_t log_rtt_init(void)
{
    return NRF_SUCCESS;
}
void log_rtt_printf(int terminal_index, char *format_msg, ...) {
    static char buffer[256];
    va_list p_args;
    va_start(p_args, format_msg);
    vsprintf(buffer, format_msg, p_args);
    va_end(p_args);
    log_debug_write_string(buffer);
}

void log_debug_printf(const char *format_msg, ...) {
    static char buffer[256];
    va_list p_args;
    // This is the most well-used debug command.  If
    // we're optimizing power, don't waste time here.
#ifndef BOOTLOADERX
    if (io_optimize_power())
        return;
#endif
    va_start(p_args, format_msg);
    vsprintf(buffer, format_msg, p_args);
    va_end(p_args);
    log_debug_write_string(buffer);
}

__INLINE void log_debug_write_string_many(int num_args, ...) {
    const char *msg;
    va_list p_args;
    va_start(p_args, num_args);
    for (int i = 0; i < num_args; i++) {
        msg = va_arg(p_args, const char *);
        log_debug_write_string(msg);
    }
    va_end(p_args);
}

__INLINE void log_debug_write_string(const char *msg) {
    while ( *msg ) {
        debug_putchar(*msg++);
    }
}

void log_debug_write_hex(uint32_t value) {
    uint8_t nibble;
    uint8_t i = 8;
    debug_putchar('0');
    debug_putchar('x');
    while ( i-- != 0 ) {
        nibble = (value >> (4 * i)) & 0x0F;
        debug_putchar( (nibble > 9) ? ('A' + nibble - 10) : ('0' + nibble) );
    }
}

void log_debug_write_hex_char(uint8_t c) {
    uint8_t nibble;
    uint8_t i = 2;
    while ( i-- != 0 ) {
        nibble = (c >> (4 * i)) & 0x0F;
        debug_putchar( (nibble > 9) ? ('A' + nibble - 10) : ('0' + nibble) );
    }
}

void log_debug_dump(uint8_t *buf, uint16_t length) {
    uint16_t i;
    for (i=0; i<length; i++)
        log_debug_write_hex_char(buf[i]);
    debug_putchar('\r');
    debug_putchar('\n');
}

const char *log_hex_char(const char c) {
    static volatile char hex_string[3];
    hex_string[2] = 0; // Null termination
    uint8_t nibble;
    uint8_t i = 2;
    while (i-- != 0) {
        nibble = (c >> (4 * i)) & 0x0F;
        hex_string[1 - i] = (nibble > 9) ? ('A' + nibble - 10) : ('0' + nibble);
    }
    return (const char *) hex_string;
}

const char *log_hex(uint32_t value) {
    static volatile char hex_string[11];
    hex_string[0] = '0';
    hex_string[1] = 'x';
    hex_string[10] = 0;
    uint8_t nibble;
    uint8_t i = 8;
    while (i-- != 0) {
        nibble = (value >> (4 * i)) & 0x0F;
        hex_string[9 - i] = (nibble > 9) ? ('A' + nibble - 10) : ('0' + nibble);
    }
    return (const char *)hex_string;
}

void app_trace_init(void)
{
}

void app_trace_dump(uint8_t * p_buffer, uint32_t len)
{
    DEBUG_PRINTF("\r\n");
    for (uint32_t index = 0; index <  len; index++)
    {
        DEBUG_PRINTF("0x%02X ", p_buffer[index]);
    }
    DEBUG_PRINTF("\r\n");
}

__WEAK void app_error_handler_bare(uint32_t error_code)
{
    DEBUG_PRINTF("PANIC (%04x) %d:%s\n", error_code);
    /* We can't really halt because UART output is blocked, and we don't want to brick the device */
}

__WEAK void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t *p_file_name) {
    DEBUG_PRINTF("PANIC (%04x) %d:%s\n", error_code, line_num, p_file_name);
    /* We can't really halt because UART output is blocked, and we don't want to brick the device */
}

__WEAK void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
    DEBUG_PRINTF("Fatal id=%08x pc=%08x info=%08x\n", id, pc, info);
    /* We can't really halt because UART output is blocked, and we don't want to brick the device */
}

__WEAK void debug_check_handler(uint32_t error_code, uint32_t line_num, const uint8_t *p_file_name) {
    DEBUG_PRINTF("DEBUG_CHECK(%04x) %d:%s\n", error_code, line_num, p_file_name);
    /* We can't really halt because UART output is blocked, and we don't want to brick the device */
}
