// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

// Debugging support, customized for debug-via-Bluetooth

#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#include "nrf_nvic.h"
#include "softdevice_handler.h"
#include "nrf_delay.h"
#include "nrf_error.h"
#include "debug.h"
#include "serial.h"
#include "ssd.h"

#if !defined(BOOTLOADERX)
#include "twi.h"
#include "btdebug.h"
#endif

// Set program debug modes
static uint32_t the_debug_flags = DBG_NONE;

void debug_init() {
#ifdef SSD
    the_debug_flags |= DBG_SENSOR;
#endif
#ifdef COMMDEBUG
    the_debug_flags |= DBG_RX | DBG_TX;
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
#ifdef BURNCOMMS
    the_debug_flags |= DBG_RX | DBG_TX;
#endif
#if defined(BURN) || defined(SENSORDEBUG)
    the_debug_flags |= DBG_SENSOR | DBG_SENSOR_MAX;
#endif
#ifdef SENSORMAXDEBUG
    the_debug_flags |= DBG_SENSOR_MAX | DBG_SENSOR_SUPERMAX;
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
    btdebug_send_byte((uint8_t) databyte);
#endif
#ifdef SSD
    // Don't allow recursion when TWI itself is doing DEBUG_PRINTF
    if (!twi_disable_twi_debug_printf()) {
        ssd1306_write((uint8_t) databyte);
        if (databyte == '\n')
            ssd1306_display();
    }
#endif
#endif
}

// For softdevice debugging, add a "-DNRF_LOG_USES_RTT=1" to compiler flags
uint32_t log_rtt_init(void) {
    return NRF_SUCCESS;
}
void log_rtt_printf(int terminal_index, char *format_msg, ...) {
    char buffer[256];
    va_list p_args;
    va_start(p_args, format_msg);
    vsnprintf(buffer, sizeof(buffer)-1, format_msg, p_args);
    va_end(p_args);
    log_debug_write_string(buffer);
}

void log_debug_printf(char *format_msg, ...) {
    char buffer[256];
    va_list p_args;
    va_start(p_args, format_msg);
    vsnprintf(buffer, sizeof(buffer)-1, format_msg, p_args);
    va_end(p_args);
    log_debug_write_string(buffer);
}

__INLINE void log_debug_write_string_many(int num_args, ...) {
    char *msg;
    va_list p_args;
    va_start(p_args, num_args);
    for (int i = 0; i < num_args; i++) {
        msg = va_arg(p_args, char *);
        log_debug_write_string(msg);
    }
    va_end(p_args);
}

__INLINE void log_debug_write_string(char *msg) {
#if !defined(DEBUG_USES_UART) && !defined(BOOTLOADERX)
    btdebug_send_string(msg);
#ifdef SSD
    // Don't allow recursion when TWI itself is doing DEBUG_PRINTF
    if (!twi_disable_twi_debug_printf()) {
        ssd1306_putstring(msg);
        ssd1306_display();
    }
#endif
#else
    while (*msg)
        debug_putchar(*msg++);
#endif
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

    // This is the proper way of doing it, assuming that the softdevice is active.
    sd_nvic_SystemReset();

    // We should never have returned, however if we do it is perhaps because the
    // softdevice wasn't enabled.  In this case, just go direct.
    NVIC_SystemReset();

}

__WEAK void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t *p_file_name) {
    DEBUG_PRINTF("PANIC (%04x) %d:%s\n", error_code, line_num, p_file_name);

    // This is the proper way of doing it, assuming that the softdevice is active.
    sd_nvic_SystemReset();

    // We should never have returned, however if we do it is perhaps because the
    // softdevice wasn't enabled.  In this case, just go direct.
    NVIC_SystemReset();

}

__WEAK void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
    DEBUG_PRINTF("Fatal id=%08x pc=%08x info=%08x\n", id, pc, info);
    /* We can't really halt because UART output is blocked, and we don't want to brick the device */
}

__WEAK void debug_check_handler(uint32_t error_code, uint32_t line_num, uint8_t *p_file_name) {
    DEBUG_PRINTF("DEBUG_CHECK(%04x) %d:%s\n", error_code, line_num, p_file_name);
    /* We can't really halt because UART output is blocked, and we don't want to brick the device */
}
