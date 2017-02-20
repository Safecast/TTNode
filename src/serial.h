// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

#ifndef SERIAL_H__
#define SERIAL_H__

void serial_send_string(char *str);
void serial_send_byte(uint8_t databyte);
void serial_init(uint32_t baudrate, bool hwfc);
void serial_term();
bool serial_transmit_enabled();
void serial_transmit_enable(bool fEnable);
bool serial_hwfc_enabled();
void serial_set_poll_mode(bool fPoll);
bool serial_wait_for_byte(uint8_t byte);
bool serial_uart_error_check(bool fClearOnly);

#endif // SERIAL_H__
