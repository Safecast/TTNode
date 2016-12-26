//  Serial I/O Support

#ifndef SERIAL_H__
#define SERIAL_H__

void serial_send_string(char *str);
void serial_send_byte(uint8_t databyte);
void serial_init(uint32_t baudrate, bool hwfc);
void serial_transmit_enable(bool fEnable);
void serial_set_poll_mode(bool fPoll);
bool serial_wait_for_byte(uint8_t byte);
bool serial_uart_error_check(bool fClearOnly);

#endif // SERIAL_H__
