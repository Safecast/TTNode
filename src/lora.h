// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

#ifndef COMM_LORA_H__
#define COMM_LORA_H__
#ifdef LORA

void lora_init();
void lora_term(bool fPowerdown);
void lora_process();
void lora_reset(bool Force);
void lora_request_state();
void lora_watchdog_reset();
bool lora_needed_to_be_reset();
bool lora_is_busy();
void lora_send(char *msg);
void lora_enter_command_mode();
bool lora_can_send_to_service();
bool lora_send_to_service(uint8_t *buffer, uint16_t length, uint16_t RequestType);
void lora_received_byte(uint8_t databyte);
uint16_t lora_get_mtu();

#endif // LORA
#endif // COMM_LORA_H__
