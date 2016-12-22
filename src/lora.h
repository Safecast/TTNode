// LoRa/LoraWAN communications definitions

#ifndef COMM_LORA_H__
#define COMM_LORA_H__
#ifdef LORA

// Public
void lora_init();
void lora_process();
void lora_reset(bool Force);
void lora_request_state();
void lora_watchdog_reset();
bool lora_needed_to_be_reset();
bool lora_is_busy();
void lora_send(char *msg);
void lora_set_listen_tags(char *newtags);
char *lora_get_listen_tags();
void lora_enter_command_mode();
bool lora_can_send_to_service();
bool lora_send_to_service(char *comment, uint8_t *buffer, uint16_t length, uint16_t RequestType);
void lora_received_byte(uint8_t databyte);

#endif // LORA
#endif // COMM_LORA_H__
