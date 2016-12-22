// LoRa/LoraWAN frequency plan support

#ifndef LORAFP_H__
#define LORAFP_H__
#ifdef LORA

bool lorafp_get_command(char *region, bool loraWAN, uint16_t cmdno, char *buffer, uint16_t length);

#endif // LORA
#endif // LORAFP_H__
