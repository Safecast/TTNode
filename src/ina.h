// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

#ifndef INA219_H__
#define INA219_H__

void s_ina_measure(void *s);
bool s_ina_upload_needed(void *s);
bool s_ina_get_value(float *pBusVoltage, float *pSOC, float *pCurrent);
bool s_ina_show_value(uint32_t when, char *buffer, uint16_t length);
void s_ina_clear_measurement();
bool s_ina_init(void *s, uint16_t param);
bool s_ina_term();
void s_ina_poll(void *s);

#endif // INA219_H__
