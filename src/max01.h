// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

#ifndef MAX17201_H__
#define MAX17201_H__

void s_max01_measure(void *s);
bool s_max01_upload_needed(void *s);
bool s_max01_get_value(float *pVoltage, float *pSOC, float *pCurrent);
bool s_max01_show_value(uint32_t when, char *buffer, uint16_t length);
void s_max01_clear_measurement();
bool s_max01_init(void *s, uint16_t param);
bool s_max01_term();
void s_max01_poll(void *s);

#endif // MAX17201_H__
