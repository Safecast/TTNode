// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

#ifndef MAX17043_H__
#define MAX17043_H__

void s_max43_voltage_measure(void *s);
bool s_max43_voltage_upload_needed(void *s);
bool s_max43_voltage_get_value(float *voltage);
void s_max43_voltage_clear_measurement();
bool s_max43_voltage_init(void *s, uint16_t param);
bool s_max43_voltage_term();
void s_max43_soc_measure(void *s);
bool s_max43_soc_upload_needed(void *s);
bool s_max43_soc_get_value(float *soc);
void s_max43_soc_clear_measurement();
bool s_max43_soc_init(void *s, uint16_t param);
bool s_max43_soc_term();

#endif // MAX17043_H__
