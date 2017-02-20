// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

#ifndef BME280B_H__
#define BME280B_H__

void s_bme280b_measure(void *);
bool s_bme280b_upload_needed(void *);
bool s_bme280b_get_value(float *tempC, float *humid, float *pressurePa);
void s_bme280b_clear_measurement();
bool s_bme280b_init(uint16_t param);
bool s_bme280b_term();

#endif // BME280B_H__
