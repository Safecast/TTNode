// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

#ifndef BME280_H__
#define BME280_H__

void s_bme280_measure(void *);
bool s_bme280_upload_needed(void *);
bool s_bme280_get_value(float *tempC, float *humid, float *pressurePa);
void s_bme280_clear_measurement();
bool s_bme280_init();
bool s_bme280_term();

#endif // BME280_H__
