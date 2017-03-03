// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

void bme(measure) (void *);
bool bme(upload_needed)(void *);
bool bme(get_value)(float *tempC, float *humid, float *pressurePa);
void bme(clear_measurement)();
bool bme(init)(uint16_t param);
bool bme(term)();
