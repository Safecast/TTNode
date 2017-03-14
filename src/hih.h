// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

#ifndef HIH6130_H__
#define HIH6130_H__

void s_hih6130_measure(void *);
bool s_hih6130_upload_needed(void *);
bool s_hih6130_get_value(float *temp, float *humid);
void s_hih6130_clear_measurement();
bool s_hih6130_init(void *s, uint16_t param);
bool s_hih6130_term();

#endif // HIH6130_H__
