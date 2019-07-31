// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

#ifndef UBLOXM8_H__
#define UBLOXM8_H__

bool s_gps_init(void *s, uint16_t param);
bool s_gps_term(void);
void s_gps_poll(void *s);
void s_gps_shutdown();
uint16_t s_gps_get_value(float *lat, float *lon, float *alt);

#endif // UBLOXM8_H__
