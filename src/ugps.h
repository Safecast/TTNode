// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

#ifndef UGPS_H__
#define UGPS_H__

#ifdef UGPS

void ugps_received_byte(uint8_t databyte);
bool s_ugps_init(void *s, uint16_t param);
bool s_ugps_term(void);
void s_ugps_poll(void *s);
bool s_ugps_active();
void s_ugps_update(void);
void s_ugps_clear_measurement();
void s_ugps_done_settling();
void s_ugps_shutdown();
uint16_t s_ugps_get_value(float *lat, float *lon, float *alt);
void s_ugps_received_byte(uint8_t databyte);
bool g_ugps_skip(void *g);
    
#endif // UGPS

#endif // UGPS_H__
