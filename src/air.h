// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

#ifndef AIR_H__
#define AIR_H__

bool s_air_upload_needed(void *);
void s_air_measure(void *);
void s_air_poll(void *s);
bool s_air_show_value(uint32_t when, char *buffer, uint16_t length);
bool s_air_init(void *s, uint16_t param);
bool s_air_term();

#endif // OPC_H__
