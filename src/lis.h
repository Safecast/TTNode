// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

#ifndef LIS3DH_H__
#define LIS3DH_H__

void s_lis_measure(void *s);
bool s_lis_init(void *s, uint16_t param);
bool s_lis_term();
void s_lis_poll(void *s);

#endif // LIS3DH_H__
