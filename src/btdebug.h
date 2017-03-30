// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

#ifndef BTDEBUG_H_
#define BTDEBUG_H_

void btdebug_send_byte(uint8_t databyte);
void btdebug_send_string(char *str);
void btdebug_create_timer();

#endif  // BTDEBUG_H_
