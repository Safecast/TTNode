// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

#ifndef COMM_PHONE_H__
#define COMM_PHONE_H__

void phone_init();
void phone_send(char *msg);
void phone_received_byte(uint8_t databyte);
void phone_process();

#endif // COMM_PHONE_H__
