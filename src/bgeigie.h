// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

#ifndef COMM_BGEIGIE_H__
#define COMM_BGEIGIE_H__

void bgeigie_init();
void bgeigie_received_byte(uint8_t databyte);
void bgeigie_process();

#endif // COMM_BGEIGIE_H__
