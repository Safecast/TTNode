// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

#ifndef IO_H__
#define IO_H__

#define IOUART  1
#define IOTWI   2
#define IOSPI   3

void io_init();
void io_request_restart();
void io_restart_if_requested();
uint32_t io_get_device_address();
uint16_t io_get_random(uint32_t mod);
void io_force_optimize_power();
bool io_optimize_power();
void io_power_stay_suboptimal();

#endif // IO_H__
