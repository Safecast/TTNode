// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

#ifndef LORAFP_H__
#define LORAFP_H__
#ifdef LORA

bool lorafp_get_command(char *region, bool loraWAN, uint16_t cmdno, char *buffer, uint16_t length);

#endif // LORA
#endif // LORAFP_H__
