// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

#ifndef TWI_H__
#define TWI_H__

#include "app_twi.h"
#include "ublox.h"
#include "hih.h"
#include "max01.h"
#include "max43.h"

bool twi_init();
bool twi_term();
void twi_status_check(bool);
bool twi_schedule(char *context, app_twi_transaction_t const * p_transaction);
bool twi_completed(char *context, ret_code_t error);

#endif // TWI_H__
