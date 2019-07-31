// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

#ifndef BT_H__
#define BT_H__

void bluetooth_init();
void bluetooth_softdevice_init(void);
bool send_byte_to_bluetooth(uint8_t databyte);
bool can_send_to_bluetooth(void);
uint32_t bluetooth_session_id();
void drop_bluetooth(void);
void db_discover_evt_handler();

// This btc function is actually implemented in bt.c.
// Note that the symbol NOBTC is defined in rs.h, if it is defined at all.
#include "bti.h"
#ifndef NOBTC
void btc_scan_start(void);
#endif

#endif // BT_H__
