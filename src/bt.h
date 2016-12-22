// Bluetooth support

#ifndef BT_H__
#define BT_H__

// Init
void bluetooth_init();
void bluetooth_softdevice_init(void);

// Misc
void send_byte_to_bluetooth(uint8_t databyte);
bool can_send_to_bluetooth(void);
uint32_t bluetooth_session_id();

// This btc function is actually implemented in bt.c.
// Note that the symbol NOBTC is defined in rs.h, if it is defined at all.
#include "bti.h"
#ifndef NOBTC
void btc_scan_start(void);
#endif

// BTLE Solicitation
void drop_bluetooth(void);
void db_discover_evt_handler();

#endif // BT_H__
