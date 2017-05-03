// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

// General lowest-level I/O device methods

#include <stdint.h>
#include "debug.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "ble_gap.h"
#include "custom_board.h"
#include "nrf_nvic.h"
#include "softdevice_handler.h"
#include "storage.h"
#include "serial.h"
#include "config.h"
#include "timer.h"
#include "sensor.h"
#include "io.h"
#include "gpio.h"
#include "comm.h"
#include "twi.h"
#include "spi.h"
#include "ssd.h"

// Device address
static uint32_t device_address = 0L;

// Optional power management in the case that the user doesn't use BT to admin the device after boot
static bool fForceOptimalPower = false;
static bool fAllowSuboptimalPower = false;

// If restart is pending
static uint32_t RestartPending = 0;

// Force us to optimize power starting NOW, used only when debugging
void io_force_optimize_power() {
    fForceOptimalPower = true;
}

// Optimize for super-low power by dropping bluetooth and other things
bool io_optimize_power() {

    // If debugging power optimization and we do a manual /drop command
    if (fForceOptimalPower)
        return (true);

    // If we're in anything but normal mode, don't drop it
    if (sensor_op_mode() != OPMODE_NORMAL)
        return false;

    // If we're debugging a device, we're willing to take the power hit in order to keep BT alive so we
    // can come in and debug it.
#if defined(BTKEEPALIVE) && !defined(POWERDEBUG)
    return false;
#endif

    // We only do this if we aren't forcing power debugging
#ifndef POWERDEBUG
    if ((storage()->flags & FLAG_BTKEEPALIVE) != 0)
        return false;
#endif

    // Don't optimize power for some period of time after boot
    if (get_seconds_since_boot() < DROP_BTADVERTISING_SECONDS)
        return (false);

    // If the user had connected with their phone, or if someone
    // (i.e. a bGeigie) had connected to us as a BT Controller,
    // make sure that we keep Bluetooth active indefinitely.
    if (fAllowSuboptimalPower)
        return (false);

    return (true);
}

// Force suboptimal power if the user is actually using the features we'd otherwise shut down
void io_power_stay_suboptimal() {
    fAllowSuboptimalPower = true;
}


// Get an address for this device as a 32-bit unsigned number
uint32_t io_get_device_address() {

    // See if one was specified in flash
    STORAGE *s = storage();
    if (s->device_id != 0L)
        return(s->device_id);

    // Determine the device address upon first call
    if (device_address == 0)
    {
        ble_gap_addr_t mac_address;
        uint32_t hi, lo;

        // Get this device's Address via its BLE-assigned MAC address.
        // Note that if this becomes an undesirable technique for whatever reason,
        // FYI another unique MAC address is available by looking at the LPWAN chip.
#if defined(NSDKV10) || defined(NSDKV11)
        uint32_t err_code = sd_ble_gap_address_get(&mac_address);
        DEBUG_CHECK(err_code);
#else
        uint32_t err_code = sd_ble_gap_addr_get(&mac_address);
        DEBUG_CHECK(err_code);
#endif

        // Make it as random as possible by XOR'ing the 48 bits to get a 32-bit number
        lo = mac_address.addr[0] | (mac_address.addr[1] << 8) | (mac_address.addr[2] << 16) | (mac_address.addr[3] << 24);
        hi = mac_address.addr[4] | (mac_address.addr[5] << 8) | (mac_address.addr[4] << 16) | (mac_address.addr[5] << 24);
        device_address = lo ^ hi;

        // Reserve the low 2^20 addresses for fixed allocation
        if (device_address < 1048576)
            device_address = ~device_address;

    }

    return (device_address);
}

// Get a random number of seconds from 0 to mod-1
uint16_t io_get_random(uint32_t mod) {
    uint8_t buffer[2];
    uint32_t rnd;
    sd_rand_application_vector_get(&buffer[0], sizeof(buffer));
    rnd = buffer[0] | (buffer[1] << 8);
    if (mod == 0)
        return(rnd);
    return ((uint16_t)((mod * rnd) / 65536L));
}

// Perform a soft reset of the device.
void io_restart_if_requested() {

    if (RestartPending == 0)
        return;

    DEBUG_PRINTF("*** ABOUT TO RESTART ***\n");
    
    // Wait several iterations of being called for things to settle down
    if (++RestartPending > 1) {

        // This is the proper way of doing it, assuming that the softdevice is active.
        sd_nvic_SystemReset();

        // We should never have returned, however if we do it is perhaps because the
        // softdevice wasn't enabled.  In this case, just go direct.
        NVIC_SystemReset();
    }

}

// Request a soft reset
void io_request_restart() {
    storage_checkpoint();
    DEBUG_PRINTF("*** REQUESTING RESTART ***\n");
    RestartPending = 1;
}

// Init the I/O subsystem.  This must be called AFTER timers have been initialized.
void io_init() {

    // Init GPIO
    gpio_init();
    
}
