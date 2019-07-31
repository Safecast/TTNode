// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

// Support for BT in its Controller role

#include <stdint.h>
#include <string.h>
#include "bti.h"
#include "debug.h"
#include "btcd.h"
#include "comm.h"
#include "io.h"
#include "bgeigie.h"
#include "bt.h"
#include "ble_db_discovery.h"
#include "nordic_common.h"
#include "ble_srv_common.h"
#include "app_timer.h"
#ifndef NOBONDING
#include "peer_manager.h"
#endif

// This is defined in bti.h, as a function of BGEIGIE
#ifndef NOBTC

// Function for handling Handle Value Notification received from the SoftDevice.
void on_hvx(btc_t *p_btc, ble_evt_t *p_ble_evt) {
    ble_gattc_evt_hvx_t *p_notif = &p_ble_evt->evt.gattc_evt.params.hvx;

    // Check if this is OUR notification
    if (p_ble_evt->evt.gattc_evt.params.hvx.handle == p_btc->rx_handle) {

        // Once we've received data, flag that we do NOT want to optimize power
        // by shutting down BT listens, even if this particular connection
        // happens to drop.
        io_power_stay_suboptimal();

        /* Received data! */
        if (comm_can_send_to_service())
            for (uint32_t i = 0; i < p_notif->len; i++)
                bgeigie_received_byte(p_notif->data[i]);

        /* Dispatch, just for good measure */
        btc_evt_t btc_evt;
        btc_evt.evt_type = BTC_EVT_NOTIFICATION;
        p_btc->evt_handler(p_btc, &btc_evt);

    }

}

// Function for handling write response events.
void on_write_response(btc_t *p_btc, ble_evt_t *p_ble_evt) {
    // Check if there is any message to be sent across to the peer and send it.
    cccd_tx_buffer_process();
}

// BTC event handler called from our general BT event handler
void btc_on_ble_evt(btc_t *p_btc, ble_evt_t *p_ble_evt) {

    if ((p_btc == NULL) || (p_ble_evt == NULL)) {
        return;
    }

    switch (p_ble_evt->header.evt_id) {
    case BLE_GAP_EVT_CONNECTED:
        p_btc->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
        break;

    case BLE_GATTC_EVT_HVX:
        on_hvx(p_btc, p_ble_evt);
        break;

    case BLE_GATTC_EVT_WRITE_RSP:
        on_write_response(p_btc, p_ble_evt);
        break;

    default:
        break;
    }
}

// Handles events for the controller
void btc_evt_handler(btc_t *p_btc, btc_evt_t *p_btc_evt) {

    switch (p_btc_evt->evt_type) {
    case BTC_EVT_DISCOVERY_COMPLETE: {

        if (debug(DBG_BT))
            DEBUG_PRINTF("BTC service discovered\r\n");

        // Initiate bonding.
#ifndef NOBONDING
        {
            ret_code_t err_code;
            err_code = pm_conn_secure(p_btc->conn_handle, false);
            if (err_code != NRF_ERROR_INVALID_STATE) {
                DEBUG_CHECK(err_code);
            }
        }
#endif

        // Service discovered. Configure the CCCD to enable HVX notification.
        ret_code_t err_code;
        err_code = cccd_configure(p_btc->conn_handle, p_btc->cccd_handle, true);
        DEBUG_CHECK(err_code);

    }
        break; // BTC_EVT_DISCOVERY_COMPLETE

    case BTC_EVT_NOTIFICATION: {
        // Nothing to do for now because it's taken care of in hvx handler
    } break; // BTC_EVT_NOTIFICATION

    default:
        if (debug(DBG_BT))
            DEBUG_PRINTF("BTC Service event ID=%d\n", p_btc_evt->evt_type);
        // No implementation needed.
        break;
    }
}

// BTC initialization
void btc_init(btc_t *p_btc) {
    uint32_t err_code;
    uint8_t *uuid128;
    ble_uuid128_t btc_service_uuid = BTC_SERVICE_UUID;
    ble_uuid128_t btc_rx_uuid = BTC_RX_CHAR_UUID;

    // Add custom base UUIDs by pulling out the magic numbers that
    // the bluetooth stack utilizes as "shortcut" UUID's rather than
    // the full 128-bit UUIDs.  The only time that this will break
    // down is if we ever have a true conflict between those exact
    // bytes between multiple UUID's that we're trying to register.
    uuid128 = (uint8_t *) &btc_service_uuid;
    p_btc->service_uuid.uuid = uuid128[12] | (uuid128[13] << 8);
    err_code = sd_ble_uuid_vs_add(&btc_service_uuid, &p_btc->service_uuid.type);
    DEBUG_CHECK(err_code);
    uuid128 = (uint8_t *) &btc_rx_uuid;
    p_btc->rx_uuid.uuid = uuid128[12] | (uuid128[13] << 8);
    err_code = sd_ble_uuid_vs_add(&btc_rx_uuid, &p_btc->rx_uuid.type);
    DEBUG_CHECK(err_code);

    // Initialize discovery
    {
#ifdef NSDKV10
        ret_code_t errcode = ble_db_discovery_init();
#else
        ret_code_t errcode = ble_db_discovery_init(db_discover_evt_handler);
#endif
        DEBUG_CHECK(errcode);
    }

    // Begin discovery to try to find the service and its attributes
    p_btc->evt_handler     = btc_evt_handler;
    p_btc->conn_handle     = BLE_CONN_HANDLE_INVALID;
    p_btc->cccd_handle     = BLE_GATT_HANDLE_INVALID;
#ifdef NSDKV10
    err_code = ble_db_discovery_evt_register(&p_btc->service_uuid, db_discover_evt_handler);
#else
    err_code = ble_db_discovery_evt_register(&p_btc->service_uuid);
#endif
    DEBUG_CHECK(err_code);

}

#endif // NOBTC
