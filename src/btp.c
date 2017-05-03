// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

// Support for TTNode BT in its Peripheral role

#include <stdint.h>
#include <string.h>
#include "bti.h"
#include "debug.h"
#include "phone.h"
#include "serial.h"
#include "nrf_delay.h"
#include "nordic_common.h"
#include "ble_srv_common.h"
#include "app_timer.h"

// Handle the BLE_GATTS_EVT_WRITE event
void on_write(btp_t *p_btp, ble_evt_t *p_ble_evt) {
    ble_gatts_evt_write_t *p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
    if (
        (p_evt_write->handle == p_btp->rx_handles.cccd_handle)
        &&
        (p_evt_write->len == 2)
        ) {
        if (ble_srv_is_notification_enabled(p_evt_write->data)) {
            p_btp->is_notification_enabled = true;
        } else {
            p_btp->is_notification_enabled = false;
        }
    } else if (
        (p_evt_write->handle == p_btp->tx_handles.value_handle)
        &&
        (p_btp->data_handler != NULL)
        ) {
        p_btp->data_handler(p_btp, p_evt_write->data, p_evt_write->len);
    } else {
        // Do Nothing. This event is not relevant for this service.
    }
}

// Function for adding RX characteristic.
uint32_t rx_char_add(btp_t *p_btp) {
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    /* Add the receive characteristic that we'll expect as a peripheral */
    memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);

    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.notify = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_btp->uuid_type;
    ble_uuid.uuid = BLE_UUID_BTP_RX_CHARACTERISTIC;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    attr_md.vloc    = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen    = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(uint8_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = BTP_MAX_RX_CHAR_LEN;

    return sd_ble_gatts_characteristic_add(p_btp->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_btp->rx_handles);

}

// Function for adding TX characteristic
uint32_t tx_char_add(btp_t *p_btp) {
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.write         = 1;
    char_md.char_props.write_wo_resp = 1;
    char_md.p_char_user_desc         = NULL;
    char_md.p_char_pf                = NULL;
    char_md.p_user_desc_md           = NULL;
    char_md.p_cccd_md                = NULL;
    char_md.p_sccd_md                = NULL;

    ble_uuid.type = p_btp->uuid_type;
    ble_uuid.uuid = BLE_UUID_BTP_TX_CHARACTERISTIC;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    attr_md.vloc    = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen    = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = 1;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = BTP_MAX_TX_CHAR_LEN;

    return sd_ble_gatts_characteristic_add(p_btp->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_btp->tx_handles);
}


// BTP BLE event handler, called from the main ble_evt_dispatch
void btp_on_ble_evt(btp_t *p_btp, ble_evt_t *p_ble_evt) {

    if ((p_btp == NULL) || (p_ble_evt == NULL)) {
        return;
    }

    switch (p_ble_evt->header.evt_id) {

    case BLE_GATTS_EVT_WRITE:
        on_write(p_btp, p_ble_evt);
        break;

    }
}

// Function for handling the data from the Relay Service.
void btp_data_handler(btp_t *p_btp, uint8_t *p_data, uint16_t length) {
    for (uint32_t i = 0; i < length; i++)
        phone_received_byte(p_data[i]);
}

// BTP initialization, called from main services_init
void btp_init(btp_t *p_btp) {

    // Initialize the service structure.
    p_btp->conn_handle             = BLE_CONN_HANDLE_INVALID;
    p_btp->data_handler            = btp_data_handler;
    p_btp->is_notification_enabled = false;

    /* [Adding proprietary Service to S110 SoftDevice] */
    // Add our custom base 128-bit UUID to the internal table.
    // Note that the second-to-highest order 16 bit word is actually ignored
    // in all use of that table, because it will be overridden
    // by a 16-bit UUID "shortcut" specified below.
    uint32_t      err_code;
    ble_uuid_t    ble_uuid;
    ble_uuid128_t btp_base_uuid = BTP_BASE_UUID;

    err_code = sd_ble_uuid_vs_add(&btp_base_uuid, &p_btp->uuid_type);
    if (err_code != NRF_SUCCESS) {
        DEBUG_CHECK(err_code);
        return;
    }

    // Our 16-bit UUID "shortcut" that will be overlaid on the above
    ble_uuid.uuid = BLE_UUID_BTP_SERVICE;
    ble_uuid.type = p_btp->uuid_type;

    // Add the service.
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_btp->service_handle);
    if (err_code != NRF_SUCCESS) {
        DEBUG_CHECK(err_code);
        return;
    }

    // Add the RX Characteristic.
    err_code = rx_char_add(p_btp);
    if (err_code != NRF_SUCCESS) {
        DEBUG_CHECK(err_code);
        return;
    }

    // Add the TX Characteristic.
    err_code = tx_char_add(p_btp);
    if (err_code != NRF_SUCCESS) {
        DEBUG_CHECK(err_code);
        return;
    }

}

// Method to check to see if we are connected
bool btp_can_send(btp_t *p_btp) {

    if (p_btp == NULL)
        return false;

    // We aren't fully connected until we get our first on_write which enables notifications
    if ((p_btp->conn_handle == BLE_CONN_HANDLE_INVALID) || (!p_btp->is_notification_enabled))
        return false;

    return true;

}

// Send a single packet to host by generating an HVX notification on the rx characteristic
uint32_t btp_string_send(btp_t *p_btp, uint8_t *p_string, uint16_t length) {
    ble_gatts_hvx_params_t hvx_params;
    uint32_t status;

    if (!btp_can_send(p_btp))
        return NRF_ERROR_INVALID_STATE;
    if (length > BTP_MAX_DATA_LEN)
        return NRF_ERROR_INVALID_PARAM;

    memset(&hvx_params, 0, sizeof(hvx_params));

    hvx_params.handle = p_btp->rx_handles.value_handle;
    hvx_params.p_data = p_string;
    hvx_params.p_len  = &length;
    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;

    status = sd_ble_gatts_hvx(p_btp->conn_handle, &hvx_params);

#ifdef DEBUG_USES_UART
    if (status == 0x3401) {
        // Handy debugging note because of the bizarre error code:
        // https://devzone.nordicsemi.com/question/6085/strange-error-code-13313-0x3401-returned-by-sd_ble_gatts_hvx
        if (debug(DBG_BT))
            DEBUG_PRINTF("*** GATTS system attributes missing ***\n");
    } else if (status == NRF_ERROR_INVALID_STATE) {
        // Error that cropped up in SDK12 with MTU request not completing?
        // https://devzone.nordicsemi.com/question/93779/s132-30-mtu-exchange-not-completing/
        if (debug(DBG_BT))
            DEBUG_PRINTF("*** Invalid State or MTU Exchange not completed ***\n");
    } else {
        if (debug(DBG_BT))
            DEBUG_PRINTF("sd_ble_gatts_hvx(hdl=0x%04x) error 0x%04x\n", p_btp->conn_handle, status);
    }
#endif

    return(status);

}
