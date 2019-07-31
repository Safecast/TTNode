// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

#ifndef BTI_H__
#define BTI_H__

#include "ble.h"
#include "ble_srv_common.h"
#include <stdint.h>
#include <stdbool.h>

// If there is no bGeigie support, we don't need BTC
#ifndef BGEIGIE
#define NOBTC
#endif

// The UUID of the base service
#define BLE_UUID_BTP_SERVICE           0x2001
// The UUID of the TX characteristic
#define BLE_UUID_BTP_TX_CHARACTERISTIC 0x2002
// The UUID of the RX characteristic
#define BLE_UUID_BTP_RX_CHARACTERISTIC 0x2003

// MTU size used in the softdevice enabling and to reply to a BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST event.
#if (NRF_SD_BLE_API_VERSION >= 3)

// To work around iPhone 7 bug https://www.nordicsemi.com/eng/supportcase/view/31801
// Note that this fix is only required for iPhone 7 running iOS 10.0.1 and iOS 10.0.2; it is fixed in 10.1.
// When that iOS version is long gone, we can eliminate this AND what depends upon this symbol.
// Note that after changing MTU size, check the SD memory requirements and update the .ld file
#define IPHONE_7_BLE_BUG_DELETE_AFTER_10_POINT_2
#ifdef IPHONE_7_BLE_BUG_DELETE_AFTER_10_POINT_2
#define NRF_BLE_MAX_MTU_SIZE 158
#else
#define NRF_BLE_MAX_MTU_SIZE GATT_MTU_SIZE_DEFAULT
#endif

#endif

// Maximum length of data (in bytes) that can be transmitted to the controller (this is 23-3==20)
#define BTP_MAX_DATA_LEN (GATT_MTU_SIZE_DEFAULT - 3)

// Maximum length of the TX characteristic data, in bytes
#define BTP_MAX_TX_CHAR_LEN BTP_MAX_DATA_LEN
// Maximum length of the RX characteristic data, in bytes
#define BTP_MAX_RX_CHAR_LEN BTP_MAX_DATA_LEN

// UUID for the Service (16-byte/128-bit vendor specific)
// Generated using http://www.itu.int/en/ITU-T/asn1/Pages/UUID/uuids.aspx, then bytes reversed because this is least-significant first
// Note that the third and fourth values from the right (what I set FYI to 0xFF 0xFF) are replaced with the 16-bit BLE_UUID_* from above
// For RS-as-Peripheral (such as an iPhone app connecting to us to relay data), this is our device's advertised UUID
#define UUID128_SIZE        16
#define BTP_BASE_UUID       {{0x1B, 0xC5, 0xD5, 0xA5, 0x02, 0x00, 0x6D, 0xAC, 0xE5, 0x11, 0x69, 0xFA, 0xFF, 0xFF, 0x55, 0xC8}}
#define BTP_SERVICE_UUID_TYPE BLE_UUID_TYPE_VENDOR_BEGIN

// For RS-as-Controller, this is the UUID of the peripheral device for which we'll accept inbound connections
#define BTC_SERVICE_UUID    {{0x7F, 0x5D, 0x79, 0xF4, 0xA5, 0x05, 0x3F, 0xBD, 0xFF, 0x41, 0xBE, 0xC3, 0x8C, 0x0D, 0x08, 0xEF}}
#define BTC_RX_CHAR_UUID    {{0x3B, 0x09, 0xB0, 0xE0, 0xDF, 0x69, 0xC6, 0x87, 0x4C, 0x4E, 0x6B, 0x69, 0xB1, 0xF5, 0xE8, 0xA1}}

// Forward declarations of our primary structure types for both peripheral and controller
typedef struct btp_s btp_t;
typedef struct btc_s btc_t;

// Event handler types
typedef void (*btp_data_handler_t) (btp_t *p_btp, uint8_t *p_data, uint16_t length);
typedef void (*btc_evt_handler_t) ();

// BTC service context structure
struct btc_s {
    // Service UUID
    ble_uuid_t service_uuid;
    // Characteristic UUID
    ble_uuid_t rx_uuid;
    // Connection handle, as provided by the SoftDevice
    uint16_t conn_handle;
    // Handle of the CCCD
    uint16_t cccd_handle;
    // Handle related to the RX characteristic, as provided by the SoftDevice
    uint16_t rx_handle;
    // App event handler to be called on an event related to the sewrvice
    btc_evt_handler_t evt_handler;
};

// Controller event types
typedef enum {
    // Event indicating that the service has been discovered at the peer
    BTC_EVT_DISCOVERY_COMPLETE = 1,
    // Event indicating that a notification of the characteristic has been received from the peer
    BTC_EVT_NOTIFICATION = 2
} btc_evt_type_t;

// Controller Event structure.
typedef struct {
    // Event type
    btc_evt_type_t evt_type;
    // Data received
    union {
        btc_t btc;
    } params;
} btc_evt_t;

// BTP service context structure
struct btp_s {
    // UUID type for Relay Service Base UUID
    uint8_t                  uuid_type;
    // Handle of Relay Service, as provided by the SoftDevice
    uint16_t                 service_handle;
    // Related to the TX characteristic, as provided by the SoftDevice
    ble_gatts_char_handles_t tx_handles;
    // Related to the RX characteristic, as provided by the SoftDevice
    ble_gatts_char_handles_t rx_handles;
    // Handle of the current connection, as provided by the SoftDevice, or BLE_CONN_HANDLE_INVALID if no connection
    uint16_t                 conn_handle;
    // Whether or not the peer has enabled notification of the RX characteristic
    bool                     is_notification_enabled;
    // Event handler to be called for handling received data
    btp_data_handler_t       data_handler;
};

// BTP Exports
void btp_init(btp_t *p_btp);
void btp_on_ble_evt(btp_t *p_btp, ble_evt_t *p_ble_evt);
uint32_t btp_string_send(btp_t *p_btp, uint8_t *p_string, uint16_t length);
bool btp_can_send(btp_t *p_btp);

// BTC Exports
void btc_init(btc_t *p_btc);
void btc_evt_handler(btc_t *p_btc, btc_evt_t *p_btc_evt);
void btc_on_ble_evt(btc_t *p_btc, ble_evt_t *p_ble_evt);

#endif // BTI_H__
