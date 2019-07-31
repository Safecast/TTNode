// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

// Support for BT in its Controller role
// Minimal procedures that are necessary to kick a characteristic's CCCD into
// such a state wherein it generates HVX write events.
// CCCD is the Client Characteristic Configuration Descriptor, which configures
// the GATT client to tell the GATT server what packets should be sent to it.

#include <stdint.h>
#include <string.h>
#include "debug.h"
#include "btcd.h"
#include "nordic_common.h"
#include "ble.h"
#include "ble_srv_common.h"

// Must be a mask of continuous zeroes, followed by continuous sequence of ones: 000...111
#define TX_BUFFER_MASK         0x07
// Size of send buffer, which is 1 higher than the mask.
#define TX_BUFFER_SIZE         (TX_BUFFER_MASK + 1)
// Length of the write message for CCCD
#define WRITE_MESSAGE_LENGTH   BLE_CCCD_VALUE_LEN

// Type of message to be transmitted to the connected central
typedef enum {
    READ_REQ,
    WRITE_REQ
} tx_request_t;

// Structure for the message to be written to the CCCD
typedef struct {
    uint8_t                  gattc_value[WRITE_MESSAGE_LENGTH];
    ble_gattc_write_params_t gattc_params;
} write_params_t;

// Structure describing a message to be transmitted to the connected central
typedef struct {
    uint16_t     conn_handle;
    tx_request_t type;
    union {
        uint16_t       read_handle;
        write_params_t write_req;
    } req;
} tx_message_t;

// Transmit buffer
static tx_message_t   m_tx_buffer[TX_BUFFER_SIZE];
// Current index in the transmit buffer where the next message should be inserted
static uint32_t       m_tx_insert_index = 0;
// Current index in the transmit buffer from where the next message to be transmitted resides
static uint32_t       m_tx_index = 0;

// Function for passing any pending request from the buffer to the stack.
void cccd_tx_buffer_process(void) {

    if (m_tx_index != m_tx_insert_index) {
        uint32_t err_code;

        if (m_tx_buffer[m_tx_index].type == READ_REQ) {
            err_code = sd_ble_gattc_read(m_tx_buffer[m_tx_index].conn_handle,
                                         m_tx_buffer[m_tx_index].req.read_handle,
                                         0);
        } else {
            err_code = sd_ble_gattc_write(m_tx_buffer[m_tx_index].conn_handle,
                                          &m_tx_buffer[m_tx_index].req.write_req.gattc_params);
        }
        if (err_code == NRF_SUCCESS) {
            DEBUG_PRINTF("SD Read/Write API returns Success.\n");
            m_tx_index++;
            m_tx_index &= TX_BUFFER_MASK;
        } else {
            DEBUG_PRINTF("SD Read/Write API returns error. This message sending will be "
                         "attempted again..\n");
        }
    }
}

// Function for creating a message to be written to the CCCD
uint32_t cccd_configure(uint16_t conn_handle, uint16_t handle_cccd, bool enable) {

    DEBUG_PRINTF("Configuring CCCD. CCCD Handle = %d, Connection Handle = %d\n",
                 handle_cccd, conn_handle);

    tx_message_t *p_msg;
    uint16_t       cccd_val = enable ? BLE_GATT_HVX_NOTIFICATION : 0;

    p_msg              = &m_tx_buffer[m_tx_insert_index++];
    m_tx_insert_index &= TX_BUFFER_MASK;

    p_msg->req.write_req.gattc_params.handle   = handle_cccd;
    p_msg->req.write_req.gattc_params.len      = WRITE_MESSAGE_LENGTH;
    p_msg->req.write_req.gattc_params.p_value  = p_msg->req.write_req.gattc_value;
    p_msg->req.write_req.gattc_params.offset   = 0;
    p_msg->req.write_req.gattc_params.write_op = BLE_GATT_OP_WRITE_REQ;
    p_msg->req.write_req.gattc_value[0]        = LSB_16(cccd_val);
    p_msg->req.write_req.gattc_value[1]        = MSB_16(cccd_val);
    p_msg->conn_handle                         = conn_handle;
    p_msg->type                                = WRITE_REQ;

    cccd_tx_buffer_process();
    return NRF_SUCCESS;

}

