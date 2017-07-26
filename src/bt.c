// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

// Top-level Bluetooth support,
// also requiring specific Controller and Peripheral support files

#include "debug.h"
#include "bti.h"
#include "config.h"
#include "comm.h"
#include "lora.h"
#include "bt.h"
#include "btcd.h"
#include "timer.h"
#include "sensor.h"
#include "io.h"
#include "serial.h"
#include "storage.h"
#include "ble_hci.h"
#include "ble_advertising.h"
#include "ble_db_discovery.h"
#include "ble_conn_state.h"
#include "ble_conn_params.h"
#include "softdevice_handler.h"
#include "nrf_delay.h"
#include "custom_board.h"

#ifdef DFU
#if defined(NSDKV10) || defined(NSDKV11)
#define OLD_DFU
#else
#define NEW_DFU
#endif
#endif

#ifdef DFU
#include "ble_dfu.h"
#ifdef OLD_DFU
#include "device_manager.h"
#include "dfu_app_handler.h"
#endif
#endif

#ifndef NOBONDING
#include "peer_manager.h"
#include "fds.h"
#include "fstorage.h"
#endif

#if !defined(NSDKV10) && !defined(NSDKV11)
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#endif

#ifdef OLD_DFU
#define DFU_REV_MAJOR                    0x00
#define DFU_REV_MINOR                    0x01
#endif

// Required if we do DFU, but be cautious because when this changes, all devices
// that have previously paired with this peripheral will have trouble connecting
// because of invalid cached databases.  On iPhone it means un-pairing, turning off
// bluetooth, rebooting phone, etc., before pairing will function again.
#ifdef DFU
#define IS_SRVC_CHANGED_CHARACT_PRESENT  1
#else
// Note that as of 12/22/2016 I've changed this to be always ON because it is
// just too confusing for the phone to keep flipping back and forth between
// ON and OFF when alternately using DFU vs not using DFU.  There's little
// downside to this except for the memory layout in the .LD file.
#define IS_SRVC_CHANGED_CHARACT_PRESENT  1
#endif

// Things are more sophisticated in the new SDK as far as resource allocation
// When changing either of these, remember to adjust the RAM settings.
#ifndef NSDKV10
#define CENTRAL_LINK_COUNT         1
#define PERIPHERAL_LINK_COUNT      1
#endif

// Speed of advertising.  The flow is fast -> slow -> idle, at which point we restart.
#define APP_INITIAL_ADV_MODE            BLE_ADV_MODE_FAST
#define APP_ADV_FAST_INTERVAL           320     // Units of 0.625ms == 200ms
#define APP_ADV_SLOW_INTERVAL          1600     // Units of 0.625ms = 1000ms
#ifndef POWERDEBUG
#define APP_ADV_FAST_TIMEOUT            120     // Units of seconds
#define APP_ADV_SLOW_TIMEOUT            300     // Units of seconds
#else
#define APP_ADV_FAST_TIMEOUT            10      // Units of seconds
#define APP_ADV_SLOW_TIMEOUT            10      // Units of seconds
#endif

// GAP scanning/connection interval, in units of 0.625ms
#define SCAN_INTERVAL                   0x00A0
// GAP scan window, in units of 0.625ms
#define SCAN_WINDOW                     0x0050
// GAP scan timeout
#define SCAN_TIMEOUT                    0
// GAP minimum connection period
#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(7.5, UNIT_1_25_MS)
// GAP maximum acceptable connection time
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)
// GAP connection slave latency
#define SLAVE_LATENCY                   3
// GAP connection supervisory timeout
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)

// BLE connection, time from initiating event (connect or start of notif) to 1st call to sd_ble_gap_conn_param_update
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)
// BLE connection, time between each call to sd_ble_gap_conn_param_update after the first call
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER)
// BLE connection, number of attempts before giving up the connection parameter negotiation w/BLE controller
#define MAX_CONN_PARAMS_UPDATE_COUNT    3

// The max number of centrals that can concurrently connect to us as a peripheral
#define MAX_CONNECTED_CENTRALS          1

// Security - do or do not perform bonding
#define SEC_PARAM_BOND                  1
// Security - man-in-the-middle protection required, or not
#define SEC_PARAM_MITM                  0
// Security - LE Secure Connections enabled or not
#define SEC_PARAM_LESC                  0
// Security - Keypress notifications enabled, or not
#define SEC_PARAM_KEYPRESS              0
// Security - I/O capabilities
#define SEC_PARAM_IO_CAPABILITIES        BLE_GAP_IO_CAPS_NONE
// Security - out-of-band data available, or not
#define SEC_PARAM_OOB                    0
// Security - minimum encryption key size in bytes
#define SEC_PARAM_MIN_KEY_SIZE           7
// Security - maximum encryption key size in bytes
#define SEC_PARAM_MAX_KEY_SIZE           16

// OTA DFU parameters
#ifdef OLD_DFU
// Handle of first app-specific service when service-changed characteristic is present
#define APP_SERVICE_HANDLE_START         0x000C
// Max handle value in BLE
#define BLE_HANDLE_MAX                   0xFFFF
// When we are using DFU, we MUST have the service-changed characteristic present
STATIC_ASSERT(IS_SRVC_CHANGED_CHARACT_PRESENT);
#endif

// General-purpose variable-length data descriptor, used for advertising
typedef struct {
    uint8_t      *p_data;
    uint16_t      data_len;
} data_t;

// GAP scan parameters
#ifndef NOBTC
#if defined(NSDKV10) || defined(NSDKV11)
static ble_gap_scan_params_t gap_scan_params = {
    // Active scanning not set.
    0,
    // Selective scanning not set.
    0,
    // No whitelist provided.
    NULL,
    SCAN_INTERVAL,
    SCAN_WINDOW,
    // No timeout.
    0x0000
};
#else
static const ble_gap_scan_params_t gap_scan_params =
{
    .active   = 1,
    .interval = SCAN_INTERVAL,
    .window   = SCAN_WINDOW,
    .timeout  = SCAN_TIMEOUT,
#if (NRF_SD_BLE_API_VERSION == 2)
    .selective   = 0,
    .p_whitelist = NULL,
#endif
#if (NRF_SD_BLE_API_VERSION >= 3)
    .use_whitelist = 0,
#endif
};
#endif
#endif

// GAP connection parameters
#ifndef NOBTC
static ble_gap_conn_params_t gap_conn_params = {
    (uint16_t)MIN_CONN_INTERVAL,
    (uint16_t)MAX_CONN_INTERVAL,
    0,
    (uint16_t)CONN_SUP_TIMEOUT
};
#endif

// Indicate whether BT is still active, or has been dropped
static bool bluetooth_dropped = false;

// Peripheral context
static btp_t m_btp;
static uint16_t btp_conn_handle = BLE_CONN_HANDLE_INVALID;
static uint32_t current_bluetooth_session_id = 0L;

// Controller context
#ifndef NOBTC
static btc_t m_btc;
static uint16_t btc_conn_handle  = BLE_CONN_HANDLE_INVALID;
#endif

// Discovery context
#ifndef NOBTC
static ble_db_discovery_t m_ble_db_discovery_btc;
#endif

// Our service's UID to be advertised
static ble_uuid_t btp_adv_uuids[] = {{BLE_UUID_BTP_SERVICE, BTP_SERVICE_UUID_TYPE}};

// True if we're actively advertising
static bool m_advertising = false;

#ifndef NOBTC
// True if we're actively gap scanning
static bool m_scanning = false;
#endif

// DFU context
#ifdef DFU
#ifdef OLD_DFU
static dm_application_instance_t m_app_handle;
#endif
static ble_dfu_t m_dfus;
#endif

// Forward references
#ifndef NOBONDING
void peer_manager_init(bool erase_bonds);
#endif

// Device manager events for dfu
#ifdef DFU
#ifdef OLD_DFU
ret_code_t device_manager_evt_handler(dm_handle_t const *p_handle,
                                      dm_event_t const *p_event,
                                      ret_code_t event_result);
#endif
void dfu_init(void);
#endif

// Debugging, so we can view message types more easily

#ifdef BLEDEBUG
char *get_ble_event_name(uint16_t eventID) {
    char *s = NULL;
    switch (eventID) {
    case BLE_EVT_TX_COMPLETE:
        s = "BLE_EVT_TX_COMPLETE";
        break;
    case BLE_EVT_USER_MEM_REQUEST:
        s = "BLE_EVT_USER_MEM_REQUEST";
        break;
    case BLE_EVT_USER_MEM_RELEASE:
        s = "BLE_EVT_USER_MEM_RELEASE";
        break;
    case BLE_EVT_DATA_LENGTH_CHANGED:
        s = "BLE_EVT_DATA_LENGTH_CHANGED";
        break;
    case BLE_GAP_EVT_CONNECTED:
        s = "BLE_GAP_EVT_CONNECTED";
        break;
    case BLE_GAP_EVT_DISCONNECTED:
        s = "BLE_GAP_EVT_DISCONNECTED";
        break;
    case BLE_GAP_EVT_CONN_PARAM_UPDATE:
        s = "BLE_GAP_EVT_CONN_PARAM_UPDATE";
        break;
    case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
        s = "BLE_GAP_EVT_SEC_PARAMS_REQUEST";
        break;
    case BLE_GAP_EVT_SEC_INFO_REQUEST:
        s = "BLE_GAP_EVT_SEC_INFO_REQUEST";
        break;
    case BLE_GAP_EVT_PASSKEY_DISPLAY:
        s = "BLE_GAP_EVT_PASSKEY_DISPLAY";
        break;
    case BLE_GAP_EVT_KEY_PRESSED:
        s = "BLE_GAP_EVT_KEY_PRESSED";
        break;
    case BLE_GAP_EVT_AUTH_KEY_REQUEST:
        s = "BLE_GAP_EVT_AUTH_KEY_REQUEST";
        break;
    case BLE_GAP_EVT_LESC_DHKEY_REQUEST:
        s = "BLE_GAP_EVT_LESC_DHKEY_REQUEST";
        break;
    case BLE_GAP_EVT_AUTH_STATUS:
        s = "BLE_GAP_EVT_AUTH_STATUS";
        break;
    case BLE_GAP_EVT_CONN_SEC_UPDATE:
        s = "BLE_GAP_EVT_CONN_SEC_UPDATE";
        break;
    case BLE_GAP_EVT_TIMEOUT:
        s = "BLE_GAP_EVT_TIMEOUT";
        break;
    case BLE_GAP_EVT_RSSI_CHANGED:
        s = "BLE_GAP_EVT_RSSI_CHANGED";
        break;
    case BLE_GAP_EVT_ADV_REPORT:
        s = "BLE_GAP_EVT_ADV_REPORT";
        break;
    case BLE_GAP_EVT_SEC_REQUEST:
        s = "BLE_GAP_EVT_SEC_REQUEST";
        break;
    case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
        s = "BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST";
        break;
    case BLE_GAP_EVT_SCAN_REQ_REPORT:
        s = "BLE_GAP_EVT_SCAN_REQ_REPORT";
        break;
    case BLE_GATTC_EVT_PRIM_SRVC_DISC_RSP:
        s = "BLE_GATTC_EVT_PRIM_SRVC_DISC_RSP";
        break;
    case BLE_GATTC_EVT_REL_DISC_RSP:
        s = "BLE_GATTC_EVT_REL_DISC_RSP";
        break;
    case BLE_GATTC_EVT_CHAR_DISC_RSP:
        s = "BLE_GATTC_EVT_CHAR_DISC_RSP";
        break;
    case BLE_GATTC_EVT_DESC_DISC_RSP:
        s = "BLE_GATTC_EVT_DESC_DISC_RSP";
        break;
    case BLE_GATTC_EVT_ATTR_INFO_DISC_RSP:
        s = "BLE_GATTC_EVT_ATTR_INFO_DISC_RSP";
        break;
    case BLE_GATTC_EVT_CHAR_VAL_BY_UUID_READ_RSP:
        s = "BLE_GATTC_EVT_CHAR_VAL_BY_UUID_READ_RSP";
        break;
    case BLE_GATTC_EVT_READ_RSP:
        s = "BLE_GATTC_EVT_READ_RSP";
        break;
    case BLE_GATTC_EVT_CHAR_VALS_READ_RSP:
        s = "BLE_GATTC_EVT_CHAR_VALS_READ_RSP";
        break;
    case BLE_GATTC_EVT_WRITE_RSP:
        s = "BLE_GATTC_EVT_WRITE_RSP";
        break;
    case BLE_GATTC_EVT_HVX:
        s = "BLE_GATTC_EVT_HVX";
        break;
    case BLE_GATTC_EVT_EXCHANGE_MTU_RSP:
        s = "BLE_GATTC_EVT_EXCHANGE_MTU_RSP";
        break;
    case BLE_GATTC_EVT_TIMEOUT:
        s = "BLE_GATTC_EVT_TIMEOUT";
        break;
    case BLE_GATTS_EVT_WRITE:
        s = "BLE_GATTS_EVT_WRITE";
        break;
    case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        s = "BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST";
        break;
    case BLE_GATTS_EVT_SYS_ATTR_MISSING:
        s = "BLE_GATTS_EVT_SYS_ATTR_MISSING";
        break;
    case BLE_GATTS_EVT_HVC:
        s = "BLE_GATTS_EVT_HVC";
        break;
    case BLE_GATTS_EVT_SC_CONFIRM:
        s = "BLE_GATTS_EVT_SC_CONFIRM";
        break;
    case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
        s = "BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST";
        break;
    case BLE_GATTS_EVT_TIMEOUT:
        s = "BLE_GATTS_EVT_TIMEOUT";
        break;
    case BLE_L2CAP_EVT_RX:
        s = "BLE_L2CAP_EVT_RX";
        break;
    }
    return(s);
}
#endif

#ifdef NEW_DFU

static void ble_dfu_evt_handler(ble_dfu_t * p_dfu, ble_dfu_evt_t * p_evt)
{
    switch (p_evt->type)
    {
    case BLE_DFU_EVT_INDICATION_DISABLED:
        NRF_LOG_INFO("Indication for BLE_DFU is disabled\r\n");
        break;

    case BLE_DFU_EVT_INDICATION_ENABLED:
        NRF_LOG_INFO("Indication for BLE_DFU is enabled\r\n");
        break;

    case BLE_DFU_EVT_ENTERING_BOOTLOADER:
        NRF_LOG_INFO("Device is entering bootloader mode!\r\n");
        break;
    default:
        NRF_LOG_INFO("Unknown event from ble_dfu\r\n");
        break;
    }
}

#endif

// Set up all necessary GAP parameters, permissions, and appearance
void gap_params_init(void) {
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    char namebuf[32];
    STORAGE *f = storage();
    char *default_label = STRINGIZE_VALUE_OF(STORAGE_LABEL);
    if (f->device_label[0] == '\0' || strcmp(f->device_label, default_label) == 0)
        sprintf(namebuf, "%lu", io_get_device_address());
    else
        strlcpy(namebuf, f->device_label, sizeof(namebuf));
    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (uint8_t *) namebuf,
                                          strlen(namebuf));
    DEBUG_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    DEBUG_CHECK(err_code);
}


// Handle Connection Parameters events
void on_conn_params_evt(ble_conn_params_evt_t *p_evt) {
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED) {
        err_code = sd_ble_gap_disconnect(btp_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        DEBUG_CHECK(err_code);
    }
}

// Handle errors from the Connection Parameters module
void conn_params_error_handler(uint32_t nrf_error) {
    DEBUG_CHECK(nrf_error);
}

// Initialize the Connection Parameters module
void conn_params_init(void) {
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID; // Start upon connection.
    cp_init.disconnect_on_fail             = true;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    DEBUG_CHECK(err_code);
}

// Stop advertising
void advertising_stop(void) {
    if (m_advertising) {
        sd_ble_gap_adv_stop();
        m_advertising = false;
    }
}

// Gracefully prepare for a system reset, in advance of a DFU update
#ifdef OLD_DFU
void reset_prepare(void) {
    if (btp_conn_handle != BLE_CONN_HANDLE_INVALID) {
        sd_ble_gap_disconnect(btp_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        btp_conn_handle = BLE_CONN_HANDLE_INVALID;
    }
    advertising_stop();
    ble_conn_params_stop();
    nrf_delay_ms(500);
}
#endif

// Handle advertising events
void on_adv_evt(ble_adv_evt_t ble_adv_evt) {

    switch (ble_adv_evt) {
    case BLE_ADV_EVT_FAST:
        if (debug(DBG_BT))
            DEBUG_PRINTF("Advertising\n");
        break;
    case BLE_ADV_EVT_IDLE: {
        ret_code_t err_code;
        if (debug(DBG_BT))
            DEBUG_PRINTF("Advertising idle; restarting\n");
        m_advertising = false;
        if (!io_optimize_power()) {
            err_code = ble_advertising_start(APP_INITIAL_ADV_MODE);
            if (err_code)
                DEBUG_CHECK(err_code);
            m_advertising = true;
        }
        break;
    }
    default:
        if (debug(DBG_BT))
            DEBUG_PRINTF("Advertising event: %d\n", ble_adv_evt);
        break;
    }
}

// Initialize advertising subsystem
void advertising_init(void) {
    uint32_t      err_code;
    ble_advdata_t advdata;
    ble_adv_modes_config_t options;

    // Build advertising data struct to pass into ble_advertising_init.
    memset(&advdata, 0, sizeof(advdata));
    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = false;
    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    advdata.uuids_complete.uuid_cnt = sizeof(btp_adv_uuids) / sizeof(btp_adv_uuids[0]);
    advdata.uuids_complete.p_uuids  = btp_adv_uuids;

    // Advertising params
    memset(&options, 0, sizeof(options));
    options.ble_adv_fast_enabled  = true;
    options.ble_adv_fast_interval = APP_ADV_FAST_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_FAST_TIMEOUT;
    options.ble_adv_slow_enabled  = true;
    options.ble_adv_slow_interval = APP_ADV_SLOW_INTERVAL;
    options.ble_adv_slow_timeout  = APP_ADV_SLOW_TIMEOUT;
    err_code = ble_advertising_init(&advdata, NULL, &options, on_adv_evt, NULL);
    DEBUG_CHECK(err_code);
}

// Init the device manager
#ifdef OLD_DFU
void device_manager_init(bool erase_bonds) {

    uint32_t               err_code;
    dm_init_param_t        init_param = {.clear_persistent_data = erase_bonds};
    dm_application_param_t register_param;

    err_code = dm_init(&init_param);
    if (err_code == NRF_ERROR_INVALID_PARAM) {
        // Note that dm_init needs to use pstorage to store bond information.  Because of this
        // it will use pstorage_register to register its need for its own storage blocks.
        // If this fails, it is because pstorage hasn't been configured with enough pages
        // for both the app and dm_ to both store their stuff in pstorage.  This can be fixed
        // by editing your pstorage_platform.h and by increasing PSTORAGE_NUM_OF_PAGES by one.
        if (debug(DBG_BT))
            DEBUG_PRINTF("*** dm_init failed because of inability to init pstorage - see comment **\n");
    }
    DEBUG_CHECK(err_code);

    memset(&register_param.sec_param, 0, sizeof(ble_gap_sec_params_t));

    register_param.sec_param.bond         = SEC_PARAM_BOND;
    register_param.sec_param.mitm         = SEC_PARAM_MITM;
    register_param.sec_param.lesc         = SEC_PARAM_LESC;
    register_param.sec_param.keypress     = SEC_PARAM_KEYPRESS;
    register_param.sec_param.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    register_param.sec_param.oob          = SEC_PARAM_OOB;
    register_param.sec_param.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    register_param.sec_param.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
    register_param.evt_handler            = device_manager_evt_handler;
    register_param.service_type           = DM_PROTOCOL_CNTXT_GATT_SRVR_ID;

    err_code = dm_register(&m_app_handle, &register_param);
    DEBUG_CHECK(err_code);

}

#endif

// start scanning and advertising
#ifndef NOBONDING
static void adv_scan_start(void)
{
    ret_code_t err_code;
    uint32_t count;

    //check if there are no flash operations in progress
    err_code = fs_queued_op_count_get(&count);
    DEBUG_CHECK(err_code);

    if(count == 0)
    {
        // Start scanning for peripherals and initiate connection to devices which
        // advertise UUIDs that we're looking for
#ifndef NOBTC
        btc_scan_start();
#endif

        // Start advertising.
        err_code = ble_advertising_start(APP_INITIAL_ADV_MODE);
        DEBUG_CHECK(err_code);
        m_advertising = true;

    } else {
        if (debug(DBG_BT))
            DEBUG_PRINTF("Cant' start scanning - flash write in progress\n");
    }

}
#endif

// Initialize bluetoooth services
void bluetooth_init() {
    uint32_t       err_code;

    // Note that we ALWAYS erase bonds
    // in the case of buttonless DFU because failing to do so
    // causes problems in dfu_transport_ble.c.
    // In dfu_transport_update_start() there is a call to
    // dfu_ble_peer_data_get, which returns an ERROR if
    // bonds are preserved. (MAJOR debugging found this.)
#if defined(OLD_DFU) || !defined(NOBONDING)
    bool erase_bonds = true;
#endif

    // Init device manager, potentially erasing bonds
#ifdef OLD_DFU
    device_manager_init(erase_bonds);
#endif

    // Init peer manager
#ifndef NOBONDING
    peer_manager_init(erase_bonds);
#endif

    // Init GAP and connection parameters
    gap_params_init();
    conn_params_init();

    // Init the btp instance used by the peripheral role
    btp_init(&m_btp);

    // Init advertising
    advertising_init();

    // Init the btc instance.
    // Note that we MUST do this after the above because the primary service that is
    // advertised to others is the one that is registered first, and we want that
    // to be the peripheral service.
#ifndef NOBTC
    btc_init(&m_btc);
#endif

    // Start scanning for peripherals
#ifndef NOBTC
    btc_scan_start();
#endif

    // Init OTA DFU (must be after bluetooth & device_manager init)
    // but must be done before BTP, because the primary service UUID
    // needs to be the DFU service in order for DFU transport to
    // rediscover this device after the jump to bootloader
#ifdef DFU
    dfu_init();
#endif

    // Start advertising
    err_code = ble_advertising_start(APP_INITIAL_ADV_MODE);
    DEBUG_CHECK(err_code);
    m_advertising = true;

}

// Initialize DFU services
#ifdef DFU
void dfu_init(void) {
    uint32_t err_code;
    ble_dfu_init_t   dfus_init;

    // Initialize the Device Firmware Update Service.
    memset(&dfus_init, 0, sizeof(dfus_init));

#ifdef OLD_DFU
    dfus_init.evt_handler   = dfu_app_on_dfu_evt;
    dfus_init.error_handler = NULL;
    dfus_init.evt_handler   = dfu_app_on_dfu_evt;
    dfus_init.revision      = ((DFU_REV_MAJOR << 8) | DFU_REV_MINOR);
#endif

#ifdef NEW_DFU
    dfus_init.evt_handler                               = ble_dfu_evt_handler;
    dfus_init.ctrl_point_security_req_write_perm        = SEC_SIGNED;
    dfus_init.ctrl_point_security_req_cccd_write_perm   = SEC_SIGNED;
#endif

    err_code = ble_dfu_init(&m_dfus, &dfus_init);
    DEBUG_CHECK(err_code);

#ifdef OLD_DFU
    dfu_app_reset_prepare_set(reset_prepare);
    dfu_app_dm_appl_instance_set(m_app_handle);
#endif

}
#endif

// Parse advertisement data, yielding advertising report if it's found
uint32_t adv_report_parse(uint8_t type, data_t *p_advdata, data_t *p_typedata) {
    uint32_t  index = 0;
    uint8_t *p_data;

    p_data = p_advdata->p_data;

    while (index < p_advdata->data_len) {
        uint8_t field_length = p_data[index];
        uint8_t field_type   = p_data[index + 1];

        if (field_type == type) {
            p_typedata->p_data   = &p_data[index + 2];
            p_typedata->data_len = field_length - 1;
            return NRF_SUCCESS;
        }
        index += field_length + 1;
    }
    return NRF_ERROR_NOT_FOUND;
}

// As a controller, stop scanning for peripherals
#ifndef NOBTC
void btc_scan_stop(void) {
    if (m_scanning) {
        sd_ble_gap_scan_stop();
        m_scanning = false;
    }
}
#endif

// Start scanning
#ifndef NOBTC
void btc_scan_start(void) {
    ret_code_t err_code;
    // If we were previously scanning, stop.
    btc_scan_stop();
    // Only start the scan if we're not optimizing for power
    if (!io_optimize_power()) {
        err_code = sd_ble_gap_scan_start(&gap_scan_params);
        // It is okay to ignore this error since we are stopping the scan anyway.
        if (err_code != NRF_ERROR_INVALID_STATE)
            DEBUG_CHECK(err_code);
        m_scanning = true;
    }
}
#endif

// Handle Peer Manager events
#ifndef NOBONDING
void pm_evt_handler(pm_evt_t const *p_evt) {
    ret_code_t err_code;

    switch(p_evt->evt_id)
    {
    case PM_EVT_BONDED_PEER_CONNECTED:
    {
        if (debug(DBG_BT))
            DEBUG_PRINTF("Connected to previously bonded device\r\n");
        err_code = pm_peer_rank_highest(p_evt->peer_id);
        if (err_code != NRF_ERROR_BUSY)
        {
            APP_ERROR_CHECK(err_code);
        }
    }break;//PM_EVT_BONDED_PEER_CONNECTED

    case PM_EVT_CONN_SEC_START:
        break;//PM_EVT_CONN_SEC_START

    case PM_EVT_CONN_SEC_SUCCEEDED:
    {
        if (debug(DBG_BT))
            DEBUG_PRINTF("Link secured. Role: %d. conn_handle: %d, Procedure: %d\r\n",
                         ble_conn_state_role(p_evt->conn_handle),
                         p_evt->conn_handle,
                         p_evt->params.conn_sec_succeeded.procedure);
        err_code = pm_peer_rank_highest(p_evt->peer_id);
        if (err_code != NRF_ERROR_BUSY)
        {
            APP_ERROR_CHECK(err_code);
        }
    }break;//PM_EVT_CONN_SEC_SUCCEEDED

    case PM_EVT_CONN_SEC_FAILED:
    {
        /** In some cases, when securing fails, it can be restarted directly. Sometimes it can
         *  be restarted, but only after changing some Security Parameters. Sometimes, it cannot
         *  be restarted until the link is disconnected and reconnected. Sometimes it is
         *  impossible, to secure the link, or the peer device does not support it. How to
         *  handle this error is highly application dependent. */
        switch (p_evt->params.conn_sec_failed.error)
        {
        case PM_CONN_SEC_ERROR_PIN_OR_KEY_MISSING:
            // Rebond if one party has lost its keys.
            err_code = pm_conn_secure(p_evt->conn_handle, true);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;//PM_CONN_SEC_ERROR_PIN_OR_KEY_MISSING

        default:
            break;
        }
    }break;//PM_EVT_CONN_SEC_FAILED

    case PM_EVT_CONN_SEC_CONFIG_REQ:
    {
        // Reject pairing request from an already bonded peer.
        pm_conn_sec_config_t conn_sec_config = {.allow_repairing = false};
        pm_conn_sec_config_reply(p_evt->conn_handle, &conn_sec_config);
    }break;//PM_EVT_CONN_SEC_CONFIG_REQ

    case PM_EVT_STORAGE_FULL:
    {
        // Run garbage collection on the flash.
        err_code = fds_gc();
        if (err_code == FDS_ERR_BUSY || err_code == FDS_ERR_NO_SPACE_IN_QUEUES)
        {
            // Retry.
        }
        else
        {
            APP_ERROR_CHECK(err_code);
        }
    }break;//PM_EVT_STORAGE_FULL

    case PM_EVT_ERROR_UNEXPECTED:
        // Assert.
        APP_ERROR_CHECK(p_evt->params.error_unexpected.error);
        break;//PM_EVT_ERROR_UNEXPECTED

    case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
        break;//PM_EVT_PEER_DATA_UPDATE_SUCCEEDED

    case PM_EVT_PEER_DATA_UPDATE_FAILED:
        // Assert.
        APP_ERROR_CHECK_BOOL(false);
        break;//PM_EVT_PEER_DATA_UPDATE_FAILED

    case PM_EVT_PEER_DELETE_SUCCEEDED:
        break;//PM_EVT_PEER_DELETE_SUCCEEDED

    case PM_EVT_PEER_DELETE_FAILED:
        // Assert.
        APP_ERROR_CHECK(p_evt->params.peer_delete_failed.error);
        break;//PM_EVT_PEER_DELETE_FAILED

    case PM_EVT_PEERS_DELETE_SUCCEEDED:
        adv_scan_start();
        break;//PM_EVT_PEERS_DELETE_SUCCEEDED

    case PM_EVT_PEERS_DELETE_FAILED:
        // Assert.
        APP_ERROR_CHECK(p_evt->params.peers_delete_failed_evt.error);
        break;//PM_EVT_PEERS_DELETE_FAILED

    case PM_EVT_LOCAL_DB_CACHE_APPLIED:
        break;//PM_EVT_LOCAL_DB_CACHE_APPLIED

    case PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED:
        // The local database has likely changed, send service changed indications.
        pm_local_database_has_changed();
        break;//PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED

    case PM_EVT_SERVICE_CHANGED_IND_SENT:
        break;//PM_EVT_SERVICE_CHANGED_IND_SENT

    case PM_EVT_SERVICE_CHANGED_IND_CONFIRMED:
        break;//PM_EVT_SERVICE_CHANGED_IND_CONFIRMED

    default:
        // No implementation needed.
        break;
    }
}
#endif // NOBONDING

// Handle BLE Stack events concerning central applications.
void on_ble_central_evt(ble_evt_t *p_ble_evt) {

#ifndef NOBTC
    // The saved addresses of others we attempted to connect to.
    static ble_gap_addr_t periph_addr_btc;
#endif

    // For readability.
    ble_gap_evt_t * p_gap_evt = &p_ble_evt->evt.gap_evt;

    // Debug code for maximum tracing of event flow
#ifdef BLEDEBUG
    if (p_ble_evt->header.evt_id != BLE_GAP_EVT_ADV_REPORT && debug(DBG_BT)) {
        char *event_name = get_ble_event_name(p_ble_evt->header.evt_id);
        if (event_name == NULL)
            DEBUG_PRINTF("[[ on_ble_central_evt type=0x%04x\n", p_ble_evt->header.evt_id);
        else
            DEBUG_PRINTF("[[ on_ble_central_evt %s\n", event_name);
    }
#endif

    switch (p_ble_evt->header.evt_id) {

#ifndef NOBTC
        // Upon connection, check which peripheral has connected (HR or BTC), initiate DB
        // discovery, update LEDs status and resume scanning if necessary.
    case BLE_GAP_EVT_CONNECTED: {
        // For readability.
        ble_gap_addr_t *peer_addr = &p_gap_evt->params.connected.peer_addr;

        // Check which peer has connected, save the connection handle and initiate DB discovery.
        // DB discovery will invoke a callback (btcc_evt_handler)
        // upon completion, which is used to enable notifications from the peer.
        if (memcmp(&periph_addr_btc, peer_addr, sizeof(ble_gap_addr_t)) == 0) {
            uint32_t err_code;

            // Reset the peer address we had saved.
            memset(&periph_addr_btc, 0, sizeof(ble_gap_addr_t));

            // Save this connection handle
            btc_conn_handle = p_gap_evt->conn_handle;

            // Start discovery
            if (debug(DBG_BT))
                DEBUG_PRINTF("BTC connect - start  DB discovery for BTC\n");
            err_code = ble_db_discovery_start(&m_ble_db_discovery_btc, p_gap_evt->conn_handle);
            DEBUG_CHECK(err_code);

        }

        // Resume scanning
        btc_scan_start();
        break; // BLE_GAP_EVT_CONNECTED

    }

        // Upon disconnection, reset the connection handle of the peer which disconnected
    case BLE_GAP_EVT_DISCONNECTED: {

        if (p_gap_evt->conn_handle == btc_conn_handle) {
            if (debug(DBG_BT))
                DEBUG_PRINTF("BTCentral disconnected (reason 0x%02x)\n", p_gap_evt->params.disconnected.reason);
            btc_conn_handle = BLE_CONN_HANDLE_INVALID;
        }

        // Start scanning again after disconnect
        btc_scan_start();

        break; // BLE_GAP_EVT_DISCONNECTED
    }

    case BLE_GAP_EVT_ADV_REPORT: {
        uint32_t err_code;
        data_t   adv_data;
        data_t   type_data;

        // For readibility.
        ble_gap_addr_t   *peer_addr = &p_gap_evt->params.adv_report.peer_addr;

        // Initialize advertisement report for parsing.
        adv_data.p_data     = (uint8_t *)p_gap_evt->params.adv_report.data;
        adv_data.data_len   = p_gap_evt->params.adv_report.dlen;

        err_code = adv_report_parse(BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_MORE_AVAILABLE,
                                    &adv_data,
                                    &type_data);

        // DEBUG_PRINTF("Adv: len=%d, errcode=%d, type_data=%d\n", adv_data.data_len, err_code, type_data);

        if (err_code != NRF_SUCCESS) {
            // Look for the services in 'complete' if it was not found in 'more available'.
            err_code = adv_report_parse(BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_COMPLETE,
                                        &adv_data,
                                        &type_data);

            if (err_code != NRF_SUCCESS) {
                // If we can't parse the data, then exit.
                break;
            }
        }

        // Verify if any UUID matches the BTC UUID

        for (uint32_t u_index = 0; u_index < (type_data.data_len / UUID128_SIZE); u_index++) {
            bool        do_connect = false;
            ble_uuid128_t btc_service_uuid = BTC_SERVICE_UUID;

            // We do not want to connect to two peripherals offering the same service, so when
            // a UUID is matched, we check that we are not already connected to a peer which
            // offers the same service. We then save the peer address, so that upon connection
            // we can tell which peer has connected and update its respective connection
            // handle.

            if ((0 == memcmp(&btc_service_uuid, &type_data.p_data[u_index * UUID128_SIZE], UUID128_SIZE)) &&
                (btc_conn_handle == BLE_CONN_HANDLE_INVALID)) {
                do_connect = true;
                memcpy(&periph_addr_btc, peer_addr, sizeof(ble_gap_addr_t));
            }

            if (do_connect) {
                if (debug(DBG_BT))
                    DEBUG_PRINTF("BTController initiating connection to known peripheral\n");
                // Initiate connection.
                err_code = sd_ble_gap_connect(peer_addr, &gap_scan_params, &gap_conn_params);
                if (debug(DBG_BT)) {
                    if (err_code != NRF_SUCCESS) {
                        DEBUG_PRINTF("Connection Request Failed (reason 0x%02x)\n", err_code);
                    } else {
                        DEBUG_PRINTF("Success\n");
                    }
                }
            }
        }
        break; // BLE_GAP_ADV_REPORT
    }

#endif // NOBTC

    case BLE_GAP_EVT_TIMEOUT: {
        // We have not specified a timeout for scanning, so only connection attemps can timeout.
        if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_SCAN) {
            if (debug(DBG_BT))
                DEBUG_PRINTF("Scan timed out.\r\n");
#ifndef NOBTC
            btc_scan_start();
#endif
        } else if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN) {
            if (debug(DBG_BT))
                DEBUG_PRINTF("Connection Request timed out.\n");
        }
        break; // BLE_GAP_EVT_TIMEOUT
    }

    case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST: {
        // Accept parameters requested by peer.
        ret_code_t err_code;
        err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                                &p_gap_evt->params.conn_param_update_request.conn_params);
        DEBUG_CHECK(err_code);
        break; // BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST
    }

#if !defined(NSDKV10) && !defined(NSDKV11)

    case BLE_GATTC_EVT_TIMEOUT:
        // Disconnect on GATT Client timeout event.
        if (debug(DBG_BT))
            DEBUG_PRINTF("GATT Client Timeout.\r\n");
        ret_code_t err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                                    BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        DEBUG_CHECK(err_code);
        break; // BLE_GATTC_EVT_TIMEOUT

    case BLE_GATTS_EVT_TIMEOUT:
        // Disconnect on GATT Server timeout event.
        if (debug(DBG_BT))
            DEBUG_PRINTF("GATT Server Timeout.\r\n");
        err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                         BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        DEBUG_CHECK(err_code);
        break; // BLE_GATTS_EVT_TIMEOUT

#if (NRF_SD_BLE_API_VERSION >= 3)

    case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
        if (debug(DBG_BT))
            DEBUG_PRINTF("Sending Reply to GATTS MTU Request. (C)\r\n");
        err_code = sd_ble_gatts_exchange_mtu_reply(p_ble_evt->evt.gatts_evt.conn_handle, NRF_BLE_MAX_MTU_SIZE);
        DEBUG_CHECK(err_code);
        break; // BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST

    case BLE_GATTC_EVT_EXCHANGE_MTU_RSP:
        if (debug(DBG_BT))
            DEBUG_PRINTF("Received MTU response (C,size=%d); exchange completed.\r\n", p_ble_evt->evt.gattc_evt.params.exchange_mtu_rsp.server_rx_mtu);
        break; // BLE_GATTC_EVT_EXCHANGE_MTU_RSP

#endif

#endif // !10 & !11

    case BLE_GATTC_EVT_PRIM_SRVC_DISC_RSP:
    case BLE_GATTC_EVT_CHAR_DISC_RSP:
    case BLE_GATTC_EVT_DESC_DISC_RSP:
    case BLE_GATTC_EVT_WRITE_RSP: {
        // Just interesting things that happen during central discovery that we don't want debugging output for
        break;
    }

    case BLE_GATTC_EVT_HVX: {
        // Nothing to do because this is handled down in the hvc handler
        break; // BLE_GATTC_EVT_HVX
    }

    default: {
        if (p_ble_evt->header.evt_id != BLE_GAP_EVT_ADV_REPORT) {
            if (debug(DBG_BT))
                DEBUG_PRINTF("Unknown central event: %u (0x%02x)\n", (uint16_t) p_ble_evt->header.evt_id, p_ble_evt->header.evt_id);
        }
        // No implementation needed.
        break;
    }

    } // Switch

// Observe entry/exit pairings
#ifdef BLEDEBUG
    if (p_ble_evt->header.evt_id != BLE_GAP_EVT_ADV_REPORT && debug(DBG_BT)) {
        char *event_name = get_ble_event_name(p_ble_evt->header.evt_id);
        if (event_name == NULL)
            DEBUG_PRINTF("]] on_ble_central_evt type=0x%04x DONE\n", p_ble_evt->header.evt_id);
        else
            DEBUG_PRINTF("]] on_ble_central_evt %s DONE\n", event_name);
    }
#endif

}


// SoftDevice peripheral event handler
void on_ble_peripheral_evt(ble_evt_t *p_ble_evt) {
    uint32_t err_code;

    // Debug code for maximum tracing of event flow
#ifdef BLEDEBUG
    if (p_ble_evt->header.evt_id != BLE_GAP_EVT_ADV_REPORT && debug(DBG_BT)) {
        char *event_name = get_ble_event_name(p_ble_evt->header.evt_id);
        if (event_name == NULL)
            DEBUG_PRINTF("[[ on_ble_peripheral_evt type=0x%04x\n", p_ble_evt->header.evt_id);
        else
            DEBUG_PRINTF("[[ on_ble_peripheral_evt %s\n", event_name);
    }
#endif

    switch (p_ble_evt->header.evt_id) {
    case BLE_GAP_EVT_CONNECTED:
#ifdef IPHONE_7_BLE_BUG_DELETE_AFTER_10_POINT_2
        // see bti.h
        err_code = sd_ble_gattc_exchange_mtu_request(p_ble_evt->evt.common_evt.conn_handle, NRF_BLE_MAX_MTU_SIZE);
        DEBUG_CHECK(err_code);
#endif
        current_bluetooth_session_id = (io_get_random(0) << 16) | io_get_random(0);
        btp_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
        m_btp.conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
        err_code = sd_ble_gatts_sys_attr_set(btp_conn_handle, NULL, 0, 0);
        DEBUG_CHECK(err_code);
        if (debug(DBG_BT))
            DEBUG_PRINTF("Peripheral connected\n");
        break;

    case BLE_GAP_EVT_DISCONNECTED:
        current_bluetooth_session_id = 0;
        if (debug(DBG_BT))
            DEBUG_PRINTF("Peripheral disconnected\n");
        btp_conn_handle = BLE_CONN_HANDLE_INVALID;
        m_btp.conn_handle = BLE_CONN_HANDLE_INVALID;
        m_btp.is_notification_enabled = false;
        break;

    case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
        // Pairing not supported
        err_code = sd_ble_gap_sec_params_reply(btp_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
        DEBUG_CHECK(err_code);
        break;

    case BLE_GATTS_EVT_SYS_ATTR_MISSING:
        // No system attributes have been stored.
        err_code = sd_ble_gatts_sys_attr_set(btp_conn_handle, NULL, 0, 0);
        DEBUG_CHECK(err_code);
        break;

    case BLE_GATTS_EVT_WRITE:
        // Write event
        break;

#if !defined(NSDKV10) && !defined(NSDKV11)
    case BLE_GATTC_EVT_TIMEOUT:
        // Disconnect on GATT Client timeout event.
        if (debug(DBG_BT))
            DEBUG_PRINTF("GATT Client Timeout.\n");
        err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                         BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        DEBUG_CHECK(err_code);
        break; // BLE_GATTC_EVT_TIMEOUT

    case BLE_GATTS_EVT_TIMEOUT:
        // Disconnect on GATT Server timeout event.
        if (debug(DBG_BT))
            DEBUG_PRINTF("GATT Server Timeout.\r\n");
        err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                         BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        DEBUG_CHECK(err_code);
        break; // BLE_GATTS_EVT_TIMEOUT

    case BLE_EVT_USER_MEM_REQUEST:
        err_code = sd_ble_user_mem_reply(btp_conn_handle, NULL);
        DEBUG_CHECK(err_code);
        break; // BLE_EVT_USER_MEM_REQUEST

    case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
    {
        ble_gatts_evt_rw_authorize_request_t  req;
        ble_gatts_rw_authorize_reply_params_t auth_reply;

        req = p_ble_evt->evt.gatts_evt.params.authorize_request;

        if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
        {
            if ((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ)     ||
                (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) ||
                (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
            {
                if (req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
                {
                    auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
                }
                else
                {
                    auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
                }
                // Bizarre, from examples: "Reply when unsupported features are requested"
                auth_reply.params.write.gatt_status = BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2;
                err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                           &auth_reply);
                DEBUG_CHECK(err_code);
            }
        }
    } break; // BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST

#if (NRF_SD_BLE_API_VERSION >= 3)

    case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
        if (debug(DBG_BT))
            DEBUG_PRINTF("Sending Reply to GATTS MTU Request. (P)\r\n");
        err_code = sd_ble_gatts_exchange_mtu_reply(p_ble_evt->evt.gatts_evt.conn_handle, NRF_BLE_MAX_MTU_SIZE);
        DEBUG_CHECK(err_code);
        break; // BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST

    case BLE_GATTC_EVT_EXCHANGE_MTU_RSP:
        if (debug(DBG_BT))
            DEBUG_PRINTF("Received MTU response (P,size=%d); exchange completed.\r\n", p_ble_evt->evt.gattc_evt.params.exchange_mtu_rsp.server_rx_mtu);
        break; // BLE_GATTC_EVT_EXCHANGE_MTU_RSP

#endif

#endif // !10 & !11

    default:
        if ((p_ble_evt->header.evt_id < BLE_EVT_BASE || p_ble_evt->header.evt_id > BLE_EVT_LAST)
            && p_ble_evt->header.evt_id != BLE_GAP_EVT_ADV_REPORT) {
            if (debug(DBG_BT))
                DEBUG_PRINTF("Peripheral event: %u 0x%04x\n", p_ble_evt->header.evt_id, p_ble_evt->header.evt_id);
        }
        // No implementation needed.
        break;
    }

    // Observe entry/exit pairings
#ifdef BLEDEBUG
    if (p_ble_evt->header.evt_id != BLE_GAP_EVT_ADV_REPORT && debug(DBG_BT)) {
        char *event_name = get_ble_event_name(p_ble_evt->header.evt_id);
        if (event_name == NULL)
            DEBUG_PRINTF("]] on_ble_peripheral_evt type=0x%04x DONE\n", p_ble_evt->header.evt_id);
        else
            DEBUG_PRINTF("]] on_ble_peripheral_evt %s DONE\n", event_name);
    }
#endif

}

// Function for handling events from the database discovery module.
// If a peer is discovered, the event handler will be called.
void db_discover_evt_handler(ble_db_discovery_evt_t *p_evt) {

    if (debug(DBG_BT))
        DEBUG_PRINTF("db_discover event, type=%d (0x%04x)\n", p_evt->evt_type, p_evt->evt_type);

#ifndef NOBTC
    // Check if the correct service was discovered.
    if (p_evt->evt_type == BLE_DB_DISCOVERY_COMPLETE &&
        p_evt->params.discovered_db.srv_uuid.uuid == m_btc.service_uuid.uuid &&
        p_evt->params.discovered_db.srv_uuid.type == m_btc.service_uuid.type) {

        m_btc.conn_handle = p_evt->conn_handle;

        // Find the CCCD Handle of the receive characteristic
        uint32_t i;

        for (i = 0; i < p_evt->params.discovered_db.char_count; i++) {

            if (p_evt->params.discovered_db.charateristics[i].characteristic.uuid.uuid ==
                m_btc.rx_uuid.uuid) {

                if (debug(DBG_BT))
                    DEBUG_PRINTF("Found RX characteristic\n");

                // Found the characteristic. Store CCCD handle and break.
                m_btc.cccd_handle =
                    p_evt->params.discovered_db.charateristics[i].cccd_handle;
                m_btc.rx_handle      =
                    p_evt->params.discovered_db.charateristics[i].characteristic.handle_value;

                // Send the complete notification
                btc_evt_t evt;
                evt.evt_type = BTC_EVT_DISCOVERY_COMPLETE;
                m_btc.evt_handler(&m_btc, &evt);

                break;
            }
        }
    }
#endif

}

// Dispatch a SoftDevice event to the appropriate event handler
void ble_evt_dispatch(ble_evt_t *p_ble_evt) {

    // The connection handle and role should really be retrievable for any event type.
    uint16_t conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
    uint16_t role = ble_conn_state_role(conn_handle);

    // Debug code for maximum tracing of event flow
#ifdef BLEDEBUG
    if (p_ble_evt->header.evt_id != BLE_GAP_EVT_ADV_REPORT && debug(DBG_BT)) {
        char *event_name = get_ble_event_name(p_ble_evt->header.evt_id);
        if (event_name == NULL)
            DEBUG_PRINTF("[ ble_evt_dispatch type=0x%04x, hdl=0x%04x role=0x%04x\n", p_ble_evt->header.evt_id, conn_handle, role);
        else
            DEBUG_PRINTF("[ ble_evt_dispatch %s, hdl=0x%04x role=0x%04x\n", event_name, conn_handle, role);
    }
#endif

    // Make sure that we process DM events, else connections won't be registered with DM.
    // The most significant result of this is that DFU won't work; it won't leave an "Enabling..." state.
    // because the DM needs to be looked up.  Call stack:
    // dfu_app_on_dfu_evt(), when evt_type == BLE_DFU_START
    //  bootloader_start(p_dfu->conn_handle), which is valid
    //   dfu_app_peer_data_set(conn_handle), which is supposed to set peer_data
    //    dm_handle_get(conn_handle, &m_dm_handle), ...
    //      looks up handle in connection table associated with dm_
    // ...so where does it get put INTO this connection table?
    //  dm_ble_evt_handler(p_ble_evt) is supposed to be called by your own ble_evt_dispatch
    //   BLE_GAP_EVT_CONNECTED adds it.
    // The bug was that I failed to have this call to dm_ble_evt_handler() in my ble_evt_dispatch.
#ifdef OLD_DFU
    dm_ble_evt_handler(p_ble_evt);
#endif

    // Connection state module
    // Remember to call ble_conn_state_on_ble_evt before calling any ble_conns_state_* functions.
    ble_conn_state_on_ble_evt(p_ble_evt);

    // Peer manager
#ifndef NOBONDING
    pm_on_ble_evt(p_ble_evt);
#endif

    // DFU service
#ifdef DFU
    ble_dfu_on_ble_evt(&m_dfus, p_ble_evt);
#endif

    // Handle advertising
    if (p_ble_evt->header.evt_id == BLE_GAP_EVT_TIMEOUT) {
        m_advertising = false;
        if (!io_optimize_power()) {
            if (debug(DBG_BT))
                DEBUG_PRINTF("Advertising timeout - restarting\n");
            ble_advertising_start(APP_INITIAL_ADV_MODE);
            m_advertising = true;
        }
    } else
        ble_advertising_on_ble_evt(p_ble_evt);

    // Based on the role this device plays in the connection, dispatch to the right applications.

#ifndef NOBTC
    if (role == BLE_GAP_ROLE_CENTRAL || role == BLE_GAP_ROLE_INVALID) {

        // on_ble_central_evt will update the connection handles, so we want to execute it
        // after dispatching to the central applications upon disconnection.
        if (p_ble_evt->header.evt_id != BLE_GAP_EVT_DISCONNECTED) {
            on_ble_central_evt(p_ble_evt);
        }

        if (conn_handle == btc_conn_handle) {
            btc_on_ble_evt(&m_btc, p_ble_evt);
            ble_db_discovery_on_ble_evt(&m_ble_db_discovery_btc, p_ble_evt);
        }

        // If the peer disconnected, we update the connection handles last.
        if (p_ble_evt->header.evt_id == BLE_GAP_EVT_DISCONNECTED) {
            on_ble_central_evt(p_ble_evt);
        }
    }
#endif

    if (role == BLE_GAP_ROLE_PERIPH || role == BLE_GAP_ROLE_INVALID) {
        on_ble_peripheral_evt(p_ble_evt);
        ble_conn_params_on_ble_evt(p_ble_evt);
        // This check is necessary because when the central connects, it has
        // BLE_GAP_ROLE_INVALID, and the event handler is confused into thinking
        // that this is a new peripheral connection.
#ifndef NOBTC
        if (conn_handle != btc_conn_handle)
            btp_on_ble_evt(&m_btp, p_ble_evt);
#else
        btp_on_ble_evt(&m_btp, p_ble_evt);
#endif
    }

    // Observe entry/exit pairings
#ifdef BLEDEBUG
    if (p_ble_evt->header.evt_id != BLE_GAP_EVT_ADV_REPORT && debug(DBG_BT)) {
        char *event_name = get_ble_event_name(p_ble_evt->header.evt_id);
        if (event_name == NULL)
            DEBUG_PRINTF("] ble_evt_dispatch type=0x%04x DONE\n", p_ble_evt->header.evt_id);
        else
            DEBUG_PRINTF("] ble_evt_dispatch %s DONE\n", event_name);
    }
#endif

}

// Function for dispatching a system event to interested modules.
void sys_evt_dispatch(uint32_t sys_evt) {
    ble_advertising_on_sys_evt(sys_evt);
    storage_sys_event_handler(sys_evt);
}

// SoftDevice & BLE event initialization
void bluetooth_softdevice_init(void) {
    uint32_t err_code;

#ifdef NSDKV10

    // Initialize SoftDevice.
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, NULL);

    // Enable BLE stack.
    ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));
#if (defined(S130) || defined(S132))
    ble_enable_params.gatts_enable_params.attr_tab_size   = BLE_GATTS_ATTR_TAB_SIZE_DEFAULT;
#endif
    ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
#ifdef S120
    ble_enable_params.gap_enable_params.role              = BLE_GAP_ROLE_CENTRAL;
#endif

    err_code = sd_ble_enable(&ble_enable_params);
    DEBUG_CHECK(err_code);

#else

    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);

    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    DEBUG_CHECK(err_code);

    // Make sure that this has room for all the UUIDs we need to register
#ifdef DFU
    ble_enable_params.common_enable_params.vs_uuid_count = 4;   // ble_dfu_init() needs one
#else
    ble_enable_params.common_enable_params.vs_uuid_count = 3;
#endif

    // Add the service changed attribute
    ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;

    // Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT, PERIPHERAL_LINK_COUNT);

    // Starting in SDK v12, support max MTU
#if (NRF_SD_BLE_API_VERSION >= 3)
    ble_enable_params.gatt_enable_params.att_mtu = NRF_BLE_MAX_MTU_SIZE;
#endif

    // Enable BLE stack.
    err_code = softdevice_enable(&ble_enable_params);
    if (err_code == NRF_ERROR_NO_MEM) {
        // This happens when ble_enable_params.common_enable_params.vs_uuid_count above isn't
        // sufficient - such as when using DFU, because DFU needs to add another attribute.
        // You will need to update the count above, but then you will need to also
        // adjust the RAM size (addr/len) in the .LD file to take account of the larger
        // GATT table.
        // 1) increase the number for vs_uuid_count until this error goes away
        // 2) in the makefile, turn enable -DDEBUG_USES_UART and -DNRF_LOG_USES_RTT
        // 3) do a make clean / make flash to burn a new image
        // 4) boot the device with a serial console attached to the uart
        // 5) ignore the RAN START/SIZE messages, but pay close attention to messages from sd_ble_enable - specifically0
        //        sd_ble_enable: app_ram_base should be adjusted to 0x20002610
        //        ram size should be adjusted to 0x59f0
        // 6) use those values to edit the .ld file used by the linker, i.e.
        //        RAM (rwx) :  ORIGIN = 0x20002610, LENGTH = 0x59f0
        // 7) do a make clean / make flash to burn an image, and you should now be fine
        if (debug(DBG_BT))
            DEBUG_PRINTF("** Insufficient space in GATT table to add required attrib ***\n");
    }
    DEBUG_CHECK(err_code);

#endif // NSDKV10

    // Subscribe for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    DEBUG_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    DEBUG_CHECK(err_code);

}


// Peer Manager initialization.
#ifndef NOBONDING
void peer_manager_init(bool erase_bonds) {
    ble_gap_sec_params_t sec_param;
    ret_code_t err_code;

    err_code = pm_init();
    DEBUG_CHECK(err_code);

    if (erase_bonds) {
        err_code = pm_peers_delete();
        DEBUG_CHECK(err_code);
    }

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    // Security parameters to be used for all security procedures.
    sec_param.bond              = SEC_PARAM_BOND;
    sec_param.mitm              = SEC_PARAM_MITM;
    sec_param.lesc              = SEC_PARAM_LESC;
    sec_param.keypress          = SEC_PARAM_KEYPRESS;
    sec_param.io_caps           = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob               = SEC_PARAM_OOB;
    sec_param.min_key_size      = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size      = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc     = 1;
    sec_param.kdist_own.id      = 1;
    sec_param.kdist_peer.enc    = 1;
    sec_param.kdist_peer.id     = 1;

    err_code = pm_sec_params_set(&sec_param);
    DEBUG_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    DEBUG_CHECK(err_code);

}
#endif // NOBONDING

// Function to manually kill bluetooth, if we're trying to save power
void drop_bluetooth() {
    if (!bluetooth_dropped) {
        if (m_btp.conn_handle != BLE_CONN_HANDLE_INVALID) {
            sd_ble_gap_disconnect(m_btp.conn_handle, BLE_HCI_LOCAL_HOST_TERMINATED_CONNECTION);
            m_btp.conn_handle = BLE_CONN_HANDLE_INVALID;
            m_btp.is_notification_enabled = false;
        }
        if (btp_conn_handle != BLE_CONN_HANDLE_INVALID) {
            sd_ble_gap_disconnect(btp_conn_handle, BLE_HCI_LOCAL_HOST_TERMINATED_CONNECTION);
            btp_conn_handle = BLE_CONN_HANDLE_INVALID;
        }
#ifndef NOBTC
        if (m_btc.conn_handle != BLE_CONN_HANDLE_INVALID) {
            sd_ble_gap_disconnect(m_btc.conn_handle, BLE_HCI_LOCAL_HOST_TERMINATED_CONNECTION);
            m_btc.conn_handle = BLE_CONN_HANDLE_INVALID;
        }
        if (btc_conn_handle != BLE_CONN_HANDLE_INVALID) {
            sd_ble_gap_disconnect(btc_conn_handle, BLE_HCI_LOCAL_HOST_TERMINATED_CONNECTION);
            btc_conn_handle = BLE_CONN_HANDLE_INVALID;
        }
        btc_scan_stop();
#endif
        advertising_stop();
        ble_conn_params_stop();
        softdevice_ble_evt_handler_set(NULL);
        softdevice_sys_evt_handler_set(NULL);
        bluetooth_dropped = true;
    }
}

// Function to check to see if we CAN send to bluetooth
bool can_send_to_bluetooth() {
    if (btp_conn_handle == BLE_CONN_HANDLE_INVALID || m_btp.conn_handle == BLE_CONN_HANDLE_INVALID)
        return false;
    return (btp_can_send(&m_btp));
}

// Function to check to see if we CAN send to bluetooth
uint32_t bluetooth_session_id() {
    return(current_bluetooth_session_id);
}

// Transmit to the BT controller device
// This function will receive a single character from the caller, and append it to
// a string. The string will be be sent over BLE when the last character received was a
// 'new line' i.e '\n' (hex 0x0D) or if the string has reached a length of BTP_MAX_DATA_LENGTH.
// The function returns TRUE if the caller can just keep sending without worrying about
// any form of pauses, or FALSE if we must pause before trying to send the next byte.
// In no case is there any data loss.
bool send_byte_to_bluetooth(uint8_t databyte) {
    static uint8_t data_array[BTP_MAX_DATA_LEN];
    static uint8_t index = 0;

    // If we can't send, don't even try.  Just swallow the character.
    if (!can_send_to_bluetooth())
        return true;

    // Echo whatever is sent to BT on the serial port where we're debugging
#ifdef DEBUG_USES_UART
    serial_send_byte(databyte);
#endif

    // Append the data byte
    if (index < BTP_MAX_DATA_LEN)
        data_array[index++] = databyte;

    if (databyte == '\n' || index >= BTP_MAX_DATA_LEN) {

        // Transmit one packet to the host
        btp_string_send(&m_btp, data_array, index);

        // Reset the buffer
        index = 0;

        // Exit, noting that the caller *must* now pause
        return false;

    }

    // The caller is allowed to come back quickly.
    return true;

}

// Load application-specific context after establishing a secure connection.
#ifdef OLD_DFU
void app_context_load(dm_handle_t const *p_handle) {

    static uint32_t          context_data;
    dm_application_context_t context;
    uint32_t                 err_code;

    context.len    = sizeof(context_data);
    context.p_data = (uint8_t *)&context_data;

    err_code = dm_application_context_get(p_handle, &context);
    if (err_code == NRF_SUCCESS) {
        // Send Service Changed Indication if ATT table has changed.
        if ((context_data & (DFU_APP_ATT_TABLE_CHANGED << DFU_APP_ATT_TABLE_POS)) != 0) {
            err_code = sd_ble_gatts_service_changed(btp_conn_handle, APP_SERVICE_HANDLE_START, BLE_HANDLE_MAX);
            if ((err_code != NRF_SUCCESS) &&
                (err_code != BLE_ERROR_INVALID_CONN_HANDLE) &&
                (err_code != NRF_ERROR_INVALID_STATE) &&
                (err_code != NRF_ERROR_BUSY) &&
                (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)) {
                DEBUG_CHECK(err_code);
            }
        }

        err_code = dm_application_context_delete(p_handle);
        DEBUG_CHECK(err_code);
    } else if (err_code == DM_NO_APP_CONTEXT) {
        // No context available. Ignore.
    } else {
        DEBUG_CHECK(err_code);
    }
}
#endif

// Function for handling the Device Manager events.
#ifdef OLD_DFU
ret_code_t device_manager_evt_handler(dm_handle_t const *p_handle,
                                      dm_event_t const *p_event,
                                      ret_code_t event_result) {

    if (p_event->event_id == DM_EVT_LINK_SECURED) {
        app_context_load(p_handle);
    }

    return NRF_SUCCESS;
}
#endif
