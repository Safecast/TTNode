// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

// Flash storage support

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "debug.h"
#include "comm.h"
#include "timer.h"
#include "nrf_delay.h"
#include "app_error.h"
#include "config.h"
#include "storage.h"
#include "softdevice_handler.h"

#define DEBUGSTORAGE false

#ifdef OLDSTORAGE
#include "pstorage.h"
#endif
#if !defined(NOBONDING) || !defined(OLDSTORAGE)
#include "fstorage.h"
#endif

#if !defined(OLDSTORAGE)

static void tt_fs_event_handler(fs_evt_t const * const evt, fs_ret_t result);
#if DB_ENABLED
static void db_fs_event_handler(fs_evt_t const * const evt, fs_ret_t result);
#endif

// Regardless of what it says in the doc, both priority 0 and priority 255 are reserved.
// Higher priority number is higher address and is given allocation priority.
FS_REGISTER_CFG(fs_config_t tt_fs_config) =
{
    .callback  = tt_fs_event_handler,
    .num_pages = TT_PAGES,
    .priority = 2
};
#if DB_ENABLED
FS_REGISTER_CFG(fs_config_t db_fs_config) =
{
    .callback  = db_fs_event_handler,
    .num_pages = DB_PAGES,
    .priority = 1
};
#endif

// Retrieve the address of a page
const uint32_t * address_of_tt_page(uint16_t page_num) {
    return tt_fs_config.p_start_addr + (page_num * PHY_PAGE_SIZE_WORDS);
}
#if DB_ENABLED
const uint32_t * address_of_db_page(uint16_t page_num) {
    return db_fs_config.p_start_addr + (page_num * PHY_PAGE_SIZE_WORDS);
}
#endif
#endif  // OLDSTORAGE

// Storage context
static bool storage_initialized = false;
static bool storage_save_pending = false;
static union ttstorage_ tt;

// Persistent Storage context
#ifdef OLDSTORAGE
static bool pstorage_waiting = false;
static pstorage_block_t pstorage_wait_handle = 0;
static uint16_t pstorage_wait_result;
static pstorage_handle_t pstorage_handle;
static pstorage_handle_t block_0_handle;
#endif

// Function for system event handling
void storage_sys_event_handler(uint32_t sys_evt) {
#ifdef OLDSTORAGE
    pstorage_sys_event_handler(sys_evt);
#endif
// This is required for peer manager, so dispatch here even if storage is disabled
#if !defined(NOBONDING) || !defined(OLDSTORAGE)
    fs_sys_event_handler(sys_evt);
#endif
}

// Event handler for fstorage
#ifndef OLDSTORAGE
static void tt_fs_event_handler(fs_evt_t const * const evt, fs_ret_t result)
{
    if (result != FS_SUCCESS)
    {
        // An error occurred.
    }
}
#endif
#if DB_ENABLED
static void db_fs_event_handler(fs_evt_t const * const evt, fs_ret_t result)
{
    if (result != FS_SUCCESS)
    {
        // An error occurred.
    }
}
#endif

// Persistent storage callback
#ifdef OLDSTORAGE
void pstorage_callback(pstorage_handle_t  * handle,
                       uint8_t              op_code,
                       uint32_t             result,
                       uint8_t            * p_data,
                       uint32_t             data_len) {

    /// If we were waiting, clear the wait
    if (pstorage_waiting && handle->block_id == pstorage_wait_handle) {
        pstorage_wait_result = result;
        pstorage_waiting = false;
    }

    switch (op_code)
    {

    case PSTORAGE_STORE_OP_CODE:
        if (result == NRF_SUCCESS)
            DEBUG_PRINTF("Flash updated.\n");
        else
            DEBUG_PRINTF("Flash storage save error: %d\n", result);
        break;
    case PSTORAGE_LOAD_OP_CODE:
        if (result == NRF_SUCCESS)
            DEBUG_PRINTF("Flash loaded.\n");
        else
            DEBUG_PRINTF("Flash storage load error: %d\n", result);
        break;
    case PSTORAGE_UPDATE_OP_CODE:
    case PSTORAGE_CLEAR_OP_CODE:
        break;
    }

}
#endif // OLDSTORAGE

// Initialize storage subsystem
void storage_init() {
    bool reinitStorage;
    bool initSuccess;

    // clear the in-memory data structure to default in case of failure
    memset(&tt, 0, sizeof(tt));

#ifdef OLDSTORAGE
    // Overall architecture:
    // - each page is 1024 bytes
    // - You can register for N fixed-size blocks within those pages
    uint32_t err_code;
    err_code = pstorage_init();
    if (err_code != NRF_SUCCESS) {
        DEBUG_CHECK(err_code);
        storage_set_to_default();
        return;
    }

    // In our app we'll use a single fixed-size page of TTSTORAGE_MAX bytes
    pstorage_module_param_t param;
    param.block_size  = TTSTORAGE_MAX;
    param.block_count = 1;
    param.cb          = pstorage_callback;
    err_code = pstorage_register(&param, &pstorage_handle);
    if (err_code != NRF_SUCCESS) {
        DEBUG_CHECK(err_code);
        storage_set_to_default();
        return;
    }
    // Get block identifier, by block #.
    // Since we only have a single block, it's #0
    pstorage_block_identifier_get(&pstorage_handle, 0, &block_0_handle);
#else
    fs_ret_t err_code = fs_init();
    if (err_code != FS_SUCCESS) {
        DEBUG_CHECK(err_code);
        storage_set_to_default();
        return;
    }
#endif

    // We've successfully initialized
    storage_initialized = true;

    // Load it
    initSuccess = storage_load();

    // Determine whether or not what we've read is valid
    reinitStorage = true;
    if (initSuccess && tt.storage.signature_top == VALID_SIGNATURE && tt.storage.signature_bottom == VALID_SIGNATURE)
        if (tt.storage.version >= MIN_SUPPORTED_VERSION && tt.storage.version <= MAX_SUPPORTED_VERSION) {
            DEBUG_PRINTF("Loaded valid params from storage\n");
            reinitStorage = false;
        }

    // Reinitialize storage if we must
    if (reinitStorage) {
        // Initialize the in-memory structure
        storage_set_to_default();
        // Write it, because we always keep the latest copy on-disk
        storage_save(true);
        // Wait a few seconds, just to make sure that when we boot
        // we have a stable state in NVRAM for subsequent boots
        nrf_delay_ms(3000);
    }

}

// Let others get access to the storage
STORAGE *storage() {
    return(&tt.storage.versions.v1);
}

// Set the in-memory structures to default values
void storage_set_to_default() {

    DEBUG_PRINTF("Setting storage to default values\n");

    // Set the storage signature
    tt.storage.signature_top = tt.storage.signature_bottom = VALID_SIGNATURE;

    // We're operating on v1 as the most current version
    tt.storage.version = 1;

    // Initialize ALL fields of our in-memory structure to the current version
#ifdef STORAGE_PRODUCT
    tt.storage.versions.v1.product = STORAGE_PRODUCT;
#else
    tt.storage.versions.v1.product = PRODUCT_SOLARCAST;
#endif
#ifdef STORAGE_SENSORS
    tt.storage.versions.v1.sensors = STORAGE_SENSORS;
#else
    tt.storage.versions.v1.sensors = SENSOR_ALL;
#endif
#ifdef AIR_COUNTS
    tt.storage.versions.v1.sensors |= SENSOR_AIR_COUNTS;
#endif
#ifdef STORAGE_FLAGS
    tt.storage.versions.v1.flags = STORAGE_FLAGS;
#else
    tt.storage.versions.v1.flags = 0x00000000L;
#endif
#ifdef SSD_UPSIDE_DOWN
    tt.storage.versions.v1.flags |= FLAG_FLIP;
#endif
#ifdef TESTDEVICE
    tt.storage.versions.v1.flags |= FLAG_TEST;
#endif
    
    tt.storage.versions.v1.restart_days = DEFAULT_RESTART_DAYS;
    tt.storage.versions.v1.uptime_days = 0;

#ifdef STORAGE_WAN
    tt.storage.versions.v1.wan = STORAGE_WAN;
#else
#ifdef CELLX
#ifdef LORA
    tt.storage.versions.v1.wan = WAN_AUTO;
#else
    tt.storage.versions.v1.wan = WAN_FONA;
#endif
#else
#ifdef LORA
    tt.storage.versions.v1.wan = WAN_LORA_THEN_LORAWAN;
#else
    tt.storage.versions.v1.wan = WAN_NONE;
#endif
#endif
#endif

    // Oneshot stuff
#ifdef STORAGE_ONESHOT
    tt.storage.versions.v1.oneshot_minutes = STORAGE_ONESHOT;
#else
    // If UART Select is defined, default to oneshot
#if defined(CELLX)
    tt.storage.versions.v1.oneshot_minutes = ONESHOT_MINUTES;
#else
    // No oneshot
    tt.storage.versions.v1.oneshot_minutes = 0;
#endif
#endif

    // Default cell upload
    tt.storage.versions.v1.oneshot_cell_minutes = ONESHOT_CELL_UPLOAD_MINUTES;

    // Default stats minutes
#ifdef HOURLYSTATS
    tt.storage.versions.v1.stats_minutes = 60;
#else
    tt.storage.versions.v1.stats_minutes = SERVICE_UPDATE_MINUTES;
#endif

    // Device ID
    tt.storage.versions.v1.device_id = 0L;

    // Device label
#if defined(LABEL)
    strcpy(tt.storage.versions.v1.device_label, STRINGIZE_VALUE_OF(LABEL));
#elif defined(STORAGE_LABEL)
    strcpy(tt.storage.versions.v1.device_label, STRINGIZE_VALUE_OF(STORAGE_LABEL));
#else
    tt.storage.versions.v1.device_label[0] = '\0'
#endif

        // Initialize things that allow us to contact the service
#ifdef STORAGE_REGION
        strcpy(tt.storage.versions.v1.lpwan_region, STRINGIZE_VALUE_OF(STORAGE_REGION));
#else
    strcpy(tt.storage.versions.v1.lpwan_region, "");
#endif
#ifdef STORAGE_APN
    strcpy(tt.storage.versions.v1.carrier_apn, STRINGIZE_VALUE_OF(STORAGE_APN));
#else
    strcpy(tt.storage.versions.v1.carrier_apn, WIRELESS_CARRIER_APN);
#endif

    // Initialize TTN parameters
    strcpy(tt.storage.versions.v1.ttn_dev_eui, "");
    strcpy(tt.storage.versions.v1.ttn_app_eui, TTN_TTSERVE_APP_EUI);
    strcpy(tt.storage.versions.v1.ttn_app_key, TTN_TTSERVE_APP_KEY);

    // Initialize fixed gps coordinages, noting that 0.0 means "not assigned"
#if defined(NOGPS) // Use when you don't want the GPS to seek
    tt.storage.versions.v1.gps_latitude = 1;
    tt.storage.versions.v1.gps_longitude = 1;
    tt.storage.versions.v1.gps_altitude = 1;
#elif defined(ROCKSGPS) // For Ray's use
    tt.storage.versions.v1.gps_latitude = 42.565;
    tt.storage.versions.v1.gps_longitude = -70.784;
    tt.storage.versions.v1.gps_altitude = 0;
#else
    tt.storage.versions.v1.gps_latitude = 0.0;
    tt.storage.versions.v1.gps_longitude = 0.0;
    tt.storage.versions.v1.gps_altitude = 0.0;
#endif
    tt.storage.versions.v1.lkg_gps_latitude = 0.0;
    tt.storage.versions.v1.lkg_gps_longitude = 0.0;
    tt.storage.versions.v1.lkg_gps_altitude = 0.0;

    // Initialize sensor params
    tt.storage.versions.v1.sensor_params[0] = '\0';

    // Initialize expected firmware build filename
#ifdef STORAGE_DFU_FIRMWARE
    strcpy(tt.storage.versions.v1.dfu_filename, STRINGIZE_VALUE_OF(STORAGE_DFU_FIRMWARE));
#else
    strcpy(tt.storage.versions.v1.dfu_filename, "");
#endif
    tt.storage.versions.v1.dfu_status = DFU_IDLE;
    tt.storage.versions.v1.dfu_error = DFU_ERR_NONE;
    tt.storage.versions.v1.dfu_count = 0;

    // Initialize data buffers
#if DB_ENABLED
    tt.storage.versions.v1.db_filled = 0;
    tt.storage.versions.v1.db_next_to_fill = 0;
    tt.storage.versions.v1.db_next_to_upload = 0;
    memset(&tt.storage.versions.v1.db_length, 0, sizeof(tt.storage.versions.v1.db_length));
    memset(&tt.storage.versions.v1.db_request_type, 0, sizeof(tt.storage.versions.v1.db_request_type));
#endif
    
}

// Get a static help string indicating how the as_string stuff works
char *storage_get_device_params_as_string_help() {
    return("wan.prod.flags.1shotMin.1shotBuffMin.StatsMin.bootDays.sensors.deviceID");
}

// Get the in-memory structures as a deterministic sequential text string
bool storage_get_device_params_as_string(char *buffer, uint16_t length) {
    char buf[100];
    sprintf(buf, "%u.%u.%lu.%u.%u.%u.%u.%lu.%lu",
            tt.storage.versions.v1.wan,
            tt.storage.versions.v1.product,
            tt.storage.versions.v1.flags,
            tt.storage.versions.v1.oneshot_minutes,
            tt.storage.versions.v1.oneshot_cell_minutes,
            tt.storage.versions.v1.stats_minutes,
            tt.storage.versions.v1.restart_days,
            tt.storage.versions.v1.sensors,
            tt.storage.versions.v1.device_id);
    if (buffer != NULL)
        strlcpy(buffer, buf, length);
    return true;
}

// Set the storage params from a text string
void storage_set_device_params_as_string(char *str) {
    long int l;

    // Check for special case shortcuts
    if (strcmp(str, "auto") == 0) {
        tt.storage.versions.v1.wan = (uint8_t) WAN_AUTO;
        return;
    }
    if (strcmp(str, "lora") == 0) {
        tt.storage.versions.v1.wan = (uint8_t) WAN_LORA;
        return;
    }
    if (strcmp(str, "lorawan") == 0 || strcmp(str, "ttn") == 0) {
        tt.storage.versions.v1.wan = (uint8_t) WAN_LORAWAN;
        return;
    }
    if (strcmp(str, "fona") == 0 || strcmp(str, "cell") == 0) {
        tt.storage.versions.v1.wan = (uint8_t) WAN_FONA;
        return;
    }
    if (strcmp(str, "mobile") == 0) {
        tt.storage.versions.v1.wan = (uint8_t) WAN_FONA_PLUS_MOBILE;
        return;
    }
        
    // Set it
    if (*str == '\0')
        return;

    l = strtol(str, &str, 0);
    tt.storage.versions.v1.wan = (uint8_t) l;
    if (*str++ == '\0')
        return;

    l = strtol(str, &str, 0);
    tt.storage.versions.v1.product = (uint16_t) l;
    if (*str++ == '\0')
        return;

    l = strtol(str, &str, 0);
    tt.storage.versions.v1.flags = (uint32_t) l;
    if (*str++ == '\0')
        return;

    l = strtol(str, &str, 0);
    tt.storage.versions.v1.oneshot_minutes = (uint16_t) l;
    if (*str++ == '\0')
        return;

    l = strtol(str, &str, 0);
    tt.storage.versions.v1.oneshot_cell_minutes = (uint16_t) l;
    if (*str++ == '\0')
        return;

    l = strtol(str, &str, 0);
    tt.storage.versions.v1.stats_minutes = (uint16_t) l;
    if (*str++ == '\0')
        return;

    l = strtol(str, &str, 0);
    tt.storage.versions.v1.restart_days = (uint16_t) l;
    if (*str++ == '\0')
        return;

    l = strtol(str, &str, 0);
    tt.storage.versions.v1.sensors = (uint32_t) l;
    if (*str++ == '\0')
        return;

    l = strtol(str, &str, 0);
    tt.storage.versions.v1.device_id = (uint32_t) l;
    if (*str++ == '\0')
        return;

}

// Get a static help string indicating how the as_string stuff works
char *storage_get_device_label_as_string_help() {
    return("label");
}

// Get the in-memory structures as a deterministic sequential text string
bool storage_get_device_label_as_string(char *buffer, uint16_t length) {
    char buf[100];
    if (tt.storage.versions.v1.device_label[0] == '\0') {
        if (buffer != NULL)
            buffer[0] = '\0';
        return false;
    }
    sprintf(buf, "%s",
            tt.storage.versions.v1.device_label);
    if (buffer != NULL)
        strlcpy(buffer, buf, length);
    return true;
}

// Set the storage params from a text string
void storage_set_device_label_as_string(char *str) {
    char ch;
    int i;

    if (*str == '\0')
        return;

    for (i=0;;) {
        ch = *str++;
        if (ch != '/')
            tt.storage.versions.v1.device_label[i++] = ch;
        tt.storage.versions.v1.device_label[i] = '\0';
        if (ch == '\0')
            return;
        if (ch == '/')
            break;
    }
    if (*str == '\0')
        return;

}

// Get a static help string indicating how the as_string stuff works
char *storage_get_service_params_as_string_help() {
    return("region/apn");
}

// Get the in-memory structures as a deterministic sequential text string
bool storage_get_service_params_as_string(char *buffer, uint16_t length) {
    char buf[100];
    if (tt.storage.versions.v1.lpwan_region[0] == '\0' && tt.storage.versions.v1.carrier_apn[0] == '\0') {
        if (buffer != NULL)
            buffer[0] = '\0';
        return false;
    }
    sprintf(buf, "%s/%s",
            tt.storage.versions.v1.lpwan_region,
            tt.storage.versions.v1.carrier_apn);
    if (buffer != NULL)
        strlcpy(buffer, buf, length);
    return true;
}

// Set the storage params from a text string
void storage_set_service_params_as_string(char *str) {
    char ch;
    int i;

    if (*str == '\0')
        return;

    for (i=0;;) {
        ch = *str++;
        if (ch != '/')
            tt.storage.versions.v1.lpwan_region[i++] = ch;
        tt.storage.versions.v1.lpwan_region[i] = '\0';
        if (ch == '\0')
            return;
        if (ch == '/')
            break;
    }
    if (*str == '\0')
        return;

    for (i=0;;) {
        ch = *str++;
        if (ch != '/')
            tt.storage.versions.v1.carrier_apn[i++] = ch;
        tt.storage.versions.v1.carrier_apn[i] = '\0';
        if (ch == '\0')
            return;
        if (ch == '/')
            break;
    }
    if (*str == '\0')
        return;

}

// Get a static help string indicating how the as_string stuff works
char *storage_get_ttn_params_as_string_help() {
    return("appeui/appkey");
}

// Get the in-memory structures as a deterministic sequential text string
bool storage_get_ttn_params_as_string(char *buffer, uint16_t length) {
    char buf[100];
    if (tt.storage.versions.v1.ttn_app_eui[0] == '\0' && tt.storage.versions.v1.ttn_app_key[0] == '\0') {
        if (buffer != NULL)
            buffer[0] = '\0';
        return false;
    }
    // Note that we obscure the app_key because it's a secret.
    char obscured_begin[5], obscured_end[5];
    strcpy(obscured_begin, "????");
    strcpy(obscured_end, "????");
    int lb = strlen(obscured_begin);
    int le = strlen(obscured_begin);
    if (strlen(tt.storage.versions.v1.ttn_app_key) >= lb+le) {
        memcpy(obscured_begin, tt.storage.versions.v1.ttn_app_key, lb);
        memcpy(obscured_end, &tt.storage.versions.v1.ttn_app_key[strlen(tt.storage.versions.v1.ttn_app_key)-le], le);
    }

    sprintf(buf, "%s/%s..%s", tt.storage.versions.v1.ttn_app_eui, obscured_begin, obscured_end);
    if (buffer != NULL)
        strlcpy(buffer, buf, length);
    return true;
}

// Set the storage params from a text string
void storage_set_ttn_params_as_string(char *str) {
    char ch;
    int i;

    if (*str == '\0')
        return;

    for (i=0;;) {
        ch = *str++;
        if (ch != '/')
            tt.storage.versions.v1.ttn_app_eui[i++] = ch;
        tt.storage.versions.v1.ttn_app_eui[i] = '\0';
        if (ch == '\0')
            return;
        if (ch == '/')
            break;
    }
    if (*str == '\0')
        return;

    for (i=0;;) {
        ch = *str++;
        if (ch != '/')
            tt.storage.versions.v1.ttn_app_key[i++] = ch;
        tt.storage.versions.v1.ttn_app_key[i] = '\0';
        if (ch == '\0')
            return;
        if (ch == '/')
            break;
    }
    if (*str == '\0')
        return;

}

// Get a static help string indicating how the as_string stuff works
char *storage_get_dfu_state_as_string_help() {
    return("firmware/count/status/error");
}

// Get the in-memory structures as a deterministic sequential text string
bool storage_get_dfu_state_as_string(char *buffer, uint16_t length) {
    char buf[100];
    sprintf(buf, "%s/%d/%d/%d",
            tt.storage.versions.v1.dfu_filename,
            tt.storage.versions.v1.dfu_count,
            tt.storage.versions.v1.dfu_status,
            tt.storage.versions.v1.dfu_error);
    if (buffer != NULL)
        strlcpy(buffer, buf, length);
    return true;
}

// Set the storage params from a text string
void storage_set_dfu_state_as_string(char *str) {
    char ch;
    int i;
    long int l;

    if (*str == '\0')
        return;

    for (i=0;;) {
        ch = *str++;
        if (ch != '/')
            tt.storage.versions.v1.dfu_filename[i++] = ch;
        tt.storage.versions.v1.dfu_filename[i] = '\0';
        if (ch == '\0')
            return;
        if (ch == '/')
            break;
    }
    if (*str == '\0')
        return;

    l = strtol(str, &str, 0);
    tt.storage.versions.v1.dfu_count = (uint16_t) l;
    if (*str++ == '\0')
        return;

    l = strtol(str, &str, 0);
    tt.storage.versions.v1.dfu_status = (uint16_t) l;
    if (*str++ == '\0')
        return;

    l = strtol(str, &str, 0);
    tt.storage.versions.v1.dfu_error = (uint16_t) l;
    if (*str++ == '\0')
        return;

}

// Get a static help string indicating how the as_string stuff works
char *storage_get_gps_params_as_string_help() {
    return("lat/lon/alt");
}

// Get the in-memory structures as a deterministic sequential text string
bool storage_get_gps_params_as_string(char *buffer, uint16_t length) {
    char buf[100];
    sprintf(buf, "%f/%f/%f",
            tt.storage.versions.v1.gps_latitude,
            tt.storage.versions.v1.gps_longitude,
            tt.storage.versions.v1.gps_altitude);
    if (buffer != NULL)
        strlcpy(buffer, buf, length);
    if (tt.storage.versions.v1.gps_latitude == 0.0
        && tt.storage.versions.v1.gps_longitude == 0.0
        && tt.storage.versions.v1.gps_altitude == 0.0)
        return false;
    return true;
}

// Set the storage params from a text string
void storage_set_gps_params_as_string(char *str) {
    float f;

    if (*str == '\0')
        return;

    f = strtof(str, &str);
    tt.storage.versions.v1.gps_latitude = f;
    if (*str++ == '\0')
        return;

    f = strtof(str, &str);
    tt.storage.versions.v1.gps_longitude = f;
    if (*str++ == '\0')
        return;

    f = strtof(str, &str);
    tt.storage.versions.v1.gps_altitude = f;
    if (*str++ == '\0')
        return;

}

// Get a static help string indicating how the as_string stuff works
char *storage_get_sensor_params_as_string_help() {
    return("g-air.r=15/g-geigers.r=5");
}

// Get the in-memory structures as a deterministic sequential text string
bool storage_get_sensor_params_as_string(char *buffer, uint16_t length) {
    if (buffer != NULL)
        strlcpy(buffer, tt.storage.versions.v1.sensor_params, length);
    if (buffer[0] == '\0')
        return false;
    return true;
}

// Set the storage params from a text string
void storage_set_sensor_params_as_string(char *str) {
    strlcpy(tt.storage.versions.v1.sensor_params, str, sizeof(tt.storage.versions.v1.sensor_params));
}

// Load from pstorage
bool storage_load() {
    if (storage_initialized) {
#ifdef OLDSTORAGE
        int i;
        pstorage_wait_handle = block_0_handle.block_id;
        pstorage_waiting = true;
        pstorage_load(tt.data, &block_0_handle, TTSTORAGE_MAX, 0);
        for (i=0; i<10 && pstorage_waiting; i++) {
            nrf_delay_ms(500);
        }
        if (!pstorage_waiting && pstorage_wait_result == NRF_SUCCESS)
            return true;
#else
        memcpy(tt.data, (uint8_t *) address_of_tt_page(0), sizeof(tt.data));
        return true;
#endif
    }
    storage_set_to_default();
    return (false);
}

// Save if necessary.  Note that we utilize deferred storage saving and storage checkpointing
// in cases where we're trying to do nvram I/O during serial I/O.  There are issues related to
// IRQ priorities that cause serial to be interrupted during periods of flash erase, and so
// it is best to defer the storage writes unless absolutely necessary to do synchronously.
void storage_checkpoint() {
    if (storage_save_pending)
        storage_save(true);
}

// Save the in-memory storage block
void storage_save(bool fSynchronous) {

    // Allows for deferred storage I/O
    if (fSynchronous)
        storage_save_pending = false;
    else {
        storage_save_pending = true;
        return;
    }

    // Exit if not yet initialized
    if (!storage_initialized)
        return;

    DEBUG_PRINTF("Checkpointing flash.\n");

#ifdef OLDSTORAGE
    pstorage_clear(&block_0_handle, TTSTORAGE_MAX);
    pstorage_store(&block_0_handle, tt.data, TTSTORAGE_MAX, 0);
#else
    uint32_t err_code;
#if DEBUGSTORAGE
    DEBUG_PRINTF("At 0x%08lx, erase %d pages, write %d words\n", address_of_tt_page(0), TT_PAGES, TT_WORDS);
#endif
    err_code = fs_erase(&tt_fs_config, address_of_tt_page(0), TT_PAGES, NULL);
    if (err_code != NRF_SUCCESS)
        DEBUG_PRINTF("Flash storage erase error: 0x%04x\n", err_code);
    err_code = fs_store(&tt_fs_config, address_of_tt_page(0), (uint32_t *) tt.data, TT_WORDS, NULL);
    if (err_code != NRF_SUCCESS)
        DEBUG_PRINTF("Flash storage save error: 0x%04x\n", err_code);

#endif

}

// Peek at the next to be uploaded, returning its length or the buffer itself
uint16_t db_get(uint8_t *buffer, uint16_t *length, uint16_t *request_type) {
#if defined(OLDSTORAGE) || !DB_ENABLED
    return 0;
#else
    STORAGE *st = storage();
    uint8_t *db = (uint8_t *) address_of_db_page(0);
    uint8_t *page = &db[db_offset_of_page(st->db_next_to_upload)];
    uint8_t *entry = &page[page_offset_of_entry(st->db_next_to_upload)];
    if (st->db_filled != 0) {
        if (buffer != NULL) {
            memcpy(buffer, entry, DB_ENTRY_BYTES);
#if DEBUGSTORAGE
            DEBUG_PRINTF("db: retrieved %d-byte buff #%d\n", st->db_length[st->db_next_to_upload], st->db_next_to_upload);
#endif
        }
        if (length != NULL)
            *length = st->db_length[st->db_next_to_upload];
        if (request_type != NULL)
            *request_type = st->db_request_type[st->db_next_to_upload];
    }
    return st->db_filled;
#endif
}

// Peek at the next to be uploaded, returning its length
void db_get_release() {
#if defined(OLDSTORAGE) || !DB_ENABLED
    return;
#else
    STORAGE *st = storage();
    if (st->db_filled != 0) {
        st->db_filled--;
#if DEBUGSTORAGE
        DEBUG_PRINTF("db: released buff #%d (now %d remaining)\n", st->db_next_to_upload, st->db_filled);
#endif
        if (++st->db_next_to_upload >= DB_ENTRIES)
            st->db_next_to_upload = 0;
        storage_save(false);
    }
#endif
}

// Is the db stuff enabled?
bool db_enabled() {
    return DB_ENABLED;
}

// Save these readings, using a policy of preferring OLDER
// readings if we run out of buffering space.
bool db_put(uint8_t *buffer, uint16_t length, uint16_t request_type) {
#if defined(OLDSTORAGE) || !DB_ENABLED
    return false;
#else
    uint32_t err_code;
    STORAGE *st = storage();
    uint16_t db_filled = st->db_filled;
    uint16_t db_next_to_fill = st->db_next_to_fill;

    // Back up and overwrite the most recent if we run out of room, so that
    // if there is a bad event that knocks out communications we save the data
    // that is in closest proximity to the event.
    if (db_filled == DB_ENTRIES) {
        db_filled--;
        if (db_next_to_fill-- == 0)
            db_next_to_fill = DB_ENTRIES-1;
    }

    // Create a page buffer and replace just the entry
    char pagebuf[PHY_PAGE_SIZE_BYTES];
    uint8_t *db = (uint8_t *) address_of_db_page(0);
    uint8_t *page = &db[db_offset_of_page(db_next_to_fill)];

#if DEBUGSTORAGE
    DEBUG_PRINTF("Base 0x%08lx at 0x%08lx, copyoff %d, erase %d pages, write %d words\n", address_of_db_page(0), page, page_offset_of_entry(db_next_to_fill), 1, PHY_PAGE_SIZE_WORDS);
#endif

    memcpy(pagebuf, page, sizeof(pagebuf));
    memcpy(&pagebuf[page_offset_of_entry(db_next_to_fill)], buffer, DB_ENTRY_BYTES);

    // Write it to flash
#if DEBUGSTORAGE
    DEBUG_PRINTF("db: queueing %d-byte buff #%d (now %d in queue)\n", length, db_next_to_fill, db_filled+1);
#endif
    err_code = fs_erase(&db_fs_config, (uint32_t *) page, 1, NULL);
    if (err_code != NRF_SUCCESS) {
        DEBUG_PRINTF("Flash storage erase error: 0x%04x\n", err_code);
        return false;
    }
    err_code = fs_store(&db_fs_config, (uint32_t *) page, (uint32_t *) pagebuf, PHY_PAGE_SIZE_WORDS, NULL);
    if (err_code != NRF_SUCCESS) {
        DEBUG_PRINTF("Flash storage save error: 0x%04x\n", err_code);
        return false;
    }

    // Save length and request type before bumping to the next one
    st->db_length[db_next_to_fill] = length;
    st->db_request_type[db_next_to_fill] = request_type;

    // Now that it's successfully saved, advance the pointers and update them in storage
    db_filled++;
    if (++db_next_to_fill >= DB_ENTRIES)
        db_next_to_fill = 0;
    st->db_filled = db_filled;
    st->db_next_to_fill = db_next_to_fill;
    storage_save(true);
    return true;

#endif
}
