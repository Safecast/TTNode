// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

#ifndef STOR_H__
#define STOR_H__

#if defined(NSDKV10) || defined(NSDKV11)
#define OLDSTORAGE
#endif

// WAN configurations
#define WAN_AUTO                        0   // FonaGPS,
                                            // then Lora-continuous,
                                            // then LoraWAN-continuous,
                                            // then Fona-oneshot
#define WAN_LORA                        1
#define WAN_LORAWAN                     2
#define WAN_FONA                        3
#define WAN_LORA_THEN_LORAWAN           4
#define WAN_LORAWAN_THEN_LORA           5
#define WAN_NONE                        6
#define WAN_FONA_PLUS_MOBILE            7

// A "Data Buffer" for buffering of data sent while comms is unavailable
// This is purely optional, however it is useful.  The rough calculations
// of buffer size are:
// 32-bytes-per sensor reading (approximately)
// 5 seconds between readings, taken in mobile mode
// 1 hour ideal upload interval
// 2 hours worst case upload interval
#ifdef OLDSTORAGE
#define DB_ENABLED      false
#else
#define DB_ENABLED      true
#define DB_MAX_TARGET   12000
#define DB_ENTRY_TARGET 1500
#endif

// We store one fixed block of this size.  Note that this must be a multiple of 4
// which is PHY_WORD_SIZE.  (There is no downside to increasing this; for a variety
// of reasons I just wanted to make very clear how large the block size it is that
// we're occupying in Flash, so I wrote this code to explicitly write only that
// fixed block size.)
#ifdef OLDSTORAGE
#define TTSTORAGE_MAX 512
#else
#define TTSTORAGE_MAX 1024
#endif

// Nordic physical flash constraints
#define PHY_WORD_SIZE         4
#if   defined(NRF51)
#define PHY_PAGE_SIZE_WORDS   (256)
#elif defined(NRF52)
#define PHY_PAGE_SIZE_WORDS   (1024)
#endif
#define PHY_PAGE_SIZE_BYTES   (PHY_PAGE_SIZE_WORDS*PHY_WORD_SIZE)

// Our app's settings
#define TT_PAGES            1
#define TT_WORDS            ((TTSTORAGE_MAX/PHY_WORD_SIZE)+1)
#if (PHY_PAGE_SIZE_WORDS < TT_WORDS)
@error Code is written assuming max of 1 physical page
#endif

#if !DB_ENABLED
// Just to allow code to compile with static buffers that are never used
#define DB_ENTRY_BYTES      10      
#else
#define DB_ENTRY_WORDS      ((DB_ENTRY_TARGET/PHY_WORD_SIZE)+1)
#define DB_ENTRY_BYTES      (DB_ENTRY_WORDS*PHY_WORD_SIZE)
#define DB_PAGES            ((DB_MAX_TARGET/PHY_PAGE_SIZE_BYTES)+1)
#define DB_BYTES            (DB_PAGES*PHY_PAGE_SIZE_BYTES)
#define DB_ENTRIES_PER_PAGE (PHY_PAGE_SIZE_BYTES/DB_ENTRY_BYTES)
#define DB_ENTRIES          (DB_PAGES*DB_ENTRIES_PER_PAGE)
#define page(x)             (x/DB_ENTRIES_PER_PAGE)
#define db_offset_of_page(x) (page(x)*PHY_PAGE_SIZE_BYTES)
#define page_offset_of_entry(x) ((x%DB_ENTRIES_PER_PAGE)*DB_ENTRY_BYTES)
#endif

// This structure must never exceed the above size
union ttstorage_ {

    uint8_t data[TTSTORAGE_MAX];

    struct storage_ {

// If a valid signature, we then check version #
#define VALID_SIGNATURE 0xAAD8C7DAL

        uint32_t signature_top;

// Anything outside this range is treated as uninitialized
#define MIN_SUPPORTED_VERSION 1
#define MAX_SUPPORTED_VERSION 1
#define STORAGE struct v1_
        uint16_t version;

        // Storage structures
        union versions_ {

            // The V1 format
            struct v1_ {

#define PRODUCT_SOLARCAST       0
                uint16_t product;

// Keep Bluetooth alive
#define FLAG_BTKEEPALIVE        0x00000001
// Send buffered updates via the most efficient transport, as opposed to the most reliable
#define FLAG_BUFFERED_EFFICIENT 0x00000002
// Keep a listen outstanding and relay Safecast messages
#define FLAG_RELAY              0x00000004
// Continuously ping ttserve
#define FLAG_PING               0x00000008
// Keep a listen outstanding for text messages
#define FLAG_LISTEN             0x00000010
// Confirm ALL service transactions
#define FLAG_CONFIRM_ALL        0x00000020
// This is a device in test mode
#define FLAG_TEST               0x00000040
// Flip the display upside down
#define FLAG_FLIP               0x00000080
                uint32_t flags;

// Sensors
#define SENSOR_AIR_COUNTS       0x00000001
#define SENSOR_GPIO_GEIGER0     0x00000002
#define SENSOR_GPIO_GEIGER1     0x00000004
#define SENSOR_TWI_MAX17043     0x00000008
#define SENSOR_TWI_MAX17201     0x00000010
#define SENSOR_TWI_HIH6130      0x00000020
#define SENSOR_TWI_UBLOXM8      0x00000040
#define SENSOR_TWI_BME280       0x00000080
#define SENSOR_TWI_INA219       0x00000100
#define SENSOR_TWI_LIS3DH       0x00000200
#define SENSOR_UART_PMS         0x00000400
#define SENSOR_UART_UGPS        0x00000800
#define SENSOR_SPI_OPC          0x00001000
#define SENSOR_ALL 0                            \
                | SENSOR_GPIO_GEIGER0           \
                | SENSOR_GPIO_GEIGER1           \
                | SENSOR_TWI_MAX17043           \
                | SENSOR_TWI_MAX17201           \
                | SENSOR_TWI_HIH6130            \
                | SENSOR_TWI_UBLOXM8            \
                | SENSOR_TWI_BME280             \
                | SENSOR_TWI_INA219             \
                | SENSOR_TWI_LIS3DH             \
                | SENSOR_UART_PMS               \
                | SENSOR_UART_UGPS              \
                | SENSOR_SPI_OPC                \
                | 0
                uint32_t sensors;

// Device ID override
                uint32_t device_id;

// Device label
                char device_label[64];

// Oneshot-mode interval
                uint16_t oneshot_minutes;

// Buffered update interval
                uint16_t oneshot_cell_minutes;

// How often to send stats and poll the service for commands
                uint16_t stats_minutes;

// Reboot-days interval, and count of days each time we restart
                uint16_t restart_days;
                uint16_t uptime_days;

// WAN_ configuration
                uint8_t wan;

// Communications region for LPWAN
                char lpwan_region[8];

// Service upload params
                char carrier_apn[32];
                char service_addr[32];

// TTN params
                char ttn_dev_eui[20];
                char ttn_app_eui[20];
                char ttn_app_key[40];

// GPS params
                float gps_latitude;
                float gps_longitude;
                float gps_altitude;

// Last known good GPS
                float lkg_gps_latitude;
                float lkg_gps_longitude;
                float lkg_gps_altitude;

// Sensor config params
                char sensor_params[100];

// DFU status
#define DFU_IDLE            0
#define DFU_PENDING         1
// Error types
#define DFU_ERR_NONE            0
#define DFU_ERR_BASIC           1
#define DFU_ERR_GETFILE         2
#define DFU_ERR_NO_NETWORK      3
#define DFU_ERR_TRANSFER        4
#define DFU_ERR_RESET           5
#define DFU_ERR_PREPARE         6
                uint16_t dfu_status;
                uint16_t dfu_error;
                uint16_t dfu_count;
                char dfu_filename[40];

// Stored data awaiting upload
#if DB_ENABLED
                uint16_t db_filled;
                uint16_t db_next_to_fill;
                uint16_t db_next_to_upload;
                uint16_t db_length[DB_ENTRIES];
                uint16_t db_request_type[DB_ENTRIES];
#endif

            } v1;

        } versions;

        uint32_t signature_bottom;

    } storage;

} ttstorage;

// Exports
STORAGE *storage();
void storage_init();
void storage_save(bool);
void storage_checkpoint();
bool storage_load();
void storage_set_to_default();
void storage_sys_event_handler(uint32_t sys_evt);
bool storage_get_device_params_as_string(char *buffer, uint16_t length);
char *storage_get_device_params_as_string_help();
void storage_set_device_params_as_string(char *str);
bool storage_get_service_params_as_string(char *buffer, uint16_t length);
char *storage_get_service_params_as_string_help();
void storage_set_service_params_as_string(char *str);
bool storage_get_device_label_as_string(char *buffer, uint16_t length);
char *storage_get_device_label_as_string_help();
void storage_set_device_label_as_string(char *str);
bool storage_get_ttn_params_as_string(char *buffer, uint16_t length);
char *storage_get_ttn_params_as_string_help();
void storage_set_ttn_params_as_string(char *str);
bool storage_get_dfu_state_as_string(char *buffer, uint16_t length);
char *storage_get_dfu_state_as_string_help();
void storage_set_dfu_state_as_string(char *str);
bool storage_get_gps_params_as_string(char *buffer, uint16_t length);
char *storage_get_gps_params_as_string_help();
void storage_set_gps_params_as_string(char *str);
bool storage_get_sensor_params_as_string(char *buffer, uint16_t length);
char *storage_get_sensor_params_as_string_help();
void storage_set_sensor_params_as_string(char *str);

uint16_t db_get(uint8_t *buffer, uint16_t *length, uint16_t *request_type);
void db_get_release();
bool db_put(uint8_t *buffer, uint16_t length, uint16_t request_type);
bool db_enabled();

#endif
