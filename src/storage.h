// Storage subsystem

#ifndef STOR_H__
#define STOR_H__

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

// We store one fixed block of this size.  Note that this must be a multiple of 4
// which is PHY_WORD_SIZE.  (There is no downside to increasing this; for a variety
// of reasons I just wanted to make very clear how large the block size it is that
// we're occupying in Flash, so I wrote this code to explicitly write only that
// fixed block size.)
#define TTSTORAGE_MAX 512

// This structure must never exceed the above size
union ttstorage_ {

    uint8_t data[TTSTORAGE_MAX];

    struct storage_ {

// If a valid signature, we then check version #
#define VALID_SIGNATURE 0xAAB8C7D9L

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

#define PRODUCT_SIMPLECAST      0
                uint16_t product;

// Keep Bluetooth alive
#define FLAG_BTKEEPALIVE        0x00000001
// Keep a listen outstanding and relay Safecast messages
#define FLAG_RELAY              0x00000002
// Continuously ping ttserve
#define FLAG_PING               0x00000004
// Keep a listen outstanding for text messages
#define FLAG_LISTEN             0x00000008
                uint32_t flags;

// Sensors
#define SENSOR_AIR_COUNTS       0x00000001
#define SENSOR_GPIO_GEIGER0     0x00000002
#define SENSOR_GPIO_GEIGER1     0x00000004
#define SENSOR_TWI_MAX17043     0x00000008
#define SENSOR_TWI_HIH6130      0x00000010
#define SENSOR_TWI_UBLOXM8      0x00000020
#define SENSOR_TWI_BME280       0x00000040
#define SENSOR_TWI_INA219       0x00000080
#define SENSOR_TWI_LIS3DH       0x00000100
#define SENSOR_UART_PMS         0x00000200
#define SENSOR_UART_UGPS        0x00000400
#define SENSOR_SPI_OPC          0x00000800
#define SENSOR_ALL 0                            \
                | SENSOR_GPIO_GEIGER0           \
                | SENSOR_GPIO_GEIGER1           \
                | SENSOR_TWI_MAX17043           \
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

// Oneshot-mode interval
                uint16_t oneshot_minutes;
                uint16_t oneshot_cell_minutes;

// Reboot-days interval
                uint16_t restart_days;

// WAN_ configuration
                uint8_t wan;

// Communications region for LPWAN
                char lpwan_region[8];

// Service upload params
                char carrier_apn[32];
                char service_addr[32];
                uint16_t service_udp_port;
                uint16_t service_http_port;

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

            } v1;

        } versions;

        uint32_t signature_bottom;

    } storage;

} ttstorage;

// Exports
STORAGE *storage();
void storage_init();
void storage_save();
bool storage_load();
void storage_set_to_default();
void storage_sys_event_handler(uint32_t sys_evt);
bool storage_get_device_params_as_string(char *buffer, uint16_t length);
char *storage_get_device_params_as_string_help();
void storage_set_device_params_as_string(char *str);
bool storage_get_service_params_as_string(char *buffer, uint16_t length);
char *storage_get_service_params_as_string_help();
void storage_set_service_params_as_string(char *str);
bool storage_get_dfu_state_as_string(char *buffer, uint16_t length);
char *storage_get_dfu_state_as_string_help();
void storage_set_dfu_state_as_string(char *str);
bool storage_get_gps_params_as_string(char *buffer, uint16_t length);
char *storage_get_gps_params_as_string_help();
void storage_set_gps_params_as_string(char *str);
bool storage_get_sensor_params_as_string(char *buffer, uint16_t length);
char *storage_get_sensor_params_as_string_help();
void storage_set_sensor_params_as_string(char *str);

#endif
