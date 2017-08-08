// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

// Master definitions that drive the sensor sampling scheduler
// Note that the following symbol defines whether or not we artificially
// serialize all sensor measurements with respect to communications.
// This slows things down tremendously, but is good for debugging.
#define DEFAULT_EXCLUSIVE false

#ifdef TWIHIH6130
static sensor_t temphumidity = {
    "s-temp",
    {0},                    // state
    SENSOR_TWI_HIH6130,     // storage_sensor_mask
    0,                      // init_parameter
    NO_HANDLER,             // init_once
    s_hih6130_init,         // init_power
    s_hih6130_term,         // term_power
    NO_HANDLER,             // show_value
    0,                      // poll_repeat_milliseconds
    false,                  // poll_continuously
    false,                  // poll_during_settling
    NO_HANDLER,             // poll_handler
    30,                     // settling_seconds
    NO_HANDLER,             // done_settling
    NO_HANDLER,             // done_group_settling
    s_hih6130_upload_needed,// upload_needed
    s_hih6130_measure,      // measure
};
#endif

#if defined(TWIBME0)
static sensor_t bme0 = {
    "s-bme0",
    {0},                    // state
    SENSOR_TWI_BME280,      // storage_sensor_mask
    0,                      // init_parameter
    NO_HANDLER,             // init_once
    s_bme280_0_init,        // init_power
    s_bme280_0_term,        // term_power
    s_bme280_0_show_value,  // show_value
    0,                      // poll_repeat_milliseconds
    false,                  // poll_continuously
    false,                  // poll_during_settling
    NO_HANDLER,             // poll_handler
    10,                     // settling_seconds
    NO_HANDLER,             // done_settling
    NO_HANDLER,             // done_group_settling
    s_bme280_0_upload_needed, // upload_needed
    s_bme280_0_measure,     // measure
};
#endif

#if defined(TWIBME1)
static sensor_t bme1 = {
    "s-bme1",
    {0},                    // state
    SENSOR_TWI_BME280,      // storage_sensor_mask
    0,                      // init_parameter
    NO_HANDLER,             // init_once
    s_bme280_1_init,        // init_power
    s_bme280_1_term,        // term_power
    NO_HANDLER,             // show_value
    0,                      // poll_repeat_milliseconds
    false,                  // poll_continuously
    false,                  // poll_during_settling
    NO_HANDLER,             // poll_handler
    10,                     // settling_seconds
    NO_HANDLER,             // done_settling
    NO_HANDLER,             // done_group_settling
    s_bme280_1_upload_needed, // upload_needed
    s_bme280_1_measure,     // measure
};
#endif


#ifdef TWIINA219
static sensor_t ina219 = {
    "s-ina",
    {0},                    // state
    SENSOR_TWI_INA219,      // storage_sensor_mask
    0,                      // init_parameter
    NO_HANDLER,             // init_once
    s_ina_init,             // init_power
    s_ina_term,             // term_power
    s_ina_show_value,       // show_value
    (PWR_SAMPLE_SECONDS*1000), // poll_repeat_milliseconds
    false,                  // poll_continuously
    false,                  // poll_during_settling
    s_ina_poll,             // poll_handler
    0,                      // settling_seconds
    NO_HANDLER,             // done_settling
    NO_HANDLER,             // done_group_settling
    s_ina_upload_needed,    // upload_needed
    s_ina_measure,          // measure
};
#endif

#ifdef TWIMAX17201
static sensor_t max01 = {
    "s-max01",
    {0},                    // state
    SENSOR_TWI_MAX17201,    // storage_sensor_mask
    0,                      // init_parameter
    NO_HANDLER,             // init_once
    s_max01_init,           // init_power
    s_max01_term,           // term_power
    s_max01_show_value,     // show_value
    (PWR_SAMPLE_SECONDS*1000), // poll_repeat_milliseconds
    false,                  // poll_continuously
    false,                  // poll_during_settling
    s_max01_poll,           // poll_handler
    0,                      // settling_seconds
    NO_HANDLER,             // done_settling
    NO_HANDLER,             // done_group_settling
    s_max01_upload_needed,  // upload_needed
    s_max01_measure,        // measure
};
#endif

#ifdef TWIMAX17043
static sensor_t max43v = {
    "s-max43v",
    {0},                    // state
    SENSOR_TWI_MAX17043,    // storage_sensor_mask
    0,                      // init_parameter
    NO_HANDLER,             // init_once
    s_max43_voltage_init,   // init_power
    s_max43_voltage_term,   // term_power
    NO_HANDLER,             // show_value
    0,                      // poll_repeat_milliseconds
    false,                  // poll_continuously
    false,                  // poll_during_settling
    NO_HANDLER,             // poll_handler
    0,                      // settling_seconds
    NO_HANDLER,             // done_settling
    NO_HANDLER,             // done_group_settling
    s_max43_voltage_upload_needed, // upload_needed
    s_max43_voltage_measure,// measure
};
static sensor_t max43s = {
    "s-max43%",
    {0},                    // state
    SENSOR_TWI_MAX17043,    // storage_sensor_mask
    0,                      // init_parameter
    NO_HANDLER,             // init_once
    s_max43_soc_init,       // init_power
    s_max43_soc_term,       // term_power
    NO_HANDLER,             // show_value
    0,                      // poll_repeat_milliseconds
    false,                  // poll_continuously
    false,                  // poll_during_settling
    NO_HANDLER,             // poll_handler
    0,                      // settling_seconds
    NO_HANDLER,             // done_settling
    NO_HANDLER,             // done_group_settling
    s_max43_soc_upload_needed,// upload_needed
    s_max43_soc_measure,    // measure
};
#endif

static repeat_t solarcast_basics_group_repeat[] = {
    {
        BAT_TEST|BAT_BURN,  // active_battery_status
        5*60                // repeat_seconds
    },
    {
        BAT_MOBILE,         // active_battery_status
        15*60               // repeat_seconds
    },
    {
        BAT_ALL,            // active_battery_status
        30*60               // repeat_seconds
    }
};
    
static group_t solarcast_basics_group = {
    "g-basics",
    {0},                    // state
    PRODUCT_SOLARCAST,      // storage_product
    BAT_ALL,                // active_battery_status
    COMM_NONE|COMM_LORA|COMM_FONA, // active_comm_mode
    NO_HANDLER,             // skip_handler
    NO_HANDLER,             // power_handler
    SENSOR_PIN_UNDEFINED,   // power_parameter
// Don't measure current during any other measurements
#if defined(TWIINA219) || defined(TWIMAX17201)
    true,                   // exclusive
#else
    DEFAULT_EXCLUSIVE,      // exclusive
#endif
    false,                  // power_exclusive
    true,                   // twi_exclusive
    0,                      // poll_repeat_milliseconds
    false,                  // poll_continuously
    false,                  // poll_during_settling
    NO_HANDLER,             // poll_handler
    0,                      // settling_seconds
    NO_HANDLER,             // done_settling
    true,                   // sense_at_boot
    solarcast_basics_group_repeat,
    UART_NONE,              // uart_required
    UART_NONE,              // uart_requested
    {                       // sensors
#ifdef TWIMAX17201
        &max01,
#endif
#ifdef TWIMAX17043
        &max43v,
        &max43s,
#endif
#ifdef TWIINA219
        &ina219,
#endif
#ifdef TWIHIH6130
        &temphumidity,
#endif
#if defined(TWIBME0) && !defined(TWIBME0AIR)
        &bme0,
#endif
        END_OF_LIST,
    },
};

#if defined(TWIBME1)
static repeat_t solarcast_board_group_repeat[] = {
    {
        BAT_TEST|BAT_BURN,  // active_battery_status
        5*60                // repeat_seconds
    },
    {
        BAT_ALL,            // active_battery_status
        30*60               // repeat_seconds
    }
};
#endif

#if defined(TWIBME1)
static group_t solarcast_board_group = {
    "g-board",
    {0},                    // state
    PRODUCT_SOLARCAST,      // storage_product
    BAT_ALL,                // active_battery_status
    COMM_NONE|COMM_LORA|COMM_FONA, // active_comm_mode
    NO_HANDLER,             // skip_handler
    NO_HANDLER,             // power_handler
    SENSOR_PIN_UNDEFINED,   // power_parameter
    DEFAULT_EXCLUSIVE,      // exclusive
    false,                  // power_exclusive
    true,                   // twi_exclusive
    0,                      // poll_repeat_milliseconds
    false,                  // poll_continuously
    false,                  // poll_during_settling
    NO_HANDLER,             // poll_handler
    0,                      // settling_seconds
    NO_HANDLER,             // done_settling
    false,                  // sense_at_boot
    solarcast_board_group_repeat,
    UART_NONE,              // uart_required
    UART_NONE,              // uart_requested
    {                       // sensors
        &bme1,
        END_OF_LIST,
    },
};
#endif

#ifdef TWILIS3DH

static sensor_t lis = {
    "s-motion",
    {0},                    // state
    SENSOR_TWI_LIS3DH,      // storage_sensor_mask
    0,                      // init_parameter
    NO_HANDLER,             // init_once
    s_lis_init,             // init_power
    s_lis_term,             // term_power
    NO_HANDLER,             // show_value
    1000,                   // poll_repeat_milliseconds
    false,                  // poll_continuously
    false,                  // poll_during_settling
    s_lis_poll,             // poll_handler
    0,                      // settling_seconds
    NO_HANDLER,             // done_settling
    NO_HANDLER,             // done_group_settling
    NO_HANDLER,             // upload_needed
    s_lis_measure,          // measure
};

static repeat_t solarcast_motion_group_repeat[] = {
    {
        BAT_TEST|BAT_BURN,  // active_battery_status
        5*60                // repeat_seconds
    },
    {
        BAT_ALL,            // active_battery_status
        15*60               // repeat_seconds
    }
};

static group_t solarcast_motion_group = {
    "g-motion",
    {0},                    // state
    PRODUCT_SOLARCAST,      // storage_product
    BAT_ALL,                // active_battery_status
    COMM_NONE|COMM_LORA|COMM_FONA, // active_comm_mode
    g_mobile_skip,          // skip_handler
    NO_HANDLER,             // power_handler
    SENSOR_PIN_UNDEFINED,   // power_parameter
    DEFAULT_EXCLUSIVE,      // exclusive
    false,                  // power_exclusive
    true,                   // twi_exclusive
    0,                      // poll_repeat_milliseconds
    false,                  // poll_continuously
    false,                  // poll_during_settling
    NO_HANDLER,             // poll_handler
    0,                      // settling_seconds
    NO_HANDLER,             // done_settling
    true,                   // sense_at_boot
    solarcast_motion_group_repeat,
    UART_NONE,              // uart_required
    UART_NONE,              // uart_requested
    {                       // sensors
        &lis,
        END_OF_LIST,
    },
};

#endif

#ifdef GEIGERX

static sensor_t geiger = {
    "s-geiger",
    {0},                    // state
    SENSOR_GPIO_GEIGER0|SENSOR_GPIO_GEIGER1, // storage_sensor_mask
    0,                      // init_parameter
    NO_HANDLER,             // init_once
    s_geiger_init,          // init_power
    s_geiger_term,          // term_power
    s_geiger_show_value,    // show_value
    GEIGER_BUCKET_SECONDS*1000, // poll_repeat_milliseconds
    false,                  // poll_continuously
    true,                   // poll_during_settling
    s_geiger_poll,          // poll_handler
    0,                      // settling_seconds
    NO_HANDLER,             // done_settling
    NO_HANDLER,             // done_group_settling
    s_geiger_upload_needed, // upload_needed
    s_geiger_measure,       // measure
};

static repeat_t solarcast_geiger_group_repeat[] = {
#ifdef GEIGERFAST
    // If in Geiger Fast mode, always have a geiger value ready
    // immediately after the prior value has been uploaded.
    {
        BAT_NORMAL|BAT_FULL|BAT_MOBILE|BAT_TEST|BAT_BURN,
        5*60                // repeat_seconds
    },
#else
    {
        BAT_MOBILE,
        5                   // repeat_seconds
    },
    {
        BAT_MOBILE|BAT_TEST|BAT_BURN,
        5*60                // repeat_seconds
    },
    {
        BAT_FULL,
        10*60               // repeat_seconds
    },
#endif
    {
        BAT_ALL,
        15*60               // repeat_seconds
    }
};

static group_t solarcast_geiger_group = {
    "g-geiger",
    {0},                    // state
    PRODUCT_SOLARCAST,      // storage_product
    BAT_NOT_DEAD,           // active_battery_status
    COMM_NONE|COMM_LORA|COMM_FONA, // active_comm_mode
    g_geiger_skip,          // skip_handler
    NO_HANDLER,             // power_handler
    SENSOR_PIN_UNDEFINED,   // power_parameter
    DEFAULT_EXCLUSIVE,      // exclusive
    false,                  // power_exclusive
    false,                  // twi_exclusive
    0,                      // poll_repeat_milliseconds
    false,                  // poll_continuously
    false,                  // poll_during_settling
    NO_HANDLER,             // poll_handler
    0,                      // settling_seconds
    NO_HANDLER,             // done_settling
    false,                  // sense_at_boot
    solarcast_geiger_group_repeat,
    UART_NONE,              // uart_required
    UART_NONE,              // uart_requested
    {                       // sensors
        &geiger,
        END_OF_LIST,
    },
};

#endif // GEIGERX

#ifdef TWIUBLOXM8
static sensor_t gps = {
    "s-twigps",
    {0},                    // state
    SENSOR_TWI_UBLOXM8,     // storage_sensor_mask
    0,                      // init_parameter
    NO_HANDLER,             // init_once
    s_gps_init,             // init_power
    s_gps_term,             // term_power
    NO_HANDLER,             // show_value
    1000,                   // poll_repeat_milliseconds, 1s polling required by ublox i2c or they shut down chip
    false,                  // poll_continuously
    true,                   // poll_during_settling
    s_gps_poll,             // poll_handler
    0,                      // settling_seconds
    NO_HANDLER,             // done_settling
    NO_HANDLER,             // done_group_settling
    NO_HANDLER,             // upload_needed
    NO_HANDLER,             // measure
};

static repeat_t solarcast_gps_group_repeat[] = {
    {
        BAT_ALL,
        (24+1)*60*60        // repeat_seconds (~daily, but shift the time so we catch differing satellites)
    }
};

static group_t solarcast_gps_group = {
    "g-twigps",
    {0},                    // state
    PRODUCT_SOLARCAST,      // storage_product
    BAT_ALL,                // active_battery_status
    COMM_NONE|COMM_LORA|COMM_FONA, // active_comm_mode
    NO_HANDLER,             // skip_handler
    sensor_set_pin_state,   // power_handler
    POWER_PIN_GPS,          // power_parameter
    DEFAULT_EXCLUSIVE,      // exclusive
    false,                  // power_exclusive
    true,                   // twi_exclusive
    0,                      // poll_repeat_milliseconds
    false,                  // poll_continuously
    false,                  // poll_during_settling
    NO_HANDLER,             // poll_handler
    30,                     // settling_seconds
    NO_HANDLER,             // done_settling
    false,                  // sense_at_boot
    solarcast_gps_group_repeat,
    UART_NONE,              // uart_required
    UART_NONE,              // uart_requested
    {                       // sensors
        &gps,
        END_OF_LIST,
    },
};
#endif

#ifdef UGPS
static sensor_t ugps = {
    "s-ugps",
    {0},                    // state
    SENSOR_UART_UGPS,       // storage_sensor_mask
    0,                      // init_parameter
    NO_HANDLER,             // init_once
    s_ugps_init,            // init_power
    s_ugps_term,            // term_power
    NO_HANDLER,             // show_value
    (GPS_POLL_SECONDS*1000), // poll_repeat_milliseconds
    false,                  // poll_continuously
    true,                   // poll_during_settling
    s_ugps_poll,            // poll_handler
    0,                      // settling_seconds
    NO_HANDLER,             // done_settling
    NO_HANDLER,             // done_group_settling
    NO_HANDLER,             // upload_needed
    NO_HANDLER,             // measure
};

static repeat_t solarcast_ugps_group_repeat[] = {
    {
        BAT_MOBILE,
        5                   // repeat_seconds
    },
    {
        // This simply defines UGPS response time when there's a motion change event.
        // Otherwise, UGPS is suppressed via g_ugps_skip, which is what triggers the resampling.
        BAT_ALL,
        10*60               // repeat_seconds
    }
};

static group_t solarcast_ugps_group = {
    "g-ugps",
    {0},                    // state
    PRODUCT_SOLARCAST,      // storage_product
    BAT_ALL,                // active_battery_status
    COMM_NONE|COMM_LORA|COMM_FONA, // active_comm_mode
    g_ugps_skip,            // skip_handler
    NO_HANDLER,             // power_handler
    SENSOR_PIN_UNDEFINED,   // power_parameter
    DEFAULT_EXCLUSIVE,      // exclusive
    false,                  // power_exclusive
    false,                  // twi_exclusive
    0,                      // poll_repeat_milliseconds
    false,                  // poll_continuously
    false,                  // poll_during_settling
    NO_HANDLER,             // poll_handler
    0,                      // settling_seconds
    NO_HANDLER,             // done_settling
    true,                   // sense_at_boot
    solarcast_ugps_group_repeat,
    UART_GPS,               // uart_required
    UART_NONE,              // uart_requested
    {                       // sensors
        &ugps,
        END_OF_LIST,
    },
};
#endif

#if defined(PMSX) && !defined(AIRX)

static sensor_t pms = {
    "s-pms",
    {0},                    // state
    SENSOR_UART_PMS,        // storage_sensor_mask
    0,                      // init_parameter
    NO_HANDLER,             // init_once
    s_pms_init,             // init_power
    s_pms_term,             // term_power
    s_pms_show_value,       // show_value
    (AIR_SAMPLE_SECONDS*1000), // poll_repeat_milliseconds
    false,                  // poll_continuously
    false,                  // poll_during_settling
    s_pms_poll,             // poll_handler
    AIR_SAMPLE_PERIOD_SECONDS, // settling_seconds
    NO_HANDLER,             // done_settling
    NO_HANDLER,             // done_group_settling
    s_pms_upload_needed,    // upload_needed
    s_pms_measure,          // measure
};

static repeat_t solarcast_pms_group_repeat[] = {
    {
        BAT_TEST|BAT_BURN,  // active_battery_status
        5*60                // repeat_seconds
    },
    {
        BAT_FULL,           // active_battery_status
        10*60               // repeat_seconds
    },
    {
        BAT_ALL,
        30*60               // repeat_seconds
    }
};

static group_t solarcast_pms_group = {
    "g-pms",
    {0},                    // state
    PRODUCT_SOLARCAST,      // storage_product
    BAT_HEALTHY,            // active_battery_status
    COMM_NONE|COMM_LORA|COMM_FONA, // active_comm_mode
    g_mobile_skip,          // skip_handler
    sensor_set_pin_state,   // power_handler
    POWER_PIN_AIR,          // power_parameter
    DEFAULT_EXCLUSIVE,      // exclusive
    true,                   // power_exclusive
#if PMSX==IOUART
    false,                  // twi_exclusive
#else
    true,                   // twi_exclusive
#endif
    0,                      // poll_repeat_milliseconds
    false,                  // poll_continuously
    false,                  // poll_during_settling
    NO_HANDLER,             // poll_handler
    0,                      // settling_seconds
    NO_HANDLER,             // done_settling
    false,                  // sense_at_boot
    solarcast_pms_group_repeat,
#if defined(PMSX) && PMSX==IOUART
    UART_PMS,               // uart_required
#else
    UART_NONE,              // uart_required
#endif
    UART_NONE,              // uart_requested
    {                       // sensors
        &pms,
        END_OF_LIST,
    },
};

#endif // PMSX && !AIRX

#if defined(SPIOPC) && !defined(AIRX)

static sensor_t opc = {
    "s-opc",
    {0},                    // state
    SENSOR_SPI_OPC,         // storage_sensor_mask
    0,                      // init_parameter
    NO_HANDLER,             // init_once
    s_opc_init,             // init_power
    s_opc_term,             // term_power
    s_opc_show_value,       // show_value
    (AIR_SAMPLE_SECONDS*1000), // poll_repeat_milliseconds
    false,                  // poll_continuously
    false,                  // poll_during_settling
    s_opc_poll,             // poll_handler
    AIR_SAMPLE_PERIOD_SECONDS, // settling_seconds
    NO_HANDLER,             // done_settling
    NO_HANDLER,             // done_group_settling
    s_opc_upload_needed,    // upload_needed
    s_opc_measure,          // measure
};

static repeat_t solarcast_opc_group_repeat[] = {
    {
        BAT_TEST|BAT_BURN,  // active_battery_status
        5*60                // repeat_seconds
    },
    {
        BAT_FULL,           // active_battery_status
        10*60               // repeat_seconds
    },
    {
        BAT_ALL,
        30*60               // repeat_seconds
    }
};

static group_t solarcast_opc_group = {
    "g-opc",
    {0},                    // state
    PRODUCT_SOLARCAST,      // storage_product
    BAT_HEALTHY,            // active_battery_status
    COMM_NONE|COMM_LORA|COMM_FONA, // active_comm_mode
    g_mobile_skip,          // skip_handler
    sensor_set_pin_state,   // power_handler
    POWER_PIN_AIR,          // power_parameter
    DEFAULT_EXCLUSIVE,      // exclusive
    true,                   // power_exclusive
    false,                  // twi_exclusive
    0,                      // poll_repeat_milliseconds
    false,                  // poll_continuously
    false,                  // poll_during_settling
    NO_HANDLER,             // poll_handler
    0,                      // settling_seconds
    NO_HANDLER,             // done_settling
    false,                  // sense_at_boot
    solarcast_opc_group_repeat,
    UART_NONE,              // uart_required
    UART_NONE,              // uart_requested
    {                       // sensors
        &opc,
        END_OF_LIST,
    },
};

#endif // SPIOPC && !AIRX

#ifdef AIRX

static sensor_t air = {
    "s-air",
    {0},                    // state
    SENSOR_SPI_OPC|SENSOR_UART_PMS, // storage_sensor_mask
    0,                      // init_parameter
    NO_HANDLER,             // init_once
    s_air_init,             // init_power
    s_air_term,             // term_power
    s_air_show_value,       // show_value
    (AIR_SAMPLE_SECONDS*1000), // poll_repeat_milliseconds
    false,                  // poll_continuously
    false,                  // poll_during_settling
    s_air_poll,             // poll_handler
    AIR_SAMPLE_PERIOD_SECONDS, // settling_seconds
    NO_HANDLER,             // done_settling
    NO_HANDLER,             // done_group_settling
    s_air_upload_needed,    // upload_needed
    s_air_measure,          // measure
};

static repeat_t solarcast_air_group_repeat[] = {
    {
        BAT_TEST|BAT_BURN,  // active_battery_status
        5*60                // repeat_seconds
    },
    {
        BAT_FULL,           // active_battery_status
        10*60               // repeat_seconds
    },
    {
        BAT_NORMAL,         // active_battery_status
        30*60               // repeat_seconds
    },
    {
        BAT_LOW,
        60*60               // repeat_seconds
    },
    {
        BAT_ALL,
        120*60              // repeat_seconds
    }
};

static group_t solarcast_air_group = {
    "g-air",
    {0},                    // state
    PRODUCT_SOLARCAST,      // storage_product
    BAT_HEALTHY,            // active_battery_status
    COMM_NONE|COMM_LORA|COMM_FONA, // active_comm_mode
    g_mobile_skip,          // skip_handler
    sensor_set_pin_state,   // power_handler
    POWER_PIN_AIR,          // power_parameter
    DEFAULT_EXCLUSIVE,      // exclusive
    true,                   // power_exclusive
#if defined(TWIBME0AIR)
    true,                   // twi_exclusive
#else
    false,                  // twi_exclusive
#endif
    0,                      // poll_repeat_milliseconds
    false,                  // poll_continuously
    false,                  // poll_during_settling
    NO_HANDLER,             // poll_handler
    0,                      // settling_seconds
    NO_HANDLER,             // done_settling
    false,                  // sense_at_boot
    solarcast_air_group_repeat,
    UART_NONE,              // uart_required
#if defined(PMSX) && PMSX==IOUART
    UART_PMS,               // uart_requested
#else
    UART_NONE,              // uart_requested
#endif
    {                       // sensors
        &air,
#if defined(TWIBME0AIR)
        &bme0,
#endif
        END_OF_LIST,
    },
};
    
#endif // AIRX

static group_t *sensor_groups[] = {
    &solarcast_basics_group,
#ifdef TWILIS3DH
    &solarcast_motion_group,
#endif
#ifdef GEIGERX
    &solarcast_geiger_group,
#endif
#ifdef TWIUBLOXM8
    &solarcast_gps_group,
#endif
#ifdef UGPS
    &solarcast_ugps_group,
#endif
#ifdef AIRX
    &solarcast_air_group,
#endif
#if defined(SPIOPC) && !defined(AIRX)
    &solarcast_opc_group,
#endif
#if defined(PMSX) && !defined(AIRX)
    &solarcast_pms_group,
#endif
#if defined(TWIBME1)
    &solarcast_board_group,
#endif
    END_OF_LIST,
};
