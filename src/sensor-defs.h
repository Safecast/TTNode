// Master definitions for all products, and all sensors within those products

#ifdef TWIHIH6130
static sensor_t temphumidity = {
    "s-temp",
    {0},                    // state
    SENSOR_TWI_HIH6130,     // storage_sensor_mask
    NO_HANDLER,             // init_once
    s_hih6130_init,         // init_power
    s_hih6130_term,         // term_power
    30,                     // settling_seconds
    s_hih6130_upload_needed,// upload_needed
    s_hih6130_measure,      // measure
};
#endif

#ifdef TWIBME280
static sensor_t bme280 = {
    "s-bme",
    {0},                    // state
    SENSOR_TWI_BME280 ,     // storage_sensor_mask
    NO_HANDLER,             // init_once
    s_bme280_init,          // init_power
    s_bme280_term,          // term_power
    5,                      // settling_seconds
    s_bme280_upload_needed, // upload_needed
    s_bme280_measure,       // measure
};
#endif

#ifdef TWIINA219
static sensor_t ina219 = {
    "s-ina",
    {0},                    // state
    SENSOR_TWI_INA219,      // storage_sensor_mask
    NO_HANDLER,             // init_once
    s_ina_init,             // init_power
    s_ina_term,             // term_power
    0,                      // settling_seconds
    s_ina_upload_needed,    // upload_needed
    s_ina_measure,          // measure
};
#endif

#ifdef TWIMAX17043
static sensor_t batvoltage = {
    "s-batv",
    {0},                    // state
    SENSOR_TWI_MAX17043,    // storage_sensor_mask
    NO_HANDLER,             // init_once
    s_bat_voltage_init,     // init_power
    s_bat_voltage_term,     // term_power
    0,                      // settling_seconds
    s_bat_voltage_upload_needed, // upload_needed
    s_bat_voltage_measure,  // measure
};
#endif

#ifdef TWIMAX17043
static sensor_t batsoc = {
    "s-bat%",
    {0},                    // state
    SENSOR_TWI_MAX17043,    // storage_sensor_mask
    NO_HANDLER,             // init_once
    s_bat_soc_init,         // init_power
    s_bat_soc_term,         // term_power
    0,                      // settling_seconds
    s_bat_soc_upload_needed,// upload_needed
    s_bat_soc_measure,      // measure
};
#endif

static group_t simplecast_basics_group = {
    "g-basics",
    {0},                    // state
    PRODUCT_SIMPLECAST,     // storage_product
    BAT_FULL|BAT_NORMAL|BAT_WARNING|BAT_EMERGENCY|BAT_DEAD, // active_battery_status
    COMM_NONE|COMM_LORA|COMM_FONA, // active_comm_mode
#if defined(TWIHIH6130) || defined(TWIBME280) || defined(TWIINA219)
    sensor_set_pin_state,   // power_handler
    POWER_PIN_BASICS,       // power_parameter
#else
    NO_HANDLER,             // power_handler
    PIN_UNDEFINED,          // power_parameter
#endif
#if defined(TWIINA219) && !defined(CURRENTDEBUG) // Don't measure current during any other measurements
    true,                   // power_exclusive
#else
    false,                  // power_exclusive
#endif
#ifdef TWIINA219            // Don't measure current during any other measurements
    (PWR_SAMPLE_SECONDS*1000), // poll_repeat_milliseconds
    false,                  // poll_continuously
    false,                  // poll_during_settling
    s_ina_poll,             // poll_handler
#else
    0,                      // poll_repeat_milliseconds
    false,                  // poll_continuously
    false,                  // poll_during_settling
    NO_HANDLER,             // poll_handler
#endif
    0,                      // settling_seconds
    NO_HANDLER,             // done_settling
#if !defined(GEIGER) && !defined(PMSX) && !defined(SPIOPC) && ~defined(AIRX)
    2,                      // repeat-minutes (debugging)
#else
    30,                     // repeat_minutes
#endif
    UART_NONE,              // uart_required
    UART_NONE,              // uart_requested
    {                       // sensors
#ifdef TWIHIH6130
        &temphumidity,
#endif
#ifdef TWIBME280
        &bme280,
#endif
#ifdef TWIMAX17043
        &batvoltage,
        &batsoc,
#endif
#ifdef TWIINA219
        &ina219,
#endif
        END_OF_LIST,
    },
};

#ifdef TWILIS3DH

static sensor_t motion = {
    "s-motion",
    {0},                    // state
    SENSOR_TWI_LIS3DH,      // storage_sensor_mask
    NO_HANDLER,             // init_once
    s_lis_init,             // init_power
    s_lis_term,             // term_power
    0,                      // settling_seconds
    NO_HANDLER,             // upload_needed
    s_lis_measure,          // measure
};

static group_t simplecast_motion_group = {
    "g-motion",
    {0},                    // state
    PRODUCT_SIMPLECAST,     // storage_product
    BAT_FULL|BAT_NORMAL|BAT_WARNING|BAT_EMERGENCY|BAT_DEAD, // active_battery_status
    COMM_NONE|COMM_LORA|COMM_FONA, // active_comm_mode
    NO_HANDLER,             // power_handler
    PIN_UNDEFINED,          // power_parameter
    false,                  // power_exclusive
    1000,                   // poll_repeat_milliseconds
    false,                  // poll_continuously
    false,                  // poll_during_settling
    s_lis_poll,             // poll_handler
    0,                      // settling_seconds
    NO_HANDLER,             // done_settling
#ifdef MOTIONDEBUG
    1,                      // repeat_minutes
#else
    15,                     // repeat_minutes
#endif
    UART_NONE,              // uart_required
    UART_NONE,              // uart_requested
    {                       // sensors
        &motion,
        END_OF_LIST,
    },
};

#endif

#ifdef GEIGER

static sensor_t geiger = {
    "s-geiger",
    {0},                    // state
    SENSOR_GPIO_GEIGER0|SENSOR_GPIO_GEIGER1, // storage_sensor_mask
    NO_HANDLER,             // init_once
    s_geiger_init,          // init_power
    NO_HANDLER,             // term_power
    GEIGER_SAMPLE_SECONDS,  // settling_seconds
    s_geiger_upload_needed, // upload_needed
    s_geiger_measure,       // measure
};

static group_t simplecast_geiger_group = {
    "g-geiger",
    {0},                    // state
    PRODUCT_SIMPLECAST,     // storage_product
    BAT_NORMAL|BAT_WARNING|BAT_EMERGENCY, // active_battery_status
    COMM_NONE|COMM_LORA|COMM_FONA, // active_comm_mode
    sensor_set_pin_state,   // power_handler
#ifdef POWER_PIN_GEIGER
    POWER_PIN_GEIGER,       // power_parameter
#else
    PIN_UNDEFINED,          // power_parameter
#endif
    false,                  // power_exclusive
    GEIGER_BUCKET_SECONDS*1000, // poll_repeat_milliseconds
    false,                  // poll_continuously
    true,                   // poll_during_settling
    s_geiger_poll,          // poll_handler
    10,                     // settling_seconds
    NO_HANDLER,             // done_settling
    15,                     // repeat_minutes
    UART_NONE,              // uart_required
    UART_NONE,              // uart_requested
    {                       // sensors
        &geiger,
        END_OF_LIST,
    },
};

static group_t simplecast_geiger_fast_group = {
    "g-geiger-fast",
    {0},                    // state
    PRODUCT_SIMPLECAST,     // storage_product
    BAT_FULL,               // active_battery_status
    COMM_NONE|COMM_LORA|COMM_FONA, // active_comm_mode
    sensor_set_pin_state,   // power_handler
#ifdef POWER_PIN_GEIGER
    POWER_PIN_GEIGER,       // power_parameter
#else
    PIN_UNDEFINED,          // power_parameter
#endif
    false,                  // power_exclusive
    GEIGER_BUCKET_SECONDS*1000, // poll_repeat_milliseconds
    false,                  // poll_continuously
    true,                   // poll_during_settling
    s_geiger_poll,          // poll_handler
    10,                     // settling_seconds
    NO_HANDLER,             // done_settling
    10,                     // repeat_minutes
    UART_NONE,              // uart_required
    UART_NONE,              // uart_requested
    {                       // sensors
        &geiger,
        END_OF_LIST,
    },
};

#endif // GEIGER

#ifdef TWIUBLOXM8
static sensor_t gps = {
    "s-gpsm8",
    {0},                    // state
    SENSOR_TWI_UBLOXM8,     // storage_sensor_mask
    NO_HANDLER,             // init_once
    s_gps_init,             // init_power
    s_gps_term,             // term_power
    0,                      // settling_seconds
    NO_HANDLER,             // upload_needed
    NO_HANDLER,             // measure
};
#endif

#ifdef TWIUBLOXM8
static group_t simplecast_gps_group = {
    "g-gps",
    {0},                    // state
    PRODUCT_SIMPLECAST,     // storage_product
    BAT_FULL|BAT_NORMAL|BAT_WARNING|BAT_EMERGENCY, // active_battery_status
    COMM_NONE|COMM_LORA|COMM_FONA, // active_comm_mode
    sensor_set_pin_state,   // power_handler
    POWER_PIN_GPS,          // power_parameter
    false,                  // power_exclusive
    1000,                   // poll_repeat_milliseconds, 1s polling required by ublox i2c or they shut down chip
    false,                  // poll_continuously
    true,                   // poll_during_settling
    s_gps_poll,             // poll_handler
    30,                     // settling_seconds
    NO_HANDLER,             // done_settling
    (24+1)*60,              // repeat_minutes (~daily, but shift the time so we catch differing satellites)
    UART_NONE,              // uart_required
    UART_NONE,              // uart_requested
    {                       // sensors
        &gps,
        END_OF_LIST,
    },
};
#endif

#if defined(PMSX) && !defined(AIRX)

static sensor_t pms = {
    "s-pms",
    {0},                    // state
    SENSOR_UART_PMS,        // storage_sensor_mask
    NO_HANDLER,             // init_once
    s_pms_init,             // init_power
    s_pms_term,             // term_power
    AIR_SAMPLE_PERIOD_SECONDS, // settling_seconds
    s_pms_upload_needed,    // upload_needed
    s_pms_measure,          // measure
};

static group_t simplecast_pms_group = {
    "g-pms",
    {0},                    // state
    PRODUCT_SIMPLECAST,     // storage_product
    BAT_FULL|BAT_NORMAL|BAT_WARNING|BAT_EMERGENCY, // active_battery_status
    COMM_NONE|COMM_LORA|COMM_FONA, // active_comm_mode
    sensor_set_pin_state,   // power_handler
    POWER_PIN_AIR,          // power_parameter
    true,                   // power_exclusive
    (AIR_SAMPLE_SECONDS*1000), // poll_repeat_milliseconds
    false,                  // poll_continuously
    false,                  // poll_during_settling
    s_pms_poll,             // poll_handler
    0,                      // settling_seconds
    s_pms_done_settling,    // done_settling
    30,                     // repeat_minutes
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
    NO_HANDLER,             // init_once
    s_opc_init,             // init_power
    s_opc_term,             // term_power
    AIR_SAMPLE_PERIOD_SECONDS, // settling_seconds
    s_opc_upload_needed,    // upload_needed
    s_opc_measure,          // measure
};

static group_t simplecast_opc_group = {
    "g-opc",
    {0},                    // state
    PRODUCT_SIMPLECAST,     // storage_product
    BAT_FULL|BAT_NORMAL|BAT_WARNING|BAT_EMERGENCY, // active_battery_status
    COMM_NONE|COMM_LORA|COMM_FONA, // active_comm_mode
    sensor_set_pin_state,   // power_handler
    POWER_PIN_AIR,          // power_parameter
    true,                   // power_exclusive
    (AIR_SAMPLE_SECONDS*1000), // poll_repeat_milliseconds
    false,                  // poll_continuously
    false,                  // poll_during_settling
    s_opc_poll,             // poll_handler
    0,                      // settling_seconds
    s_opc_done_settling,    // done_settling
    30,                     // repeat_minutes
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
    NO_HANDLER,             // init_once
    s_air_init,             // init_power
    s_air_term,             // term_power
    AIR_SAMPLE_PERIOD_SECONDS, // settling_seconds
    s_air_upload_needed,    // upload_needed
    s_air_measure,          // measure
};

static group_t simplecast_air_group = {
    "g-air",
    {0},                    // state
    PRODUCT_SIMPLECAST,     // storage_product
    BAT_NORMAL|BAT_WARNING|BAT_EMERGENCY, // active_battery_status
    COMM_NONE|COMM_LORA|COMM_FONA, // active_comm_mode
    sensor_set_pin_state,   // power_handler
    POWER_PIN_AIR,          // power_parameter
    true,                   // power_exclusive
    (AIR_SAMPLE_SECONDS*1000), // poll_repeat_milliseconds
    false,                  // poll_continuously
    false,                  // poll_during_settling
    s_air_poll,             // poll_handler
    0,                      // settling_seconds
    s_air_done_settling,    // done_settling
    30,                     // repeat_minutes
    UART_NONE,              // uart_required
#if defined(PMSX) && PMSX==IOUART
    UART_PMS,               // uart_requested
#else
    UART_NONE,              // uart_requested
#endif
    {                       // sensors
        &air,
        END_OF_LIST,
    },
};

static group_t simplecast_air_fast_group = {
    "g-air-fast",
    {0},                    // state
    PRODUCT_SIMPLECAST,     // storage_product
    BAT_FULL,               // active_battery_status
    COMM_NONE|COMM_LORA|COMM_FONA, // active_comm_mode
    sensor_set_pin_state,   // power_handler
    POWER_PIN_AIR,          // power_parameter
    true,                   // power_exclusive
    (AIR_SAMPLE_SECONDS*1000), // poll_repeat_milliseconds
    false,                  // poll_continuously
    false,                  // poll_during_settling
    s_air_poll,             // poll_handler
    0,                      // settling_seconds
    s_air_done_settling,    // done_settling
    10,                     // repeat_minutes
    UART_NONE,              // uart_required
#if defined(PMSX) && PMSX==IOUART
    UART_PMS,               // uart_requested
#else
    UART_NONE,              // uart_requested
#endif
    {                       // sensors
        &air,
        END_OF_LIST,
    },
};
    
#endif // AIRX

static group_t *sensor_groups[] = {
#ifdef GEIGER
    &simplecast_geiger_group,
    &simplecast_geiger_fast_group,
#endif
#ifdef TWILIS3DH
    &simplecast_motion_group,
#endif
    &simplecast_basics_group,
#ifdef TWIUBLOXM8
    &simplecast_gps_group,
#endif
#ifdef AIRX
    &simplecast_air_group,
    &simplecast_air_fast_group,
#endif
#if defined(SPIOPC) && !defined(AIRX)
    &simplecast_opc_group,
#endif
#if defined(PMSX) && !defined(AIRX)
    &simplecast_pms_group,
#endif
    END_OF_LIST,
};
