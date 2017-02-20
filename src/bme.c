// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

// BME280 Temp/Humidity Sensor Support

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "debug.h"
#include "boards.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_drv_twi.h"
#include "app_util_platform.h"
#include "app_twi.h"
#include "gpio.h"
#include "config.h"
#include "timer.h"
#include "sensor.h"
#include "storage.h"
#include "comm.h"
#include "misc.h"
#include "twi.h"
#include "io.h"
#include "bme.h"

#ifdef TWIBME280

// Default I2C address
#define BME280_I2C_ADDRESS      0x77

// Our primary chip operating modes
// (Note that FORCED causes the chip to remain in low-power mode except when measuring)
#define SAMPLING_HUMIDITY       SAMPLING_1x
#define SAMPLING_TEMPERATURE    SAMPLING_1x
#define SAMPLING_PRESSURE       SAMPLING_1x
#define SAMPLING_MODE           MODE_FORCED

enum sampling_t {
    SAMPLING_0x         = 0x00,                 ///< skipped, output set to 0x80000
    SAMPLING_1x         = 0x01,                 ///< oversampling x1
    SAMPLING_2x         = 0x02,                 ///< oversampling x2
    SAMPLING_4x         = 0x03,                 ///< oversampling x4
    SAMPLING_8x         = 0x04,                 ///< oversampling x8
    SAMPLING_16x        = 0x05                  ///< oversampling x16
};
/// Standby times for auto mode, ignored in forced mode
enum standby_t {
    STANDBY_0_5         = 0x0,                  ///<    0.5ms standby between autonomous measurements
    STANDBY_62_5        = 0x1,                  ///<   62.5ms standby time
    STANDBY_125         = 0x2,                  ///<  125.0ms standby time
    STANDBY_250         = 0x3,                  ///<  250.0ms standby time
    STANDBY_500         = 0x4,                  ///<  500.0ms standby time
    STANDBY_1000        = 0x5,                  ///< 1000.0ms standby time
    STANDBY_10          = 0x6,                  ///<   10.0ms standby time
    STANDBY_20          = 0x7                   ///<   20.0ms standby time
};
/// Filter settings
enum filter_t {
    FILTER_OFF          = 0x0,                  ///< no IIR filtering, immediately reaches 75% of step response
    FILTER_2            = 0x1,                  ///< 75% of step response reached after 2 samples
    FILTER_4            = 0x2,                  ///< 75% of step response reached after 5 samples
    FILTER_8            = 0x3,                  ///< 75% of step response reached after 11 samples
    FILTER_16           = 0x4                   ///< 75% of step response reached after 22 samples
};
/// Sensor mode
enum mode_t {
    MODE_SLEEP          = 0x00,                 ///< sleep mode, can be achieved by calling reset()
    MODE_FORCED         = 0x01,                 ///< forced mode, conversion only upon request
    MODE_AUTO           = 0x03                  ///< continuous measurement mode
};

#define    BME280_DIG_T1_REG   0x88
#define    BME280_DIG_T2_REG   0x8A
#define    BME280_DIG_T3_REG   0x8C
#define    BME280_DIG_P1_REG   0x8E
#define    BME280_DIG_P2_REG   0x90
#define    BME280_DIG_P3_REG   0x92
#define    BME280_DIG_P4_REG   0x94
#define    BME280_DIG_P5_REG   0x96
#define    BME280_DIG_P6_REG   0x98
#define    BME280_DIG_P7_REG   0x9A
#define    BME280_DIG_P8_REG   0x9C
#define    BME280_DIG_P9_REG   0x9E
#define    BME280_DIG_H1_REG   0xA1
#define    BME280_DIG_H2_REG   0xE1
#define    BME280_DIG_H3_REG   0xE3
#define    BME280_DIG_H4_REG   0xE4
#define    BME280_DIG_H5_REG   0xE5
#define    BME280_DIG_H6_REG   0xE7
#define    BME280_REGISTER_CHIPID           0xD0
#define    BME280_REGISTER_VERSION          0xD1
#define    BME280_REGISTER_SOFTRESET        0xE0
#define    BME280_REGISTER_CAL26            0xE1
#define    BME280_REGISTER_CONTROLHUMID     0xF2
#define    BME280_REGISTER_STATUS           0xF3
#define    BME280_REGISTER_CONTROL          0xF4
#define    BME280_REGISTER_CONFIG           0xF5
#define    BME280_REGISTER_PRESSUREDATA     0xF7
#define    BME280_REGISTER_TEMPDATA         0xFA
#define    BME280_REGISTER_HUMIDDATA        0xFD

#define    BME280_REGISTER_FIRST            0x88
#define    BME280_REGISTER_END              0xFF
#define    BME280_REGISTER_LENGTH           (BME280_REGISTER_END - BME280_REGISTER_FIRST)
#define    BME280_REGISTER_VALUES           0xF7
#define    BME280_REGISTER_VALUES_LENGTH    (BME280_REGISTER_END - BME280_REGISTER_VALUES)

#define     VAL_CALIB26_41_SIZE   0x10
#define     VAL_CALIB00_25_SIZE   0x1A
#define     VAL_CHIP_ID           0x60
#define     VAL_RESET             0xB6
#define     VAL_TRIGGER_FORCED_CONVERSION 0x00

#define     STATUS_IDLE           0x00
#define     STATUS_MEASURING      0x08
#define     STATUS_UPDATING       0x01
#define     STATUS_ERROR          0xFF

// Calibration data
// structure to hold the calibration data that is programmed into the sensor in the factory
// during manufacture

static bool have_calibration_data = false;
#define     DEFAULT_FINE_TEMP 0x1f3e6
static int32_t fine_temp;

static uint8_t chip_id;
static uint8_t version;
static uint8_t config;
static uint8_t status;
static uint8_t control;

static uint16_t dig_T1;
static int16_t  dig_T2;
static int16_t  dig_T3;
static uint16_t dig_P1;
static int16_t  dig_P2;
static int16_t  dig_P3;
static int16_t  dig_P4;
static int16_t  dig_P5;
static int16_t  dig_P6;
static int16_t  dig_P7;
static int16_t  dig_P8;
static int16_t  dig_P9;

static uint8_t  dig_H1;
static int16_t  dig_H2;
static uint8_t  dig_H3;
static int16_t  dig_H4;
static int16_t  dig_H5;
static int8_t   dig_H6;

// Various commands
static uint8_t cmd_CFG[2] = {BME280_REGISTER_CONFIG, (STANDBY_0_5 << 5) | (FILTER_OFF << 2)};
static uint8_t cmd_CH[2] = {BME280_REGISTER_CONTROLHUMID, SAMPLING_HUMIDITY};
static uint8_t cmd_C[2] = {BME280_REGISTER_CONTROL, ((SAMPLING_TEMPERATURE << 5) | (SAMPLING_PRESSURE << 2) | SAMPLING_MODE)};
static uint8_t reg_STATUS = BME280_REGISTER_STATUS;
static uint8_t val_STATUS;

// I/O for retrieving values
static uint8_t reg_V = BME280_REGISTER_VALUES;
static uint8_t val_V[BME280_REGISTER_VALUES_LENGTH];

static uint8_t reg_ALL = BME280_REGISTER_FIRST;
static uint8_t val_ALL[BME280_REGISTER_LENGTH];

// Values that we read from the chip; tempcal is a calibration value
static void *sensor;

// The values we send out after having measured
bool reported = false;
static float reported_temperature = 1.23;
static float reported_humidity = 2.34;
static float reported_pressure = 4.56;

// Initializaiton
static bool fInit = false;

// Forward

uint32_t initiate_read();

// Asynchronous continuation
void s_bme280_measure_3(ret_code_t result, void *ignore) {

    // Kill the sensor if we get an error
    if (result != NRF_SUCCESS) {
        DEBUG_PRINTF("BME280: Measurement error 3:(%d)\n", result);
        sensor_measurement_completed(sensor);
        return;
    }

    // First, extract the raw data into these variables
    int32_t adc_P = (((int32_t)val_V[0]) << 12) | (((int32_t)val_V[1]) << 4) | (((int32_t)val_V[2]) >> 4);
    int32_t adc_T = (((int32_t)val_V[3]) << 12) | (((int32_t)val_V[4]) << 4) | (((int32_t)val_V[5]) >> 4);
    int32_t adc_H = (((int32_t)val_V[6]) << 8) | ((int32_t)val_V[7]);

    // Exit if not yet ready
    if (adc_P == 0x80000 && adc_T == 0x80000 && adc_H == 0x8000) {
        DEBUG_PRINTF("BME280: No data.\n");
        nrf_delay_ms(500);
        s_bme280_measure(sensor);
        return;
    }

    // Read Temperature
    float temperature;
    float vt_x1,vt_x2;
    vt_x1 = (((float)adc_T) / 16384.0 - ((float)dig_T1) / 1024.0) * ((float)dig_T2);
    vt_x2 = ((float)adc_T) / 131072.0 - ((float)dig_T1) / 8192.0;
    vt_x2 = (vt_x2 * vt_x2) * ((float)dig_T3);
    fine_temp = (uint32_t)(vt_x1 + vt_x2);
    temperature = ((vt_x1 + vt_x2) / 5120.0);

    // Read Humidity
    float humidity;
    float h;
    h = (((float)fine_temp) - 76800.0);
    if (h != 0) {
        h = (adc_H - (((float)dig_H4) * 64.0 + ((float)dig_H5) / 16384.0 * h));
        h = h * (((float)dig_H2) / 65536.0 * (1.0 + ((float)dig_H6) / 67108864.0 * h * (1.0 + ((float)dig_H3) / 67108864.0 * h)));
        h = h * (1.0 - ((float)dig_H1) * h / 524288.0);
        if (h > 100.0) {
            h = 100.0;
        } else if (h < 0.0) {
            h = 0.0;
        }
    }
    humidity = h;

    // Read Pressure
    float pressure;
    float vp_x1, vp_x2, p;
    vp_x1 = ((float)fine_temp / 2.0) - 64000.0;
    vp_x2 = vp_x1 * vp_x1 * ((float)dig_P6) / 32768.0;
    vp_x2 = vp_x2 + vp_x1 * ((float)dig_P5) * 2.0;
    vp_x2 = (vp_x2 / 4.0) + (((float)dig_P4) * 65536.0);
    vp_x1 = (((float)dig_P3) * vp_x1 * vp_x1 / 524288.0 + ((float)dig_P2) * vp_x1) / 524288.0;
    vp_x1 = (1.0 + vp_x1 / 32768.0) * ((float)dig_P1);
    if (vp_x1 == 0)
        p = 0;
    else {
        p = 1048576.0 - (float)adc_P;
        p = (p - (vp_x2 / 4096.0)) * 6250.0 / vp_x1;
        vp_x1 = ((float)dig_P9) * p * p / 2147483648.0;
        vp_x2 = p * ((float)dig_P8) / 32768.0;
        p += (vp_x1 + vp_x2 + ((float)dig_P7)) / 16.0;
        p = p * .01f;
    }
    pressure = p;

    // Debug
    if (debug(DBG_SENSOR_MAX))
        DEBUG_PRINTF("BME: %.3fC %.3f%% %.3fPa\n", temperature, humidity, pressure);

    // Done.
    reported_temperature = temperature;
    reported_humidity = humidity;
    reported_pressure = pressure;
    reported = true;
    sensor_measurement_completed(sensor);
}

// Start a TWI read of the data values
uint32_t initiate_read() {
    uint32_t err_code;
    static app_twi_transfer_t const mtransfers3[] = {
        APP_TWI_WRITE(BME280_I2C_ADDRESS, &reg_V, sizeof(reg_V), APP_TWI_NO_STOP),
        APP_TWI_READ(BME280_I2C_ADDRESS, &val_V, sizeof(val_V), 0)
    };
    static app_twi_transaction_t const mtransaction3 = {
        .callback            = s_bme280_measure_3,
        .p_user_data         = NULL,
        .p_transfers         = mtransfers3,
        .number_of_transfers = sizeof(mtransfers3) / sizeof(mtransfers3[0])
    };
    err_code = app_twi_schedule(twi_context(), &mtransaction3);
    if (err_code != NRF_SUCCESS)
        DEBUG_CHECK(err_code);
    return err_code;
}

// Asynchronous continuation of measurement
void s_bme280_measure_2(ret_code_t result, void *ignore) {
    uint32_t err_code;

    // Kill the sensor if we get an error
    if (result != NRF_SUCCESS) {
        DEBUG_PRINTF("BME280: Measurement error 2:(%d)\n", result);
        sensor_measurement_completed(sensor);
        return;
    }

#ifdef BMEDEBUG
    DEBUG_PRINTF("Status: 0x%02x %s\n", val_STATUS, ((val_STATUS & STATUS_MEASURING) != 0) ? "busy" : "done");
#endif

    // Wait for the measurement to complete
    if ((val_STATUS & STATUS_MEASURING) != 0) {

        // Delay a bit to allow measurement to proceed
        nrf_delay_ms(20);

        // Retry status
        static app_twi_transfer_t const mtransfers2a[] = {
            APP_TWI_WRITE(BME280_I2C_ADDRESS, &reg_STATUS, sizeof(reg_STATUS), APP_TWI_NO_STOP),
            APP_TWI_READ(BME280_I2C_ADDRESS, &val_STATUS, sizeof(val_STATUS), 0)
        };
        static app_twi_transaction_t const mtransaction2a = {
            .callback            = s_bme280_measure_2,
            .p_user_data         = NULL,
            .p_transfers         = mtransfers2a,
            .number_of_transfers = sizeof(mtransfers2a) / sizeof(mtransfers2a[0])
        };
        err_code = app_twi_schedule(twi_context(), &mtransaction2a);
        if (err_code != NRF_SUCCESS) {
            DEBUG_CHECK(err_code);
            sensor_measurement_completed(sensor);
        }

        return;

    }

    // Initiate a TWI read of the data
    if (initiate_read() != NRF_SUCCESS)
        sensor_measurement_completed(sensor);

}

// Measurement needed?  Say "no" just so as not to trigger an upload just because of this
bool s_bme280_upload_needed(void *s) {
    return(s_bme280_get_value(NULL, NULL, NULL));
}

// Measure temp
void s_bme280_measure(void *s) {
    uint32_t err_code;

#ifdef BMEDEBUG
    DEBUG_PRINTF("BME280 Measure\n");
#endif

    // Exit if never initialized
    if (!fInit) {
        sensor_measurement_completed(sensor);
        return;
    }
    
    // Remember which sensor this pertains to
    sensor = s;

    // Do different things, whether auto or forced
    if (SAMPLING_MODE == MODE_FORCED) {

        // Write the control register again to indicate that we are in forced-conversion mode
        static app_twi_transfer_t const mtransfers2[] = {
            APP_TWI_WRITE(BME280_I2C_ADDRESS, &cmd_C, sizeof(cmd_C), 0),
            APP_TWI_WRITE(BME280_I2C_ADDRESS, &reg_STATUS, sizeof(reg_STATUS), APP_TWI_NO_STOP),
            APP_TWI_READ(BME280_I2C_ADDRESS, &val_STATUS, sizeof(val_STATUS), 0)
        };
        static app_twi_transaction_t const mtransaction2 = {
            .callback            = s_bme280_measure_2,
            .p_user_data         = NULL,
            .p_transfers         = mtransfers2,
            .number_of_transfers = sizeof(mtransfers2) / sizeof(mtransfers2[0])
        };
        err_code = app_twi_schedule(twi_context(), &mtransaction2);
        if (err_code != NRF_SUCCESS) {
            DEBUG_CHECK(err_code);
            sensor_unconfigure(s, err_code);
        }

    } else {

        // Initiate a read of the data
        err_code = initiate_read();
        if (err_code != NRF_SUCCESS)
            sensor_unconfigure(s, err_code);

    }

}

// The main access method for our data
bool s_bme280_get_value(float *tempC, float *humid, float *pressurePa) {
    if (tempC != NULL)
        *tempC = reported_temperature;
    if (humid != NULL)
        *humid = reported_humidity;
    if (pressurePa != NULL)
        *pressurePa = reported_pressure;
    if (!reported)
        return false;
    return true;
}

// Clear it out
void s_bme280_clear_measurement() {
    reported = false;
}

uint8_t u8(int index) {
    return(val_ALL[index-BME280_REGISTER_FIRST]);
}

int8_t s8(int index) {
    return((int8_t)u8(index));
}

#define mu16(x) ((uint16_t)(x))
#define ms16(x) ((int16_t)(x))

uint16_t u16(int index) {
    return (mu16(u8(index))|(mu16(u8(index+1))<<8));
}

int16_t s16(int index) {
    return ((int16_t)u16(index));
}


// Asynchronous continuation
void s_bme280_init_3(ret_code_t result, void *ignore) {

    // Exit if error
    if (result != NRF_SUCCESS) {
        DEBUG_PRINTF("BME280: Init error 3:(%d)\n", result);
        return;
    }
    
    // Extract values as appropriate
    chip_id = u8(BME280_REGISTER_CHIPID);
    version = u8(BME280_REGISTER_VERSION);
    config = u8(BME280_REGISTER_CONFIG);
    status = u8(BME280_REGISTER_STATUS);
    control = u8(BME280_REGISTER_CONTROL);

    dig_T1 = u16(BME280_DIG_T1_REG);
    dig_T2 = s16(BME280_DIG_T2_REG);
    dig_T3 = s16(BME280_DIG_T3_REG);

    dig_P1 = u16(BME280_DIG_P1_REG);
    dig_P2 = s16(BME280_DIG_P2_REG);
    dig_P3 = s16(BME280_DIG_P3_REG);
    dig_P4 = s16(BME280_DIG_P4_REG);
    dig_P5 = s16(BME280_DIG_P5_REG);
    dig_P6 = s16(BME280_DIG_P6_REG);
    dig_P7 = s16(BME280_DIG_P7_REG);
    dig_P8 = s16(BME280_DIG_P8_REG);
    dig_P9 = s16(BME280_DIG_P9_REG);

    dig_H1 = u8(BME280_DIG_H1_REG);
    dig_H2 = s16(BME280_DIG_H2_REG);
    dig_H3 = u8(BME280_DIG_H3_REG);
    dig_H4 = ms16((mu16(u8(BME280_DIG_H4_REG))<<8)|mu16((u8(BME280_DIG_H4_REG+1)&0xF)<<4))>>4;
    dig_H5 = ms16((mu16(u8(BME280_DIG_H5_REG+1))<<8)|mu16((u8(BME280_DIG_H5_REG)&0xF0))>>4);
    dig_H6 = s8(BME280_DIG_H6_REG);

    // Validate
    if (chip_id != VAL_CHIP_ID) {
        DEBUG_PRINTF("Bad chip ID 0x%02x\n", chip_id);
        return;
    }

#ifdef BMEDEBUG
    DEBUG_PRINTF("id=0x%02x v=0x%02x\n", chip_id, version);
    DEBUG_PRINTF("cfg=0x%02x sts=0x%02x ctl=0x%02x\n", config, status, control);
    DEBUG_PRINTF("dig_T1=%d dig_T2=%d\n", dig_T1, dig_T2);
    DEBUG_PRINTF("dig_T3=%d\n", dig_T3);
    DEBUG_PRINTF("dig_P1=%d dig_P2=%d\n", dig_P1, dig_P2);
    DEBUG_PRINTF("dig_P3=%d dig_P4=%d\n", dig_P3, dig_P4);
    DEBUG_PRINTF("dig_P5=%d dig_P6=%d\n", dig_P5, dig_P6);
    DEBUG_PRINTF("dig_P7=%d dig_P8=%d\n", dig_P7, dig_P8);
    DEBUG_PRINTF("dig_P9=%d\n", dig_P9);
    DEBUG_PRINTF("dig_H1=%d dig_H2=%d\n", dig_H1, dig_H2);
    DEBUG_PRINTF("dig_H3=%d dig_H4=%d\n", dig_H3, dig_H4);
    DEBUG_PRINTF("dig_H5=%d dig_H6=%d\n", dig_H5, dig_H6);
#endif

    // Don't fetch it again.
    have_calibration_data = true;

}

// Asynchronous continuation
void s_bme280_init_2(ret_code_t result, void *ignore) {
    uint32_t err_code;

    // Exit if error
    if (result != NRF_SUCCESS) {
        DEBUG_PRINTF("BME280: Init error 2:(%d)\n", result);
        return;
    }
    
    // Exit if we've been here before
    if (have_calibration_data)
        return;

    // Fetch calibration data
    static app_twi_transfer_t const itransfers2[] = {
        APP_TWI_WRITE(BME280_I2C_ADDRESS, &reg_ALL, sizeof(reg_ALL), APP_TWI_NO_STOP),
        APP_TWI_READ(BME280_I2C_ADDRESS, &val_ALL, sizeof(val_ALL), 0)
    };
    static app_twi_transaction_t const itransaction2 = {
        .callback            = s_bme280_init_3,
        .p_user_data         = NULL,
        .p_transfers         = itransfers2,
        .number_of_transfers = sizeof(itransfers2) / sizeof(itransfers2[0])
    };
    err_code = app_twi_schedule(twi_context(), &itransaction2);
    DEBUG_CHECK(err_code);

}

// Init sensor
bool s_bme280_init() {
    uint32_t err_code;

#ifdef BMEDEBUG
    DEBUG_PRINTF("BME280 Init\n");
#endif

    // Initialize TWI
    if (!twi_init())
        return false;
    
    // Initialize last temperature for thermo-compensation
    fine_temp = DEFAULT_FINE_TEMP;

    // Start fresh
    s_bme280_clear_measurement();

    static app_twi_transfer_t const itransfers1[] = {
        APP_TWI_WRITE(BME280_I2C_ADDRESS, &cmd_CFG, sizeof(cmd_CFG), 0),
        APP_TWI_WRITE(BME280_I2C_ADDRESS, &cmd_CH, sizeof(cmd_CH), 0),
        APP_TWI_WRITE(BME280_I2C_ADDRESS, &cmd_C, sizeof(cmd_C), 0),
    };
    static app_twi_transaction_t const itransaction1 = {
        .callback            = s_bme280_init_2,
        .p_user_data         = NULL,
        .p_transfers         = itransfers1,
        .number_of_transfers = sizeof(itransfers1) / sizeof(itransfers1[0])
    };
    err_code = app_twi_schedule(twi_context(), &itransaction1);
    if (err_code != NRF_SUCCESS) {
        DEBUG_PRINTF("TWI error = 0x%04x\n", err_code);
        return false;
    }

    fInit = true;
    return true;
    
}

// Term sensor
bool s_bme280_term() {
    twi_term();
    return true;
}

#endif // TWIBME280
