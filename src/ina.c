// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

// INA219 Fuel Gauge

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "debug.h"
#include "boards.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_drv_twi.h"
#include "app_util_platform.h"
#include "gpio.h"
#include "config.h"
#include "timer.h"
#include "sensor.h"
#include "storage.h"
#include "comm.h"
#include "misc.h"
#include "twi.h"
#include "ina.h"
#include "io.h"
#include "battery.h"

#define CONFIG_MODE_TRIGGERED   0
#define CONFIG_MODE_CONTINUOUS  1
#define CONFIG_MODE CONFIG_MODE_CONTINUOUS

#ifdef TWIINA219

// Specification
// http://www.ti.com/lit/ds/symlink/ina219.pdf

/*=========================================================================
  I2C ADDRESS/BITS
  -----------------------------------------------------------------------*/
#define INA219_I2C_ADDRESS                     (0x40)    // 1000000 (A0+A1=GND)
#define INA219_READ                            (0x01)
/*=========================================================================*/

/*=========================================================================
  CONFIG REGISTER (R/W)
  -----------------------------------------------------------------------*/
#define INA219_REG_CONFIG                      (0x00)
/*---------------------------------------------------------------------*/
#define INA219_CONFIG_RESET                    (0x8000)  // Reset Bit

#define INA219_CONFIG_BVOLTAGERANGE_MASK       (0x2000)  // Bus Voltage Range Mask
#define INA219_CONFIG_BVOLTAGERANGE_16V        (0x0000)  // 0-16V Range
#define INA219_CONFIG_BVOLTAGERANGE_32V        (0x2000)  // 0-32V Range

#define INA219_CONFIG_GAIN_MASK                (0x1800)  // Gain Mask
#define INA219_CONFIG_GAIN_1_40MV              (0x0000)  // Gain 1, 40mV Range
#define INA219_CONFIG_GAIN_2_80MV              (0x0800)  // Gain 2, 80mV Range
#define INA219_CONFIG_GAIN_4_160MV             (0x1000)  // Gain 4, 160mV Range
#define INA219_CONFIG_GAIN_8_320MV             (0x1800)  // Gain 8, 320mV Range

#define INA219_CONFIG_BADCRES_MASK             (0x0780)  // Bus ADC Resolution Mask
#define INA219_CONFIG_BADCRES_9BIT             (0x0080)  // 9-bit bus res = 0..511
#define INA219_CONFIG_BADCRES_10BIT            (0x0100)  // 10-bit bus res = 0..1023
#define INA219_CONFIG_BADCRES_11BIT            (0x0200)  // 11-bit bus res = 0..2047
#define INA219_CONFIG_BADCRES_12BIT            (0x0400)  // 12-bit bus res = 0..4097

#define INA219_CONFIG_SADCRES_MASK             (0x0078)  // Shunt ADC Resolution and Averaging Mask
#define INA219_CONFIG_SADCRES_9BIT_1S_84US     (0x0000)  // 1 x 9-bit shunt sample
#define INA219_CONFIG_SADCRES_10BIT_1S_148US   (0x0008)  // 1 x 10-bit shunt sample
#define INA219_CONFIG_SADCRES_11BIT_1S_276US   (0x0010)  // 1 x 11-bit shunt sample
#define INA219_CONFIG_SADCRES_12BIT_1S_532US   (0x0018)  // 1 x 12-bit shunt sample
#define INA219_CONFIG_SADCRES_12BIT_2S_1060US  (0x0048)  // 2 x 12-bit shunt samples averaged together
#define INA219_CONFIG_SADCRES_12BIT_4S_2130US  (0x0050)  // 4 x 12-bit shunt samples averaged together
#define INA219_CONFIG_SADCRES_12BIT_8S_4260US  (0x0058)  // 8 x 12-bit shunt samples averaged together
#define INA219_CONFIG_SADCRES_12BIT_16S_8510US (0x0060)  // 16 x 12-bit shunt samples averaged together
#define INA219_CONFIG_SADCRES_12BIT_32S_17MS   (0x0068)  // 32 x 12-bit shunt samples averaged together
#define INA219_CONFIG_SADCRES_12BIT_64S_34MS   (0x0070)  // 64 x 12-bit shunt samples averaged together
#define INA219_CONFIG_SADCRES_12BIT_128S_69MS  (0x0078)  // 128 x 12-bit shunt samples averaged together

#define INA219_CONFIG_MODE_MASK                (0x0007)  // Operating Mode Mask
#define INA219_CONFIG_MODE_POWERDOWN           (0x0000)
#define INA219_CONFIG_MODE_SVOLT_TRIGGERED     (0x0001)
#define INA219_CONFIG_MODE_BVOLT_TRIGGERED     (0x0002)
#define INA219_CONFIG_MODE_SANDBVOLT_TRIGGERED (0x0003)
#define INA219_CONFIG_MODE_ADCOFF              (0x0004)
#define INA219_CONFIG_MODE_SVOLT_CONTINUOUS    (0x0005)
#define INA219_CONFIG_MODE_BVOLT_CONTINUOUS    (0x0006)
#define INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS (0x0007)
/*=========================================================================*/

/*=========================================================================
  SHUNT VOLTAGE REGISTER (R)
  -----------------------------------------------------------------------*/
#define INA219_REG_SHUNTVOLTAGE                (0x01)
/*=========================================================================*/

/*=========================================================================
  BUS VOLTAGE REGISTER (R)
  -----------------------------------------------------------------------*/
#define INA219_REG_BUSVOLTAGE                  (0x02)
/*=========================================================================*/

/*=========================================================================
  POWER REGISTER (R)
  -----------------------------------------------------------------------*/
#define INA219_REG_POWER                       (0x03)
/*=========================================================================*/

/*=========================================================================
  CURRENT REGISTER (R)
  -----------------------------------------------------------------------*/
#define INA219_REG_CURRENT                     (0x04)
/*=========================================================================*/

/*=========================================================================
  CALIBRATION REGISTER (R/W)
  -----------------------------------------------------------------------*/
#define INA219_REG_CALIBRATION                 (0x05)
/*=========================================================================*/


// I/O buffer
typedef struct {
    // Buffers for reading/writing values
    uint8_t buf_config[3];
    uint8_t buf_calibration[3];
    uint8_t buf_shunt_voltage_cmd[1];
    uint8_t buf_shunt_voltage_val[2];
    uint8_t buf_bus_voltage_cmd[1];
    uint8_t buf_bus_voltage_val[2];
    uint8_t buf_current_cmd[1];
    uint8_t buf_current_val[2];
} ina_data_t;
static ina_data_t io;

static bool fTWIInit = false;
static bool reported = false;
static bool ever_reported = false;
static float reported_shunt_voltage = 0.0;
static float reported_bus_voltage = 4.0;
static float reported_load_voltage = 4.0;
static float reported_soc = 100.0;
static float reported_current = 0.0;
static float sampled_shunt_voltage;
static float sampled_bus_voltage;
static float sampled_load_voltage;
static float sampled_current;

static uint16_t ina219_cfgValue;
static uint32_t ina219_calValue;
static uint32_t ina219_currentDivider_mA;
static uint32_t ina219_powerDivider_mW;

#define PWR_SAMPLE_BINS (PWR_SAMPLE_PERIOD_SECONDS/PWR_SAMPLE_SECONDS)
static bool first_sample;
static uint16_t num_samples;
static uint16_t num_current_samples;
static uint16_t num_polls;
static bool measure_on_next_poll = false;

// Configures to INA219 to be able to measure up to 32V and 2A
// of current.  Each unit of current corresponds to 100uA, and
// each unit of power corresponds to 2mW. Counter overflow
// occurs at 3.2A.
// These calculations assume a 0.1 ohm resistor is present.
void ina219_setCalibration_32V_2A(void)
{
    // By default we use a pretty huge range for the input voltage,
    // which probably isn't the most appropriate choice for system
    // that don't use a lot of power.  But all of the calculations
    // are shown below if you want to change the settings.  You will
    // also need to change any relevant register settings, such as
    // setting the VBUS_MAX to 16V instead of 32V, etc.

    // VBUS_MAX = 32V             (Assumes 32V, can also be set to 16V)
    // VSHUNT_MAX = 0.32          (Assumes Gain 8, 320mV, can also be 0.16, 0.08, 0.04)
    // RSHUNT = 0.1               (Resistor value in ohms)

    // 1. Determine max possible current
    // MaxPossible_I = VSHUNT_MAX / RSHUNT
    // MaxPossible_I = 3.2A

    // 2. Determine max expected current
    // MaxExpected_I = 2.0A

    // 3. Calculate possible range of LSBs (Min = 15-bit, Max = 12-bit)
    // MinimumLSB = MaxExpected_I/32767
    // MinimumLSB = 0.000061              (61uA per bit)
    // MaximumLSB = MaxExpected_I/4096
    // MaximumLSB = 0,000488              (488uA per bit)

    // 4. Choose an LSB between the min and max values
    //    (Preferrably a roundish number close to MinLSB)
    // CurrentLSB = 0.0001 (100uA per bit)

    // 5. Compute the calibration register
    // Cal = trunc (0.04096 / (Current_LSB * RSHUNT))
    // Cal = 4096 (0x1000)

    ina219_calValue = 4096;

    // 6. Calculate the power LSB
    // PowerLSB = 20 * CurrentLSB
    // PowerLSB = 0.002 (2mW per bit)

    // 7. Compute the maximum current and shunt voltage values before overflow
    //
    // Max_Current = Current_LSB * 32767
    // Max_Current = 3.2767A before overflow
    //
    // If Max_Current > Max_Possible_I then
    //    Max_Current_Before_Overflow = MaxPossible_I
    // Else
    //    Max_Current_Before_Overflow = Max_Current
    // End If
    //
    // Max_ShuntVoltage = Max_Current_Before_Overflow * RSHUNT
    // Max_ShuntVoltage = 0.32V
    //
    // If Max_ShuntVoltage >= VSHUNT_MAX
    //    Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
    // Else
    //    Max_ShuntVoltage_Before_Overflow = Max_ShuntVoltage
    // End If

    // 8. Compute the Maximum Power
    // MaximumPower = Max_Current_Before_Overflow * VBUS_MAX
    // MaximumPower = 3.2 * 32V
    // MaximumPower = 102.4W

    // Set multipliers to convert raw current/power values
    ina219_currentDivider_mA = 10;  // Current LSB = 100uA per bit (1000/100 = 10)
    ina219_powerDivider_mW = 2;     // Power LSB = 1mW per bit (2/1)

    // Set config to take into account the settings above
    ina219_cfgValue = INA219_CONFIG_BVOLTAGERANGE_32V |
        INA219_CONFIG_GAIN_8_320MV |
        INA219_CONFIG_BADCRES_12BIT |
        INA219_CONFIG_SADCRES_12BIT_1S_532US;

#if (CONFIG_MODE==CONFIG_MODE_TRIGGERED)
    ina219_cfgValue |= INA219_CONFIG_MODE_SANDBVOLT_TRIGGERED;
#else
    ina219_cfgValue |= INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
#endif

}

// Configures to INA219 to be able to measure up to 32V and 1A
// of current.  Each unit of current corresponds to 40uA. Counter
// overflow occurs at 1.3A.
// These calculations assume a 0.1 ohm resistor is present
void ina219_setCalibration_32V_1A(void)
{
    // By default we use a pretty huge range for the input voltage,
    // which probably isn't the most appropriate choice for system
    // that don't use a lot of power.  But all of the calculations
    // are shown below if you want to change the settings.  You will
    // also need to change any relevant register settings, such as
    // setting the VBUS_MAX to 16V instead of 32V, etc.

    // VBUS_MAX = 32V       (Assumes 32V, can also be set to 16V)
    // VSHUNT_MAX = 0.32    (Assumes Gain 8, 320mV, can also be 0.16, 0.08, 0.04)
    // RSHUNT = 0.1         (Resistor value in ohms)

    // 1. Determine max possible current
    // MaxPossible_I = VSHUNT_MAX / RSHUNT
    // MaxPossible_I = 3.2A

    // 2. Determine max expected current
    // MaxExpected_I = 1.0A

    // 3. Calculate possible range of LSBs (Min = 15-bit, Max = 12-bit)
    // MinimumLSB = MaxExpected_I/32767
    // MinimumLSB = 0.0000305             (30.5�A per bit)
    // MaximumLSB = MaxExpected_I/4096
    // MaximumLSB = 0.000244              (244�A per bit)

    // 4. Choose an LSB between the min and max values
    //    (Preferrably a roundish number close to MinLSB)
    // CurrentLSB = 0.0000400 (40�A per bit)

    // 5. Compute the calibration register
    // Cal = trunc (0.04096 / (Current_LSB * RSHUNT))
    // Cal = 10240 (0x2800)

    ina219_calValue = 10240;

    // 6. Calculate the power LSB
    // PowerLSB = 20 * CurrentLSB
    // PowerLSB = 0.0008 (800�W per bit)

    // 7. Compute the maximum current and shunt voltage values before overflow
    //
    // Max_Current = Current_LSB * 32767
    // Max_Current = 1.31068A before overflow
    //
    // If Max_Current > Max_Possible_I then
    //    Max_Current_Before_Overflow = MaxPossible_I
    // Else
    //    Max_Current_Before_Overflow = Max_Current
    // End If
    //
    // ... In this case, we're good though since Max_Current is less than MaxPossible_I
    //
    // Max_ShuntVoltage = Max_Current_Before_Overflow * RSHUNT
    // Max_ShuntVoltage = 0.131068V
    //
    // If Max_ShuntVoltage >= VSHUNT_MAX
    //    Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
    // Else
    //    Max_ShuntVoltage_Before_Overflow = Max_ShuntVoltage
    // End If

    // 8. Compute the Maximum Power
    // MaximumPower = Max_Current_Before_Overflow * VBUS_MAX
    // MaximumPower = 1.31068 * 32V
    // MaximumPower = 41.94176W

    // Set multipliers to convert raw current/power values
    ina219_currentDivider_mA = 25;      // Current LSB = 40uA per bit (1000/40 = 25)
    ina219_powerDivider_mW = 1;         // Power LSB = 800�W per bit

    // Set config to take into account the settings above
    ina219_cfgValue = INA219_CONFIG_BVOLTAGERANGE_32V |
        INA219_CONFIG_GAIN_8_320MV |
        INA219_CONFIG_BADCRES_12BIT |
        INA219_CONFIG_SADCRES_12BIT_1S_532US;

#if (CONFIG_MODE==CONFIG_MODE_TRIGGERED)
    ina219_cfgValue |= INA219_CONFIG_MODE_SANDBVOLT_TRIGGERED;
#else
    ina219_cfgValue |= INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
#endif

}

// Calibration which uses the highest precision for
// current measurement (0.1mA), at the expense of
// only supporting 16V at 400mA max.
void ina219_setCalibration_16V_400mA(void) {

    // VBUS_MAX = 16V
    // VSHUNT_MAX = 0.04          (Assumes Gain 1, 40mV)
    // RSHUNT = 0.1               (Resistor value in ohms)

    // 1. Determine max possible current
    // MaxPossible_I = VSHUNT_MAX / RSHUNT
    // MaxPossible_I = 0.4A

    // 2. Determine max expected current
    // MaxExpected_I = 0.4A

    // 3. Calculate possible range of LSBs (Min = 15-bit, Max = 12-bit)
    // MinimumLSB = MaxExpected_I/32767
    // MinimumLSB = 0.0000122              (12uA per bit)
    // MaximumLSB = MaxExpected_I/4096
    // MaximumLSB = 0.0000977              (98uA per bit)

    // 4. Choose an LSB between the min and max values
    //    (Preferrably a roundish number close to MinLSB)
    // CurrentLSB = 0.00005 (50uA per bit)

    // 5. Compute the calibration register
    // Cal = trunc (0.04096 / (Current_LSB * RSHUNT))
    // Cal = 8192 (0x2000)

    ina219_calValue = 8192;

    // 6. Calculate the power LSB
    // PowerLSB = 20 * CurrentLSB
    // PowerLSB = 0.001 (1mW per bit)

    // 7. Compute the maximum current and shunt voltage values before overflow
    //
    // Max_Current = Current_LSB * 32767
    // Max_Current = 1.63835A before overflow
    //
    // If Max_Current > Max_Possible_I then
    //    Max_Current_Before_Overflow = MaxPossible_I
    // Else
    //    Max_Current_Before_Overflow = Max_Current
    // End If
    //
    // Max_Current_Before_Overflow = MaxPossible_I
    // Max_Current_Before_Overflow = 0.4
    //
    // Max_ShuntVoltage = Max_Current_Before_Overflow * RSHUNT
    // Max_ShuntVoltage = 0.04V
    //
    // If Max_ShuntVoltage >= VSHUNT_MAX
    //    Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
    // Else
    //    Max_ShuntVoltage_Before_Overflow = Max_ShuntVoltage
    // End If
    //
    // Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
    // Max_ShuntVoltage_Before_Overflow = 0.04V

    // 8. Compute the Maximum Power
    // MaximumPower = Max_Current_Before_Overflow * VBUS_MAX
    // MaximumPower = 0.4 * 16V
    // MaximumPower = 6.4W

    // Set multipliers to convert raw current/power values
    ina219_currentDivider_mA = 20;  // Current LSB = 50uA per bit (1000/50 = 20)
    ina219_powerDivider_mW = 1;     // Power LSB = 1mW per bit

    // Set config to take into account the settings above
    ina219_cfgValue = INA219_CONFIG_BVOLTAGERANGE_16V |
        INA219_CONFIG_GAIN_1_40MV |
        INA219_CONFIG_BADCRES_12BIT |
        INA219_CONFIG_SADCRES_12BIT_1S_532US;

#if (CONFIG_MODE==CONFIG_MODE_TRIGGERED)
    ina219_cfgValue |= INA219_CONFIG_MODE_SANDBVOLT_TRIGGERED;
#else
    ina219_cfgValue |= INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
#endif

}

// Callback when TWI data has been read, or timeout
void ina_callback(ret_code_t result, twi_context_t *t) {
    uint16_t value;
    int16_t raw;
    float shunt_voltage;
    float bus_voltage;
    float load_voltage;
    float current;

    // If error, flag that this I/O has been completed.
    if (!twi_completed(t)) {
        sensor_measurement_completed(t->sensor);
        return;
    }

    // If this is the first sample, discard it
    if (num_samples == 0 && first_sample) {
        first_sample = false;
        measure_on_next_poll = true;
        return;
    }

    // Shunt voltage - extract raw, then convert to mV, then V
    value = (io.buf_shunt_voltage_val[0] << 8) | io.buf_shunt_voltage_val[1];
    raw = (int16_t) value;
    shunt_voltage = ((float) (raw * .01)) / 1000;
    // Bus voltage "Shift to the right 3 to drop CNVR and OVF, then multiply by LSB" and convert to V
    value = (io.buf_bus_voltage_val[0] << 8) | io.buf_bus_voltage_val[1];
    raw = (int16_t)((value >> 3) * 4);
    bus_voltage = (float) raw * 0.001;
    // Load voltage
    load_voltage = bus_voltage + abs(shunt_voltage);
    // Current in mA given the config settings
    value = (io.buf_current_val[0] << 8) | io.buf_current_val[1];
    // Convert it in a way such that the sign bit gets extended, because we want to see negative currents
    raw = (int16_t) value;
    current = ((float) raw) / ina219_currentDivider_mA;

    // Do a kludge at low current to try to be more accurate, because it takes about 3mA to actually
    // power the TWI to do the readout.
#define TWI_CURRENT_FUDGE 3.0
    if (current > TWI_CURRENT_FUDGE)
        current -= TWI_CURRENT_FUDGE;

    // Store it into the bin IF AND ONLY IF nobody is currently sucking power on the UART if in oneshot mode
    if (num_samples < PWR_SAMPLE_BINS) {
        sampled_shunt_voltage += shunt_voltage;
        sampled_load_voltage += load_voltage;
        sampled_bus_voltage += bus_voltage;
        if (!comm_oneshot_currently_enabled() || gpio_current_uart() == UART_NONE) {
            sampled_current += current;
            num_current_samples++;
        }
        num_samples++;
    }

    // If we're not done, request another measurement
    if (num_samples < PWR_SAMPLE_BINS) {
        // We should rarely hit this (generally only when the UART is busy with oneshot during init),
        // but this is a protective measure to ensure that we don't just sit here for too long
        // and hold up other sensors from operating
        if (num_polls++ < (PWR_SAMPLE_BINS*3))
            measure_on_next_poll = true;
        else {
            sensor_measurement_completed(t->sensor);
        }
        if (debug(DBG_SENSOR_MAX))
            DEBUG_PRINTF("INA219 %.1fV %.1fmA\n", load_voltage, current);

    } else {

        // Compute the voltage and current
        reported_shunt_voltage = sampled_shunt_voltage / num_samples;
        reported_load_voltage = sampled_load_voltage / num_samples;
        reported_bus_voltage = sampled_bus_voltage / num_samples;
        if (num_current_samples == 0)
            reported_current = 0;
        else
            reported_current = sampled_current / num_current_samples;

        // Since the INA219 doesn't report SOC, compute it.
        reported_soc = battery_soc_from_voltage(reported_load_voltage);

        // When debugging current, just poll continuously
#ifdef CURRENTDEBUG

        // Start over
        s_ina_clear_measurement();
        measure_on_next_poll = true;

#else
        // Done
        reported = ever_reported = true;

        // Tell the sensor package that we retrieved an SOC value, and what it is
        battery_set_soc(reported_soc);

        // Flag that this I/O has been completed.
        sensor_measurement_completed(t->sensor);

#endif  // !CURRENTDEBUG

        if (debug(DBG_SENSOR))
            DEBUG_PRINTF("INA219 reports %.1fV %.1fmA %.1f%%\n",
                         reported_load_voltage, reported_current, reported_soc);

    }

}

// Measurement needed?  Say "no" just so as to ensure continuous re-measuring even if upload pending
// Note that it WILL get uploaded opportunistically whenever something else needs uploading.
bool s_ina_upload_needed(void *s) {
    return false;
}

// Poller continuously re-measures when requested to do so
void s_ina_poll(void *s) {
    if (!sensor_is_polling_valid(s))
        return;
    if (measure_on_next_poll) {
        measure_on_next_poll = false;
        s_ina_measure(s);
    }
}

// Measure voltage
void s_ina_measure(void *s) {
    uint16_t cfg = ina219_cfgValue;
    io.buf_config[0] = INA219_REG_CONFIG;
    io.buf_config[1] = 0xff & (cfg >> 8);
    io.buf_config[2] = 0xff & cfg;
    uint16_t cal = ina219_calValue;
    io.buf_calibration[0] = INA219_REG_CALIBRATION;
    io.buf_calibration[1] = 0xff & (cal >> 8);
    io.buf_calibration[2] = 0xff & cal;
    io.buf_current_cmd[0] = INA219_REG_CURRENT;
    io.buf_shunt_voltage_cmd[0] = INA219_REG_SHUNTVOLTAGE;
    io.buf_bus_voltage_cmd[0] = INA219_REG_BUSVOLTAGE;
    static app_twi_transfer_t const transfers[] = {
        APP_TWI_WRITE(INA219_I2C_ADDRESS, &io.buf_config[0], sizeof(io.buf_config), 0),
        APP_TWI_WRITE(INA219_I2C_ADDRESS, &io.buf_calibration[0], sizeof(io.buf_calibration), 0),
        APP_TWI_WRITE(INA219_I2C_ADDRESS, &io.buf_current_cmd[0], sizeof(io.buf_current_cmd), APP_TWI_NO_STOP),
        APP_TWI_READ(INA219_I2C_ADDRESS, &io.buf_current_val[0], sizeof(io.buf_current_val), 0),
        APP_TWI_WRITE(INA219_I2C_ADDRESS, &io.buf_shunt_voltage_cmd[0], sizeof(io.buf_shunt_voltage_cmd), APP_TWI_NO_STOP),
        APP_TWI_READ(INA219_I2C_ADDRESS, &io.buf_shunt_voltage_val[0], sizeof(io.buf_shunt_voltage_val), 0),
        APP_TWI_WRITE(INA219_I2C_ADDRESS, &io.buf_bus_voltage_cmd[0], sizeof(io.buf_bus_voltage_cmd), APP_TWI_NO_STOP),
        APP_TWI_READ(INA219_I2C_ADDRESS, &io.buf_bus_voltage_val[0], sizeof(io.buf_bus_voltage_val), 0)
    };
    static app_twi_transaction_t const transaction = {
        .callback            = twi_callback,
        .p_user_data         = "INA",
        .p_transfers         = transfers,
        .number_of_transfers = sizeof(transfers) / sizeof(transfers[0])
    };
    if (!twi_schedule(s, ina_callback, &transaction))
        sensor_unconfigure(s);
}

// Show the current value
bool s_ina_show_value(uint32_t when, char *buffer, uint16_t length) {
    static uint32_t last = 0;
    char msg[128];
    if (when == last)
        return false;
    last = when;
    if (ever_reported)
        sprintf(msg, "BAT %.2fV %.0f%% %.0fmA", reported_load_voltage, reported_soc, reported_current);
    else
        sprintf(msg, "BAT not yet measured");
    strlcpy(buffer, msg, length);
    return true;
}

// The main access method for our data
bool s_ina_get_value(float *pLoadVoltage, float *pSOC, float *pCurrent) {
    if (pCurrent != NULL)
        *pCurrent = reported_current;
    if (pLoadVoltage != NULL)
        *pLoadVoltage = reported_load_voltage;
    if (pSOC != NULL)
        *pSOC = reported_soc;
    return (reported);
}

// Clear it out
void s_ina_clear_measurement() {
    reported = false;
    num_polls = 0;
    measure_on_next_poll = false;
    num_samples = 0;
    num_current_samples = 0;
    first_sample = true;
    sampled_shunt_voltage = sampled_bus_voltage = sampled_load_voltage = sampled_current = 0.0;
}

// Init sensor
bool s_ina_init(void *s, uint16_t param) {

    // Init TWI
    if (!twi_init())
        return false;
    fTWIInit = true;

    // Clear the measurement
    s_ina_clear_measurement();

    // Set up for highest precision measurement
    ina219_setCalibration_16V_400mA();

    // Success
    return true;
}

// Term sensor
bool s_ina_term() {
    if (fTWIInit) {
        twi_term();
        fTWIInit = false;
    }
    return true;
}

#endif // TWIINA219
