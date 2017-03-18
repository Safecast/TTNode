// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

// SPI handling

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "debug.h"
#include "boards.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "app_util_platform.h"
#include "nrf_drv_spi.h"
#include "opc.h"
#include "gpio.h"
#include "config.h"
#include "timer.h"
#include "sensor.h"
#include "storage.h"
#include "comm.h"
#include "misc.h"
#include "io.h"
#include "stats.h"

#ifdef SPIX

// SPI Init Params.  Note that we can't use instance 0 because that's used
// by TWI, so we use instance 1 and do this in sdk_config.h
#define SPI_INSTANCE_INDEX  1
static nrf_drv_spi_t m_spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE_INDEX);
static bool fInit = false;

// Get the address of the SPI context structure
nrf_drv_spi_t *spi_context() {
    if (!fInit)
        DEBUG_PRINTF("SPI not initialized!\n");
    return(&m_spi);
}

// One-time initialization of SPI subsystem
bool spi_init() {
    uint32_t err_code;
    uint16_t pin;
    nrf_drv_spi_handler_t handler;

    // Exit if already initialized
    if (fInit) {
        DEBUG_PRINTF("SPI Init: already initialized!\n");
        return true;
    }
    
    if (debug(DBG_SENSOR_MAX))
        DEBUG_PRINTF("SPI Init\n");
    
    // Get OPC setup info
#ifdef SPIOPC
    s_opc_get_spi(&pin, &handler);
#else
    Error: what SPI device??
#endif    

    // Initialize SPI handling.  This code only works
    // for a single slave device and needs to be enhanced
    // if we ever have a second slave.
    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    spi_config.bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;
    spi_config.mode = NRF_DRV_SPI_MODE_1;
    spi_config.frequency = NRF_DRV_SPI_FREQ_1M;
    spi_config.miso_pin = SPI_PIN_MISO;
    spi_config.mosi_pin = SPI_PIN_MOSI;
    spi_config.sck_pin  = SPI_PIN_SCLK;
    spi_config.ss_pin   = pin;
    err_code = nrf_drv_spi_init(&m_spi, &spi_config, handler);
    if (err_code == NRF_SUCCESS)
        fInit = true;
    else {
        DEBUG_PRINTF("SPI init error = 0x%04x\n", err_code);
        fInit = false;
        stats()->errors_spi++;
    }

    return fInit;

}

// Terminate SPI
bool spi_term() {

    // Exit if already ununinitialized
    if (!fInit)
        return false;
    
    if (debug(DBG_SENSOR_MAX))
        DEBUG_PRINTF("SPI Term\n");

    // Uninitialize SPI, to stop the current drain on the about-to-be powered-off device
    nrf_drv_spi_uninit(&m_spi);

    // Clear the select pin, because THIS IS THE CAUSE of a 7ma drain and we can safely do this
    // once the SPI subsystem no longer owns the pin.
    gpio_pin_set(SPI_PIN_SS_OPC, false);
    gpio_cfg_input(SPI_PIN_SS_OPC);

    // Done
    fInit = false;

    return true;
}

#endif // SPIX
