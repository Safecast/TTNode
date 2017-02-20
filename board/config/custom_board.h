// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

#ifndef CUSTOM_BOARD_H__
#define CUSTOM_BOARD_H__

////////
///
/// REDBEAR BLE NANO
///
///////

#ifdef blenano

#include "app_uart.h"   // for UART_PIN_DISCONNECTED

// Geiger Pins
#ifdef GEIGERX
#define PIN_GEIGER0 4
#define PIN_GEIGER1 5
#endif

// TWI pins
#ifdef TWIX
#define TWI_PIN_SCL 8
#define TWI_PIN_SDA 10
#endif

// UART pins 
#define TX_PIN 9
#define RX_PIN 11
// there is no cts/rts on blenano, but these symbols are required
#define CTS_PIN UART_PIN_DISCONNECTED
#define RTS_PIN UART_PIN_DISCONNECTED
#define HWFC false

// Power Pins
#define POWER_PIN_GPS 28
#define POWER_PIN_BASICS 7

#endif // BLENANO

////////
///
/// I-SYST IBK-BLUEIO Breakout Board with  IMM-NRF52832 Module
///
///////

#ifdef blueio

#ifdef TWIX
#define TWI_PIN_SDA 25
#define TWI_PIN_SCL 26
#endif

#ifdef SPIX
#define SPI_PIN_MOSI 3
#define SPI_PIN_MISO 4
#define SPI_PIN_SS_OPC 5
#define SPI_PIN_SCLK 6
#endif

#define UART_SELECT true
#define UART_SELECT0 21
#define UART_SELECT1 22
#define UART_DESELECT 27
#define UART_SELECT_PIN0 0x0001
#define UART_SELECT_PIN1 0x0002
#define UART_SELECT_LORA (0 | 0 )
#define UART_SELECT_CELL (UART_SELECT_PIN0 | 0)
#define UART_SELECT_PMS  (0 | UART_SELECT_PIN1)
#define UART_SELECT_GPS (UART_SELECT_PIN0 | UART_SELECT_PIN1)

#define RX_PIN 8
#define TX_PIN 7

#ifndef HWFC
#define HWFC true
#endif
#ifdef HWFC
#define CTS_PIN 12
#define RTS_PIN 11
#endif
 
#ifdef GEIGERX
#define PIN_GEIGER0 28
#define PIN_GEIGER1 29
#endif

#define POWER_PIN_AIR 13
#define POWER_PIN_GEIGER 14
#define POWER_PIN_BASICS 15
#define POWER_PIN_CELL 16
#define POWER_PIN_LORA 17
#define POWER_PIN_GPS 18
#define POWER_PIN_ROCK 19
#define POWER_PIN_PS_BATTERY 20
#define POWER_PIN_PS_5V 10    

// Sensing pins
#define SENSE_PIN_MOTION 2
#define SENSE_PIN_OVERCURRENT 30
#define SENSE_BOOTLOADER_DEBUG SENSE_PIN_MOTION

// LEDs - note that if you change these you should also update BSP_LED_* in ttboot's custom_board.h
#define LED_COLOR
#define LED_START   23
#define LED_STOP    24
#define LED_PIN_RED LED_START
#define LED_PIN_YEL LED_STOP

// This is ONLY used when we are compiling the bootloader.  Sadly, the NRF bootloader libraries
// assume that there are a few LEDs and a single button.  Since we are buttonless, we can fake this
// (without modifying their code) by picking any pin that we know isn't pulled to zero at boot.
// These are referenced by the Nordic SDK:
//      [sdk-root]/components/libraries/bootloader/nrf_dfu.h
//      [sdk-root]/components/libraries/bootloader/ble_dfu/nrf_ble_dfu.c
#ifdef BLE_BOOTLOADER
#define BSP_LED_0       LED_START
#define BSP_LED_1       LED_START
#define BSP_LED_2       LED_STOP
#define BSP_LED_0_MASK (1<<BSP_LED_0)
#define BSP_LED_1_MASK (1<<BSP_LED_1)
#define BSP_LED_2_MASK (1<<BSP_LED_2)
#define LEDS_MASK      ( BSP_LED_0_MASK | BSP_LED_1_MASK | BSP_LED_2_MASK )
// Because the SDK is hard-wired to use BSP_BUTTON_3 for BOOTLOADER_BUTTON
#define BSP_BUTTON_3    SENSE_BOOTLOADER_DEBUG
#define BUTTON_PULL     NRF_GPIO_PIN_PULLUP
#endif

#endif // I-SYST IBK-BLUEIO Breakout Board with IMM-NRF52832 Module

// Low frequency clock source to be used by the SoftDevice
#define NRF_CLOCK_LFCLKSRC      {.source        = NRF_CLOCK_LF_SRC_XTAL,            \
                                 .rc_ctiv       = 0,                                \
                                 .rc_temp_ctiv  = 0,                                \
                                 .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM}

#endif /* CUSTOM_BOARD_H__ */
