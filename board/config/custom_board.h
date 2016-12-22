/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */
#ifndef CUSTOM_BOARD_H__
#define CUSTOM_BOARD_H__

#define PIN_UNDEFINED 0xff

#ifdef blenano

// TWI pins
#ifdef TWIX
#define TWI_PIN_SDA 10
#define TWI_PIN_SCL 8
#endif

// UART pins 
#define RX_PIN 11
#define TX_PIN 9
// there is no cts/rts on blenano, but these symbols are required
#define CTS_PIN PIN_UNDEFINED
#define RTS_PIN PIN_UNDEFINED
#define HWFC false

// Geiger Pins
#ifdef GEIGER
#define PIN_GEIGER0 4
#define PIN_GEIGER1 5
#endif

// Power Pins
#define POWER_PIN_GPS 28
#define POWER_PIN_BASICS 7
#endif // BLENANO

#ifdef blueio // I-SYST IBK-BLUEIO Breakout Board with IMM-NRF52832 Module

// TWI pins - note that -DCONFIG_NFCT_PINS_AS_GPIOS is necessary to use these pins on nRF52
#ifdef TWIX
#define TWI_PIN_SDA 25
#define TWI_PIN_SCL 26
#endif

// SPI pins
#ifdef SPIX
#define SPI_PIN_MOSI 3
#define SPI_PIN_MISO 4
#define SPI_PIN_SS_OPC 5
#define SPI_PIN_SCLK 6
#endif

// UART Selector pin
#define UART_SELECT true
#define UART_SELECT0 21
#define UART_SELECT1 22
#define UART_DESELECT 27
#define UART_SELECT_PIN0 0x0001
#define UART_SELECT_PIN1 0x0002
#define UART_SELECT_LORA (0 | 0 )
#define UART_SELECT_CELL (UART_SELECT_PIN0 | 0)
#define UART_SELECT_PMS  (0 | UART_SELECT_PIN1)
#define UART_SELECT_SPARE (UART_SELECT_PIN0 | UART_SELECT_PIN1)

// UART pins - note that we set HWFC to true because
// some of our peripherals DO use it, even though some also don't.
#define RX_PIN 8
#define TX_PIN 7
#define CTS_PIN 12
#define RTS_PIN 11
#define HWFC true
    
// Geiger Pins
#ifdef GEIGER
#define PIN_GEIGER0 28
#define PIN_GEIGER1 29
#endif

// Power Pins
#define POWER_PIN_AIR 13
#define POWER_PIN_GEIGER 14
#define POWER_PIN_BASICS 15
#define POWER_PIN_CELL 16
#define POWER_PIN_LORA 17

// Power Sensing Pin
#define SENSE_POWER_PIN 20

// Motion Sensing pin
#define SENSE_MOTION_PIN 2

// LEDs - note that if you change these you should also update BSP_LED_* in ttboot's custom_board.h
#define LED_COLOR
#define LED_START   23
#define LED_STOP    24
#define LED_PIN_RED LED_START
#define LED_PIN_YEL LED_STOP

// A spare pin that can be used as input for testing/debugging
#define SPARE_PIN 19

// This is ONLY used when we are compiling the bootloader.  Sadly, the NRF bootloader libraries
// assume that there are a few LEDs and a single button.  Since we are buttonless, we can fake this
// (without modifying their code) by picking any pin that we know isn't pulled to zero at boot.
// These are referenced by the Nordic SDK:
//      [sdk-root]/components/libraries/bootloader/nrf_dfu.h
//      [sdk-root]/components/libraries/bootloader/ble_dfu/nrf_ble_dfu.c
#ifdef COMPILING_BOOTLOADER
#define BSP_LED_0       LED_START
#define BSP_LED_1       LED_START
#define BSP_LED_2       LED_STOP
#define BSP_LED_0_MASK (1<<BSP_LED_0)
#define BSP_LED_1_MASK (1<<BSP_LED_1)
#define BSP_LED_2_MASK (1<<BSP_LED_2)
#define LEDS_MASK      ( BSP_LED_0_MASK | BSP_LED_1_MASK | BSP_LED_2_MASK )
#define BSP_BUTTON_3    SPARE_PIN
#define BUTTON_PULL     NRF_GPIO_PIN_PULLUP
#endif

#endif // I-SYST IBK-BLUEIO Breakout Board with IMM-NRF52832 Module

// Low frequency clock source to be used by the SoftDevice
#define NRF_CLOCK_LFCLKSRC      {.source        = NRF_CLOCK_LF_SRC_XTAL,            \
                                 .rc_ctiv       = 0,                                \
                                 .rc_temp_ctiv  = 0,                                \
                                 .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM}

#endif /* CUSTOM_BOARD_H__ */
