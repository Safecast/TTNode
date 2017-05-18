#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include "debug.h"
#include "config.h"
#include "nrf_delay.h"
#include "nordic_common.h"
#include "ssd.h"
#include "app_error.h"
#include "app_util_platform.h"
#include "glcdfont.c"
#include "twi.h"
#include "sensor.h"
#include "comm.h"

#ifdef SSD

// A useful hardware description can be found at:
// https://robotcantalk.blogspot.com/2015/03/interfacing-arduino-with-ssd1306-driven.html
// Also note that on the solarcast board, when looking at the board with the writing
// right-side up, the 4 U9 pins (just above the U7 writing) are, left to right,
// GND 3V3 SDA SCL where 3V3 is switched by POWER_PIN_TWI

// Address for 128x32 is 0x3C
// Address for 128x64 is 0x3D (default) or 0x3C (if SA0 is grounded)
#define SSD1306_I2C_ADDRESS   (0x3C)    // 011110+SA0+RW - 0x3C or 0x3D

// The driver is used only for a 128x64 display.
#define SSD1306_LCDWIDTH    128
#define SSD1306_LCDHEIGHT   64

// First byte transmitted is always command/data
#define CMDSTR  0x00
#define CMDBYTE 0x80
#define DATSTR  0x40
#define DATBYTE 0xC0

// Data transfer buffer
#define DATCHUNK    16
#define DATCMDCHUNK (1+DATCHUNK)
#define DATCHUNKS ((SSD1306_LCDHEIGHT * SSD1306_LCDWIDTH / 8) / DATCHUNK)
static uint8_t data[DATCHUNKS * DATCMDCHUNK];

// VCC defines
#define SSD1306_EXTERNALVCC 0x1
#define SSD1306_SWITCHCAPVCC 0x2
#define SSD1306_VCC SSD1306_SWITCHCAPVCC

// Commands
#define SSD1306_SETCONTRAST 0x81
#define SSD1306_SETCONTRAST_OFFSET 2
static uint8_t setcontrast[3] = {CMDSTR, SSD1306_SETCONTRAST, 0xFF};
#define SSD1306_DISPLAYALLON_RESUME 0xA4
static uint8_t displayallon_resume[2] = {CMDSTR, SSD1306_DISPLAYALLON_RESUME};
#define SSD1306_DISPLAYALLON 0xA5
// DISPLAYALLON unused
#define SSD1306_NORMALDISPLAY 0xA6
static uint8_t normaldisplay[2] = {CMDSTR, SSD1306_NORMALDISPLAY};
#define SSD1306_INVERTDISPLAY 0xA7
static uint8_t invertdisplay[2] = {CMDSTR, SSD1306_INVERTDISPLAY};
#define SSD1306_DISPLAYOFF 0xAE
static uint8_t displayoff[2] = {CMDSTR, SSD1306_DISPLAYOFF};
#define SSD1306_DISPLAYON 0xAF
static uint8_t displayon[2] = {CMDSTR, SSD1306_DISPLAYON};
#define SSD1306_SETDISPLAYOFFSET 0xD3
static uint8_t setdisplayoffset[3] = {CMDSTR, SSD1306_SETDISPLAYOFFSET, 0};
#define SSD1306_SETCOMPINS 0xDA
static uint8_t setcompins[3] = {CMDSTR, SSD1306_SETCOMPINS, 0x12};
#define SSD1306_SETVCOMDETECT 0xDB
static uint8_t setvcomdetect[3] = {CMDSTR, SSD1306_SETVCOMDETECT, 0x40};
#define SSD1306_SETDISPLAYCLOCKDIV 0xD5
static uint8_t setdisplayclockdiv[3] = {CMDSTR, SSD1306_SETDISPLAYCLOCKDIV, 0x80};
#define SSD1306_SETPRECHARGE 0xD9
#if (SSD1306_VCC == SSD1306_EXTERNALVCC)
static uint8_t setprecharge[3] = {CMDSTR, SSD1306_SETPRECHARGE, 0x22};
#else
static uint8_t setprecharge[3] = {CMDSTR, SSD1306_SETPRECHARGE, 0xF1};
#endif
#define SSD1306_SETMULTIPLEX 0xA8
static uint8_t setmultiplex[3] = {CMDSTR, SSD1306_SETMULTIPLEX, 0x3F};
#define SSD1306_SETLOWCOLUMN 0x00
// SETLOWCOLUMN unused
#define SSD1306_SETHIGHCOLUMN 0x10
// SETHIGHCOLUMN unused
#define SSD1306_SETSTARTLINE 0x40
static uint8_t setstartline0[2] = {CMDSTR, SSD1306_SETSTARTLINE | 0x00};
#define SSD1306_MEMORYMODE 0x20
static uint8_t memorymode[4] = {CMDSTR, SSD1306_MEMORYMODE, 0x00, 0x00};        // 0x00 act like ks0108
#define SSD1306_COLUMNADDR 0x21
static uint8_t columnaddr[4] = {CMDSTR, SSD1306_COLUMNADDR, 0, SSD1306_LCDWIDTH-1};
#define SSD1306_PAGEADDR   0x22
static uint8_t pageaddr[4] = {CMDSTR, SSD1306_PAGEADDR, 0, 7};                  // 0=reset, 7=page-end-address
#define SSD1306_COMSCANINC 0xC0
//static uint8_t comscaninc[2] = {CMDSTR, SSD1306_COMSCANINC};
#define SSD1306_COMSCANDEC 0xC8
static uint8_t comscandec[2] = {CMDSTR, SSD1306_COMSCANDEC};
#define SSD1306_SEGREMAP   0xA0
static uint8_t segremap1[2] = {CMDSTR, SSD1306_SEGREMAP | 0x01};
#define SSD1306_CHARGEPUMP 0x8D
#if (SSD1306_VCC == SSD1306_EXTERNALVCC)
static uint8_t chargepump[3] = {CMDSTR, SSD1306_CHARGEPUMP, 0x10};
#else
static uint8_t chargepump[3] = {CMDSTR, SSD1306_CHARGEPUMP, 0x14};
#endif
// Scrolling #defines
#define SSD1306_ACTIVATE_SCROLL 0x2F
static uint8_t activate_scroll[2] = {CMDSTR, SSD1306_ACTIVATE_SCROLL};
#define SSD1306_DEACTIVATE_SCROLL 0x2E
static uint8_t deactivate_scroll[2] = {CMDSTR, SSD1306_DEACTIVATE_SCROLL};
#define SSD1306_SET_VERTICAL_SCROLL_AREA 0xA3
static uint8_t set_vertical_scroll_area[4] = {CMDSTR, SSD1306_SET_VERTICAL_SCROLL_AREA, 0x00, SSD1306_LCDHEIGHT};
#define SSD1306_RIGHT_HORIZONTAL_SCROLL 0x26
#define SSD1306_RIGHT_HORIZONTAL_SCROLL_START_OFFSET 3
#define SSD1306_RIGHT_HORIZONTAL_SCROLL_STOP_OFFSET 5
static uint8_t right_horizontal_scroll[8] = {CMDSTR, SSD1306_RIGHT_HORIZONTAL_SCROLL, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF};
#define SSD1306_LEFT_HORIZONTAL_SCROLL 0x27
#define SSD1306_LEFT_HORIZONTAL_SCROLL_START_OFFSET 3
#define SSD1306_LEFT_HORIZONTAL_SCROLL_STOP_OFFSET 5
static uint8_t left_horizontal_scroll[8] = {CMDSTR, SSD1306_LEFT_HORIZONTAL_SCROLL, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF};
#define SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL 0x29
#define SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL_START_OFFSET 3
#define SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL_STOP_OFFSET 5
static uint8_t vertical_and_right_horizontal_scroll[7] = {CMDSTR, SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL, 0x00, 0xFF, 0x00, 0xFF, 0x01};
#define SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL 0x2A
#define SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL_START_OFFSET 3
#define SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL_STOP_OFFSET 5
static uint8_t vertical_and_left_horizontal_scroll[7] = {CMDSTR, SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL, 0x00, 0xFF, 0x00, 0xFF, 0x01};

// Statics
static int16_t _width, _height, WIDTH, HEIGHT, cursor_x, cursor_y;
static uint8_t textsize, rotation, lineheight, charwidth;
static uint16_t textcolor, textbgcolor;
static bool wrap,   // If set, 'wrap' text at right edge of display
    _cp437; // If set, use correct CP437 charset (default is off)
static bool twiinit = false;
static bool display_initialized = false;
static bool display_reinit_requested = false;
static bool display_reinit_in_progress = false;
static int InitCount = 0;
static bool display_needed = false;
static bool display_deferred = false;
static bool display_in_progress = false;
static bool wrap_on_next_char = false;
static bool wrap_prefix = false;

// the memory buffer for the LCD
static uint8_t buffer[SSD1306_LCDHEIGHT * SSD1306_LCDWIDTH / 8] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80,
    0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x80, 0x80, 0xC0, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x80, 0xC0, 0xE0, 0xF0, 0xF8, 0xFC, 0xF8, 0xE0, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x80,
    0x80, 0x80, 0x00, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00, 0xFF,
    0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x80, 0x80, 0x00, 0x00,
    0x80, 0xFF, 0xFF, 0x80, 0x80, 0x00, 0x80, 0x80, 0x00, 0x80, 0x80, 0x80, 0x80, 0x00, 0x80, 0x80,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x00, 0x00, 0x8C, 0x8E, 0x84, 0x00, 0x00, 0x80, 0xF8,
    0xF8, 0xF8, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xE0, 0xE0, 0xC0, 0x80,
    0x00, 0xE0, 0xFC, 0xFE, 0xFF, 0xFF, 0xFF, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0xFF, 0xC7, 0x01, 0x01,
    0x01, 0x01, 0x83, 0xFF, 0xFF, 0x00, 0x00, 0x7C, 0xFE, 0xC7, 0x01, 0x01, 0x01, 0x01, 0x83, 0xFF,
    0xFF, 0xFF, 0x00, 0x38, 0xFE, 0xC7, 0x83, 0x01, 0x01, 0x01, 0x83, 0xC7, 0xFF, 0xFF, 0x00, 0x00,
    0x01, 0xFF, 0xFF, 0x01, 0x01, 0x00, 0xFF, 0xFF, 0x07, 0x01, 0x01, 0x01, 0x00, 0x00, 0x7F, 0xFF,
    0x80, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x7F, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x01, 0xFF,
    0xFF, 0xFF, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x03, 0x0F, 0x3F, 0x7F, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xE7, 0xC7, 0xC7, 0x8F,
    0x8F, 0x9F, 0xBF, 0xFF, 0xFF, 0xC3, 0xC0, 0xF0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC, 0xFC, 0xFC,
    0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xF8, 0xF8, 0xF0, 0xF0, 0xE0, 0xC0, 0x00, 0x01, 0x03, 0x03, 0x03,
    0x03, 0x03, 0x01, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x03, 0x03, 0x03, 0x01, 0x01,
    0x03, 0x01, 0x00, 0x00, 0x00, 0x01, 0x03, 0x03, 0x03, 0x03, 0x01, 0x01, 0x03, 0x03, 0x00, 0x00,
    0x00, 0x03, 0x03, 0x00, 0x00, 0x00, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
    0x03, 0x03, 0x03, 0x03, 0x03, 0x01, 0x00, 0x00, 0x00, 0x01, 0x03, 0x01, 0x00, 0x00, 0x00, 0x03,
    0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x80, 0xC0, 0xE0, 0xF0, 0xF9, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x3F, 0x1F, 0x0F,
    0x87, 0xC7, 0xF7, 0xFF, 0xFF, 0x1F, 0x1F, 0x3D, 0xFC, 0xF8, 0xF8, 0xF8, 0xF8, 0x7C, 0x7D, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0x3F, 0x0F, 0x07, 0x00, 0x30, 0x30, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0xFE, 0xFE, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0xC0, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x30, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0xC0, 0xFE, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0x7F, 0x3F, 0x1F,
    0x0F, 0x07, 0x1F, 0x7F, 0xFF, 0xFF, 0xF8, 0xF8, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0xF8, 0xE0,
    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0xFE, 0x00, 0x00,
    0x00, 0xFC, 0xFE, 0xFC, 0x0C, 0x06, 0x06, 0x0E, 0xFC, 0xF8, 0x00, 0x00, 0xF0, 0xF8, 0x1C, 0x0E,
    0x06, 0x06, 0x06, 0x0C, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0xFE, 0xFE, 0x00, 0x00, 0x00, 0x00, 0xFC,
    0xFE, 0xFC, 0x00, 0x18, 0x3C, 0x7E, 0x66, 0xE6, 0xCE, 0x84, 0x00, 0x00, 0x06, 0xFF, 0xFF, 0x06,
    0x06, 0xFC, 0xFE, 0xFC, 0x0C, 0x06, 0x06, 0x06, 0x00, 0x00, 0xFE, 0xFE, 0x00, 0x00, 0xC0, 0xF8,
    0xFC, 0x4E, 0x46, 0x46, 0x46, 0x4E, 0x7C, 0x78, 0x40, 0x18, 0x3C, 0x76, 0xE6, 0xCE, 0xCC, 0x80,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x01, 0x07, 0x0F, 0x1F, 0x1F, 0x3F, 0x3F, 0x3F, 0x3F, 0x1F, 0x0F, 0x03,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x0F, 0x00, 0x00,
    0x00, 0x0F, 0x0F, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x0F, 0x00, 0x00, 0x03, 0x07, 0x0E, 0x0C,
    0x18, 0x18, 0x0C, 0x06, 0x0F, 0x0F, 0x0F, 0x00, 0x00, 0x01, 0x0F, 0x0E, 0x0C, 0x18, 0x0C, 0x0F,
    0x07, 0x01, 0x00, 0x04, 0x0E, 0x0C, 0x18, 0x0C, 0x0F, 0x07, 0x00, 0x00, 0x00, 0x0F, 0x0F, 0x00,
    0x00, 0x0F, 0x0F, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x0F, 0x00, 0x00, 0x00, 0x07,
    0x07, 0x0C, 0x0C, 0x18, 0x1C, 0x0C, 0x06, 0x06, 0x00, 0x04, 0x0E, 0x0C, 0x18, 0x0C, 0x0F, 0x07,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// Forwards
void draw_fast_hline_internal(int16_t x, int16_t y, int16_t w, uint16_t color);
void draw_fast_vline_internal(int16_t x, int16_t __y, int16_t __h, uint16_t color);
void draw_circle_helper(int16_t x0, int16_t y0, int16_t r, uint8_t cornername, uint16_t color);
void fill_circle_helper(int16_t x0, int16_t y0, int16_t r, uint8_t cornername, int16_t delta, uint16_t color);
void ssd1306_complete_reset();

// Utility functions
#define ssd1306_swap(a, b) { int16_t t = a; a = b; b = t; }
#define adagfxswap(a, b)   { int16_t t = a; a = b; b = t; }

// Return the size of the display (per current rotation)
int16_t ssd1306_width(void) {
    return _width;
}
int16_t ssd1306_height(void) {
    return _height;
}

// Rotate the screen
void set_rotation(uint8_t x) {
    rotation = (x & 3);
    switch (rotation) {
    case 0:
    case 2:
        _width  = WIDTH;
        _height = HEIGHT;
        break;
    case 1:
    case 3:
        _width  = HEIGHT;
        _height = WIDTH;
        break;
    }
}

// Callback for all TWI I/O
void ssd_twi_callback(ret_code_t result, twi_context_t *t) {

    // If error, flag that this I/O has been completed.
    if (!twi_completed(t)) {
        return;
    }

    // Done

}

// Callback for JUST ssd_init
void ssd_init_callback(ret_code_t result, twi_context_t *t) {

    // If error, flag that this I/O has been completed.
    if (!twi_completed(t)) {
        DEBUG_PRINTF("Display not present (%d)\n", result);
        return;
    }

    // We're done
    display_reinit_in_progress = false;
    display_reinit_requested = false;
    display_initialized = true;
    display_in_progress = false;
    display_deferred = false;
    
    // Now that reinit is no longer in progress, continue the reset
    ssd1306_complete_reset();

}

// Callback for display TWI I/O
void ssd_display_callback(ret_code_t result, twi_context_t *t) {

    // Mark that we are no longer processing the display stuff
    display_in_progress = false;

    // If error, flag that this I/O has been completed.
    if (!twi_completed(t))
        return;

    // If we had deferred the display I/O, initiate another
    if (display_deferred) {
        display_deferred = false;
        ssd1306_display();
    }

    // Done

}

// Process a display reinit request
bool ssd1306_reinit_in_progress() {

    // Exit if reinit already in progress
    if (display_reinit_in_progress)
        return true;
    
    // Exit if no reinit requested
    if (!display_reinit_requested)
        return false;

    // Mark display reinit as currently in-progress
    display_reinit_in_progress = true;
    display_in_progress = false;
    display_deferred = false;
    display_initialized = false;

    // Reinitialize
#if (SSD1306_VCC == SSD1306_EXTERNALVCC)
    setcontrast[SSD1306_SETCONTRAST_OFFSET] = 0x9F;
#else
    setcontrast[SSD1306_SETCONTRAST_OFFSET] = 0xCF;
#endif
    static app_twi_transfer_t const transfers[] = {
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, displayoff, sizeof(displayoff), 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, setdisplayclockdiv, sizeof(setdisplayclockdiv), 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, setmultiplex, sizeof(setmultiplex), 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, setdisplayoffset, sizeof(setdisplayoffset), 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, setstartline0, sizeof(setstartline0), 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, chargepump, sizeof(chargepump), 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, memorymode, sizeof(memorymode), 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, segremap1, sizeof(segremap1), 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, comscandec, sizeof(comscandec), 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, setcompins, sizeof(setcompins), 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, setcontrast, sizeof(setcontrast), 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, setprecharge, sizeof(setprecharge), 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, setvcomdetect, sizeof(setvcomdetect), 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, displayallon_resume, sizeof(displayallon_resume), 0),
#ifdef SSD_INVERT
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, invertdisplay, sizeof(invertdisplay), 0),
#else
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, normaldisplay, sizeof(normaldisplay), 0),
#endif
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, displayon, sizeof(displayon), 0)
    };
    static app_twi_transaction_t const transaction = {
        .callback            = twi_callback,
        .p_user_data         = "~SSD-INIT",
        .p_transfers         = transfers,
        .number_of_transfers = sizeof(transfers) / sizeof(transfers[0])
    };

    // Schedule the TWI transactions
    if (!twi_schedule(NULL, ssd_init_callback, &transaction)) {
        display_reinit_in_progress = false;
        display_reinit_requested = true;
    }

    // Done
    return true;

}

// Reset the display to a fresh state
void ssd1306_reset_display() {
    display_reinit_requested = true;
    ssd1306_reinit_in_progress();
}

void ssd1306_complete_reset() {
    if (ssd1306_reinit_in_progress())
        return;
    ssd1306_clear_display();
    DEBUG_PRINTF("SAFECAST SOLARCAST\n");
    DEBUG_PRINTF("%s\n", comm_connect_state());
    sensor_show_values(true);
    DEBUG_PRINTF("\n");
    ssd1306_display();
}

// See if we're already initialized
bool ssd1306_active() {
    return (InitCount > 0);
}

// Force reinitialization
void ssd1306_force_reset() {
    InitCount = 0;
}

// Initialize
bool ssd1306_init() {

    if (InitCount++ > 0) {
        if (debug(DBG_SENSOR_MAX))
            DEBUG_PRINTF("SSD Init nested, now %d users\n", InitCount);
        return true;
    }

    DEBUG_PRINTF("SSD Init\n");

    textsize  = 1;
    lineheight = 8;
    charwidth = 6;
    textcolor = textbgcolor = SSD1306_WHITE;
    wrap      = true;
    _cp437    = false;
    _width = WIDTH = SSD1306_LCDWIDTH;
    _height = HEIGHT = SSD1306_LCDHEIGHT;
#ifdef SSD_UPSIDE_DOWN
    rotation  = 2;
#else
    rotation  = 0;
#endif
    ssd1306_set_cursor(0, 0);
    memset(buffer, 0, (SSD1306_LCDWIDTH * SSD1306_LCDHEIGHT / 8));

    // Init TWI
    if (!twi_init()) {
        InitCount--;
        return false;
    }
    twiinit = true;

    // Give the display to initialize after the power
    // has been applied, because we've found that a number of units
    // will display garbage unless they are given sufficient time
    // to initialize themselves before taking the first TWI commands.
    nrf_delay_ms(250);

    // Request a display reinit
    display_reinit_requested = false;
    display_reinit_in_progress = false;
    display_in_progress = false;
    display_deferred = false;
    display_initialized = false;
    ssd1306_reset_display();

    // Done
    return true;
}

// Terminate
bool ssd1306_term() {

    // Just defensive programming
    if (InitCount <= 0) {
        InitCount = 0;
        return false;
    }

    if (--InitCount == 0) {

        if (twiinit && !twi_one_user()) {

            // Schedule a TWI transaction to turn off the display.
            static app_twi_transfer_t const transfers[] = {
                APP_TWI_WRITE(SSD1306_I2C_ADDRESS, displayoff, sizeof(displayoff), 0)
            };
            static app_twi_transaction_t const transaction = {
                .callback            = twi_callback,
                .p_user_data         = "~SSD-TERM",
                .p_transfers         = transfers,
                .number_of_transfers = sizeof(transfers) / sizeof(transfers[0])
            };
            twi_schedule(NULL, ssd_twi_callback, &transaction);

        }

        // Terminate TWI
        if (twiinit) {
            twi_term();
            twiinit = false;
        }

        display_initialized = false;

        DEBUG_PRINTF("SSD Term\n");

    } else {

        DEBUG_PRINTF("SSD Term nested, %d remaining\n", InitCount);

    }

    return true;

}

// Set the flag indicating that display is needed, and pause if this
// is the transition from 0 to 1 just to force a delay between prior
// TWI commands and the request to display.
void ssd1306_display_needed(void) {
    if (!display_needed)
        nrf_delay_ms(250);
    display_needed = true;
}

// the most basic function, set a single pixel
void ssd1306_draw_pixel(int16_t x, int16_t y, uint16_t color) {

    if ((x < 0) || (x >= ssd1306_width()) || (y < 0) || (y >= ssd1306_height()))
        return;

    // check rotation, move pixel around if necessary
    switch (rotation) {
    case 1:
        ssd1306_swap(x, y);
        x = WIDTH - x - 1;
        break;
    case 2:
        x = WIDTH - x - 1;
        y = HEIGHT - y - 1;
        break;
    case 3:
        ssd1306_swap(x, y);
        y = HEIGHT - y - 1;
        break;
    }

    // x is which column
    switch (color) {
    case SSD1306_WHITE:
        buffer[x + (y / 8)*SSD1306_LCDWIDTH] |=  (1 << (y & 7));
        break;
    case SSD1306_BLACK:
        buffer[x + (y / 8)*SSD1306_LCDWIDTH] &= ~(1 << (y & 7));
        break;
    case SSD1306_INVERSE:
        buffer[x + (y / 8)*SSD1306_LCDWIDTH] ^=  (1 << (y & 7));
        break;
    }

    // Mark display as needing refresh
    ssd1306_display_needed();

}

// the most basic function, set a single pixel
uint16_t ssd1306_get_pixel(int16_t x, int16_t y) {

    if ((x < 0) || (x >= ssd1306_width()) || (y < 0) || (y >= ssd1306_height()))
        return SSD1306_BLACK;

    // check rotation, move pixel around if necessary
    switch (rotation) {
    case 1:
        ssd1306_swap(x, y);
        x = WIDTH - x - 1;
        break;
    case 2:
        x = WIDTH - x - 1;
        y = HEIGHT - y - 1;
        break;
    case 3:
        ssd1306_swap(x, y);
        y = HEIGHT - y - 1;
        break;
    }

    if ((buffer[x + (y / 8)*SSD1306_LCDWIDTH] &  (1 << (y & 7))) == 0)
        return SSD1306_BLACK;

    return SSD1306_WHITE;

}

void ssd1306_invert_display(bool fInvert) {
    static app_twi_transfer_t const itransfers[] = {
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, invertdisplay, sizeof(invertdisplay), 0)
    };
    static app_twi_transaction_t const itransaction = {
        .callback            = twi_callback,
        .p_user_data         = "~SSD-INVI",
        .p_transfers         = itransfers,
        .number_of_transfers = sizeof(itransfers) / sizeof(itransfers[0])
    };
    static app_twi_transfer_t const ntransfers[] = {
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, normaldisplay, sizeof(normaldisplay), 0)
    };
    static app_twi_transaction_t const ntransaction = {
        .callback            = twi_callback,
        .p_user_data         = "~SSD-INVN",
        .p_transfers         = ntransfers,
        .number_of_transfers = sizeof(ntransfers) / sizeof(ntransfers[0])
    };
    if (!twiinit)
        return;
    twi_schedule(NULL, ssd_twi_callback, fInvert ? &itransaction : &ntransaction);
}


// startscrollright
// Activate a right handed scroll for rows start through stop
// Hint, the display is 16 rows tall. To scroll the whole display, run:
// display.scrollright(0x00, 0x0F)
void ssd1306_start_scroll_right(uint8_t start, uint8_t stop) {
    right_horizontal_scroll[SSD1306_RIGHT_HORIZONTAL_SCROLL_START_OFFSET] = start;
    right_horizontal_scroll[SSD1306_RIGHT_HORIZONTAL_SCROLL_STOP_OFFSET] = stop;
    static app_twi_transfer_t const transfers[] = {
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, normaldisplay, sizeof(normaldisplay), 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, right_horizontal_scroll, sizeof(right_horizontal_scroll), 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, activate_scroll, sizeof(activate_scroll), 0)
    };
    static app_twi_transaction_t const transaction = {
        .callback            = twi_callback,
        .p_user_data         = "~SSD-SR",
        .p_transfers         = transfers,
        .number_of_transfers = sizeof(transfers) / sizeof(transfers[0])
    };
    if (!twiinit)
        return;
    twi_schedule(NULL, ssd_twi_callback, &transaction);
}

// startscrollleft
// Activate a right handed scroll for rows start through stop
// Hint, the display is 16 rows tall. To scroll the whole display, run:
// display.scrollright(0x00, 0x0F)
void ssd1306_start_scroll_left(uint8_t start, uint8_t stop) {
    left_horizontal_scroll[SSD1306_LEFT_HORIZONTAL_SCROLL_START_OFFSET] = start;
    left_horizontal_scroll[SSD1306_LEFT_HORIZONTAL_SCROLL_STOP_OFFSET] = stop;
    static app_twi_transfer_t const transfers[] = {
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, normaldisplay, sizeof(normaldisplay), 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, left_horizontal_scroll, sizeof(left_horizontal_scroll), 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, activate_scroll, sizeof(activate_scroll), 0)
    };
    static app_twi_transaction_t const transaction = {
        .callback            = twi_callback,
        .p_user_data         = "~SSD-SL",
        .p_transfers         = transfers,
        .number_of_transfers = sizeof(transfers) / sizeof(transfers[0])
    };
    if (!twiinit)
        return;
    twi_schedule(NULL, ssd_twi_callback, &transaction);
}

// startscrolldiagright
// Activate a diagonal scroll for rows start through stop
// Hint, the display is 16 rows tall. To scroll the whole display, run:
// display.scrollright(0x00, 0x0F)
void ssd1306_start_scroll_diag_right(uint8_t start, uint8_t stop) {
    vertical_and_right_horizontal_scroll[SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL_START_OFFSET] = start;
    vertical_and_right_horizontal_scroll[SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL_STOP_OFFSET] = stop;
    static app_twi_transfer_t const transfers[] = {
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, set_vertical_scroll_area, sizeof(set_vertical_scroll_area), 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, vertical_and_right_horizontal_scroll, sizeof(vertical_and_right_horizontal_scroll), 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, activate_scroll, sizeof(activate_scroll), 0)
    };
    static app_twi_transaction_t const transaction = {
        .callback            = twi_callback,
        .p_user_data         = "~SSD-SVR",
        .p_transfers         = transfers,
        .number_of_transfers = sizeof(transfers) / sizeof(transfers[0])
    };
    if (!twiinit)
        return;
    twi_schedule(NULL, ssd_twi_callback, &transaction);
}

// startscrolldiagleft
// Activate a diagonal scroll for rows start through stop
// Hint, the display is 16 rows tall. To scroll the whole display, run:
// display.scrollright(0x00, 0x0F)
void ssd1306_start_scroll_diag_left(uint8_t start, uint8_t stop) {
    vertical_and_left_horizontal_scroll[SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL_START_OFFSET] = start;
    vertical_and_left_horizontal_scroll[SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL_STOP_OFFSET] = stop;
    static app_twi_transfer_t const transfers[] = {
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, set_vertical_scroll_area, sizeof(set_vertical_scroll_area), 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, vertical_and_left_horizontal_scroll, sizeof(vertical_and_left_horizontal_scroll), 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, activate_scroll, sizeof(activate_scroll), 0)
    };
    static app_twi_transaction_t const transaction = {
        .callback            = twi_callback,
        .p_user_data         = "~SSD-SVL",
        .p_transfers         = transfers,
        .number_of_transfers = sizeof(transfers) / sizeof(transfers[0])
    };
    if (!twiinit)
        return;
    twi_schedule(NULL, ssd_twi_callback, &transaction);
}

void ssd1306_stop_scroll(void) {
    static app_twi_transfer_t const transfers[] = {
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, deactivate_scroll, sizeof(deactivate_scroll), 0)
    };
    static app_twi_transaction_t const transaction = {
        .callback            = twi_callback,
        .p_user_data         = "~SSD-SS",
        .p_transfers         = transfers,
        .number_of_transfers = sizeof(transfers) / sizeof(transfers[0])
    };
    if (!twiinit)
        return;
    twi_schedule(NULL, ssd_twi_callback, &transaction);
}

// Dim the display
// dim = true: display is dimmed
// dim = false: display is normal
void ssd1306_dim(bool dim) {
    uint8_t contrast;
    if (dim) {
        contrast = 0; // Dimmed display
    } else {
#if (SSD1306_VCC == SSD1306_EXTERNALVCC)
        contrast = 0x9F;
#else
        contrast = 0xCF;
#endif
    }
    setcontrast[SSD1306_SETCONTRAST_OFFSET] = contrast;
    static app_twi_transfer_t const transfers[] = {
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, setcontrast, sizeof(setcontrast), 0)
    };
    static app_twi_transaction_t const transaction = {
        .callback            = twi_callback,
        .p_user_data         = "~SSD-SCON",
        .p_transfers         = transfers,
        .number_of_transfers = sizeof(transfers) / sizeof(transfers[0])
    };
    if (!twiinit)
        return;
    twi_schedule(NULL, ssd_twi_callback, &transaction);
}

void ssd1306_display(void) {
    uint16_t i, j;

    // Exit if we shouldn't be doing anything
    if (!twiinit || !display_initialized)
        return;

    // Exit if no display refresh is needed
    if (!display_needed)
        return;

    // Exit if we're already processing a display
    if (display_in_progress) {
        display_deferred = true;
        return;
    }

    // It's now in progress
    display_in_progress = true;

    // Move to I/O buffer
    for (i = j = 0; i < (SSD1306_LCDWIDTH * SSD1306_LCDHEIGHT / 8);) {
        data[j] = DATSTR;
        memcpy(&data[j+1], &buffer[i], DATCHUNK);
        i += DATCHUNK;
        j += DATCMDCHUNK;
    }

    // Now that we've copied it, we can begin filling it again
    display_needed = false;

    // Do the TWI I/O
    static app_twi_transfer_t const transfers[] = {
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, columnaddr, sizeof(columnaddr), 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, pageaddr, sizeof(pageaddr), 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, &data[DATCMDCHUNK*0], DATCMDCHUNK, 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, &data[DATCMDCHUNK*1], DATCMDCHUNK, 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, &data[DATCMDCHUNK*2], DATCMDCHUNK, 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, &data[DATCMDCHUNK*3], DATCMDCHUNK, 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, &data[DATCMDCHUNK*4], DATCMDCHUNK, 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, &data[DATCMDCHUNK*5], DATCMDCHUNK, 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, &data[DATCMDCHUNK*6], DATCMDCHUNK, 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, &data[DATCMDCHUNK*7], DATCMDCHUNK, 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, &data[DATCMDCHUNK*8], DATCMDCHUNK, 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, &data[DATCMDCHUNK*9], DATCMDCHUNK, 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, &data[DATCMDCHUNK*10], DATCMDCHUNK, 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, &data[DATCMDCHUNK*11], DATCMDCHUNK, 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, &data[DATCMDCHUNK*12], DATCMDCHUNK, 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, &data[DATCMDCHUNK*13], DATCMDCHUNK, 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, &data[DATCMDCHUNK*14], DATCMDCHUNK, 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, &data[DATCMDCHUNK*15], DATCMDCHUNK, 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, &data[DATCMDCHUNK*16], DATCMDCHUNK, 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, &data[DATCMDCHUNK*17], DATCMDCHUNK, 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, &data[DATCMDCHUNK*18], DATCMDCHUNK, 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, &data[DATCMDCHUNK*19], DATCMDCHUNK, 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, &data[DATCMDCHUNK*20], DATCMDCHUNK, 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, &data[DATCMDCHUNK*21], DATCMDCHUNK, 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, &data[DATCMDCHUNK*22], DATCMDCHUNK, 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, &data[DATCMDCHUNK*23], DATCMDCHUNK, 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, &data[DATCMDCHUNK*24], DATCMDCHUNK, 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, &data[DATCMDCHUNK*25], DATCMDCHUNK, 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, &data[DATCMDCHUNK*26], DATCMDCHUNK, 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, &data[DATCMDCHUNK*27], DATCMDCHUNK, 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, &data[DATCMDCHUNK*28], DATCMDCHUNK, 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, &data[DATCMDCHUNK*29], DATCMDCHUNK, 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, &data[DATCMDCHUNK*30], DATCMDCHUNK, 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, &data[DATCMDCHUNK*31], DATCMDCHUNK, 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, &data[DATCMDCHUNK*32], DATCMDCHUNK, 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, &data[DATCMDCHUNK*33], DATCMDCHUNK, 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, &data[DATCMDCHUNK*34], DATCMDCHUNK, 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, &data[DATCMDCHUNK*35], DATCMDCHUNK, 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, &data[DATCMDCHUNK*36], DATCMDCHUNK, 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, &data[DATCMDCHUNK*37], DATCMDCHUNK, 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, &data[DATCMDCHUNK*38], DATCMDCHUNK, 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, &data[DATCMDCHUNK*39], DATCMDCHUNK, 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, &data[DATCMDCHUNK*40], DATCMDCHUNK, 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, &data[DATCMDCHUNK*41], DATCMDCHUNK, 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, &data[DATCMDCHUNK*42], DATCMDCHUNK, 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, &data[DATCMDCHUNK*43], DATCMDCHUNK, 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, &data[DATCMDCHUNK*44], DATCMDCHUNK, 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, &data[DATCMDCHUNK*45], DATCMDCHUNK, 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, &data[DATCMDCHUNK*46], DATCMDCHUNK, 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, &data[DATCMDCHUNK*47], DATCMDCHUNK, 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, &data[DATCMDCHUNK*48], DATCMDCHUNK, 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, &data[DATCMDCHUNK*49], DATCMDCHUNK, 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, &data[DATCMDCHUNK*50], DATCMDCHUNK, 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, &data[DATCMDCHUNK*51], DATCMDCHUNK, 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, &data[DATCMDCHUNK*52], DATCMDCHUNK, 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, &data[DATCMDCHUNK*53], DATCMDCHUNK, 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, &data[DATCMDCHUNK*54], DATCMDCHUNK, 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, &data[DATCMDCHUNK*55], DATCMDCHUNK, 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, &data[DATCMDCHUNK*56], DATCMDCHUNK, 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, &data[DATCMDCHUNK*57], DATCMDCHUNK, 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, &data[DATCMDCHUNK*58], DATCMDCHUNK, 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, &data[DATCMDCHUNK*59], DATCMDCHUNK, 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, &data[DATCMDCHUNK*60], DATCMDCHUNK, 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, &data[DATCMDCHUNK*61], DATCMDCHUNK, 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, &data[DATCMDCHUNK*62], DATCMDCHUNK, 0),
        APP_TWI_WRITE(SSD1306_I2C_ADDRESS, &data[DATCMDCHUNK*63], DATCMDCHUNK, 0)
    };
    static app_twi_transaction_t const transaction = {
        .callback            = twi_callback,
        .p_user_data         = "~SSD-DISPLAY",
        .p_transfers         = transfers,
        .number_of_transfers = sizeof(transfers) / sizeof(transfers[0])
    };
    if (!twi_schedule(NULL, ssd_display_callback, &transaction))
        display_in_progress = false;

}

// clear everything
void ssd1306_clear_display(void) {
    ssd1306_set_cursor(0, 0);
    memset(buffer, 0, (SSD1306_LCDWIDTH * SSD1306_LCDHEIGHT / 8));
    ssd1306_display_needed();
}

// Draw line
void ssd1306_draw_fast_hline(int16_t x, int16_t y, int16_t w, uint16_t color) {
    bool __swap = false;
    switch (rotation) {
    case 0:
        // 0 degree rotation, do nothing
        break;
    case 1:
        // 90 degree rotation, swap x & y for rotation, then invert x
        __swap = true;
        ssd1306_swap(x, y);
        x = WIDTH - x - 1;
        break;
    case 2:
        // 180 degree rotation, invert x and y - then shift y around for height.
        x = WIDTH - x - 1;
        y = HEIGHT - y - 1;
        x -= (w - 1);
        break;
    case 3:
        // 270 degree rotation, swap x & y for rotation, then invert y  and adjust y for w (not to become h)
        __swap = true;
        ssd1306_swap(x, y);
        y = HEIGHT - y - 1;
        y -= (w - 1);
        break;
    }

    if (__swap) {
        draw_fast_vline_internal(x, y, w, color);
    }
    else {
        draw_fast_hline_internal(x, y, w, color);
    }
}

// Draw a line internal
void draw_fast_hline_internal(int16_t x, int16_t y, int16_t w, uint16_t color) {
    // Do bounds/limit checks
    if (y < 0 || y >= HEIGHT) {
        return;
    }

    // make sure we don't try to draw below 0
    if (x < 0) {
        w += x;
        x = 0;
    }

    // make sure we don't go off the edge of the display
    if ( (x + w) > WIDTH) {
        w = (WIDTH - x);
    }

    // if our width is now negative, punt
    if (w <= 0) {
        return;
    }

    // set up the pointer for  movement through the buffer
    register uint8_t *pBuf = buffer;
    // adjust the buffer pointer for the current row
    pBuf += ((y / 8) * SSD1306_LCDWIDTH);
    // and offset x columns in
    pBuf += x;

    register uint8_t mask = 1 << (y & 7);

    switch (color) {
    case SSD1306_WHITE:
        while (w--) {
            *pBuf++ |= mask;
        };
        break;
    case SSD1306_BLACK:
        mask = ~mask;
        while (w--) {
            *pBuf++ &= mask;
        };
        break;
    case SSD1306_INVERSE:
        while (w--) {
            *pBuf++ ^= mask;
        };
        break;
    }

    // Mark display as needing refresh
    ssd1306_display_needed();

}

// Vertical line
void ssd1306_draw_fast_vline(int16_t x, int16_t y, int16_t h, uint16_t color) {
    bool __swap = false;
    switch (rotation) {
    case 0:
        break;
    case 1:
        // 90 degree rotation, swap x & y for rotation, then invert x and adjust x for h (now to become w)
        __swap = true;
        ssd1306_swap(x, y);
        x = WIDTH - x - 1;
        x -= (h - 1);
        break;
    case 2:
        // 180 degree rotation, invert x and y - then shift y around for height.
        x = WIDTH - x - 1;
        y = HEIGHT - y - 1;
        y -= (h - 1);
        break;
    case 3:
        // 270 degree rotation, swap x & y for rotation, then invert y
        __swap = true;
        ssd1306_swap(x, y);
        y = HEIGHT - y - 1;
        break;
    }

    if (__swap) {
        draw_fast_hline_internal(x, y, h, color);
    }
    else {
        draw_fast_vline_internal(x, y, h, color);
    }
}

// Vertical line internal
void draw_fast_vline_internal(int16_t x, int16_t __y, int16_t __h, uint16_t color) {

    // do nothing if we're off the left or right side of the screen
    if (x < 0 || x >= WIDTH) {
        return;
    }

    // make sure we don't try to draw below 0
    if (__y < 0) {
        // __y is negative, this will subtract enough from __h to account for __y being 0
        __h += __y;
        __y = 0;

    }

    // make sure we don't go past the height of the display
    if ( (__y + __h) > HEIGHT) {
        __h = (HEIGHT - __y);
    }

    // if our height is now negative, punt
    if (__h <= 0) {
        return;
    }

    // this display doesn't need ints for coordinates, use local byte registers for faster juggling
    register uint8_t y = __y;
    register uint8_t h = __h;


    // set up the pointer for fast movement through the buffer
    register uint8_t *pBuf = buffer;
    // adjust the buffer pointer for the current row
    pBuf += ((y / 8) * SSD1306_LCDWIDTH);
    // and offset x columns in
    pBuf += x;

    // do the first partial byte, if necessary - this requires some masking
    register uint8_t mod = (y & 7);
    if (mod) {
        // mask off the high n bits we want to set
        mod = 8 - mod;

        // note - lookup table results in a nearly 10% performance improvement in fill* functions
        // register uint8_t mask = ~(0xFF >> (mod));
        static uint8_t premask[8] = {0x00, 0x80, 0xC0, 0xE0, 0xF0, 0xF8, 0xFC, 0xFE };
        register uint8_t mask = premask[mod];

        // adjust the mask if we're not going to reach the end of this byte
        if ( h < mod) {
            mask &= (0XFF >> (mod - h));
        }

        switch (color) {
        case SSD1306_WHITE:
            *pBuf |=  mask;
            break;
        case SSD1306_BLACK:
            *pBuf &= ~mask;
            break;
        case SSD1306_INVERSE:
            *pBuf ^=  mask;
            break;
        }

        // fast exit if we're done here!
        if (h < mod) {
            return;
        }

        h -= mod;

        pBuf += SSD1306_LCDWIDTH;
    }


    // write solid bytes while we can - effectively doing 8 rows at a time
    if (h >= 8) {
        if (color == SSD1306_INVERSE)  {          // separate copy of the code so we don't impact performance of the black/white write version with an extra comparison per loop
            do  {
                *pBuf = ~(*pBuf);

                // adjust the buffer forward 8 rows worth of data
                pBuf += SSD1306_LCDWIDTH;

                // adjust h & y (there's got to be a faster way for me to do this, but this should still help a fair bit for now)
                h -= 8;
            }
            while (h >= 8);
        }
        else {
            // store a local value to work with
            register uint8_t val = (color == SSD1306_WHITE) ? 255 : 0;

            do  {
                // write our value in
                *pBuf = val;

                // adjust the buffer forward 8 rows worth of data
                pBuf += SSD1306_LCDWIDTH;

                // adjust h & y (there's got to be a faster way for me to do this, but this should still help a fair bit for now)
                h -= 8;
            }
            while (h >= 8);
        }
    }

    // now do the final partial byte, if necessary
    if (h) {
        mod = h & 7;
        // this time we want to mask the low bits of the byte, vs the high bits we did above
        // register uint8_t mask = (1 << mod) - 1;
        // note - lookup table results in a nearly 10% performance improvement in fill* functions
        static uint8_t postmask[8] = {0x00, 0x01, 0x03, 0x07, 0x0F, 0x1F, 0x3F, 0x7F };
        register uint8_t mask = postmask[mod];
        switch (color) {
        case SSD1306_WHITE:
            *pBuf |=  mask;
            break;
        case SSD1306_BLACK:
            *pBuf &= ~mask;
            break;
        case SSD1306_INVERSE:
            *pBuf ^=  mask;
            break;
        }
    }

    // Mark display as needing refresh
    ssd1306_display_needed();

}


// Draw a circle outline
void ssd1306_draw_circle(int16_t x0, int16_t y0, int16_t r, uint16_t color) {
    int16_t f = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x = 0;
    int16_t y = r;

    ssd1306_draw_pixel(x0  , y0 + r, color);
    ssd1306_draw_pixel(x0  , y0 - r, color);
    ssd1306_draw_pixel(x0 + r, y0  , color);
    ssd1306_draw_pixel(x0 - r, y0  , color);

    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        ssd1306_draw_pixel(x0 + x, y0 + y, color);
        ssd1306_draw_pixel(x0 - x, y0 + y, color);
        ssd1306_draw_pixel(x0 + x, y0 - y, color);
        ssd1306_draw_pixel(x0 - x, y0 - y, color);
        ssd1306_draw_pixel(x0 + y, y0 + x, color);
        ssd1306_draw_pixel(x0 - y, y0 + x, color);
        ssd1306_draw_pixel(x0 + y, y0 - x, color);
        ssd1306_draw_pixel(x0 - y, y0 - x, color);
    }
}

void draw_circle_helper(int16_t x0, int16_t y0, int16_t r, uint8_t cornername, uint16_t color) {
    int16_t f     = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x     = 0;
    int16_t y     = r;

    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f     += ddF_y;
        }
        x++;
        ddF_x += 2;
        f     += ddF_x;
        if (cornername & 0x4) {
            ssd1306_draw_pixel(x0 + x, y0 + y, color);
            ssd1306_draw_pixel(x0 + y, y0 + x, color);
        }
        if (cornername & 0x2) {
            ssd1306_draw_pixel(x0 + x, y0 - y, color);
            ssd1306_draw_pixel(x0 + y, y0 - x, color);
        }
        if (cornername & 0x8) {
            ssd1306_draw_pixel(x0 - y, y0 + x, color);
            ssd1306_draw_pixel(x0 - x, y0 + y, color);
        }
        if (cornername & 0x1) {
            ssd1306_draw_pixel(x0 - y, y0 - x, color);
            ssd1306_draw_pixel(x0 - x, y0 - y, color);
        }
    }
}

void ssd1306_fill_circle(int16_t x0, int16_t y0, int16_t r, uint16_t color) {
    ssd1306_draw_fast_vline(x0, y0 - r, 2 * r + 1, color);
    fill_circle_helper(x0, y0, r, 3, 0, color);
}

// Used to do circles and roundrects
void fill_circle_helper(int16_t x0, int16_t y0, int16_t r, uint8_t cornername, int16_t delta, uint16_t color) {

    int16_t f     = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x     = 0;
    int16_t y     = r;

    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f     += ddF_y;
        }
        x++;
        ddF_x += 2;
        f     += ddF_x;

        if (cornername & 0x1) {
            ssd1306_draw_fast_vline(x0 + x, y0 - y, 2 * y + 1 + delta, color);
            ssd1306_draw_fast_vline(x0 + y, y0 - x, 2 * x + 1 + delta, color);
        }
        if (cornername & 0x2) {
            ssd1306_draw_fast_vline(x0 - x, y0 - y, 2 * y + 1 + delta, color);
            ssd1306_draw_fast_vline(x0 - y, y0 - x, 2 * x + 1 + delta, color);
        }
    }
}

// Bresenham's algorithm - thx wikpedia
void ssd1306_draw_line(int16_t x0, int16_t y0,
                       int16_t x1, int16_t y1,
                       uint16_t color) {
    int16_t steep = abs(y1 - y0) > abs(x1 - x0);
    if (steep) {
        adagfxswap(x0, y0);
        adagfxswap(x1, y1);
    }

    if (x0 > x1) {
        adagfxswap(x0, x1);
        adagfxswap(y0, y1);
    }

    int16_t dx, dy;
    dx = x1 - x0;
    dy = abs(y1 - y0);

    int16_t err = dx / 2;
    int16_t ystep;

    if (y0 < y1) {
        ystep = 1;
    }
    else {
        ystep = -1;
    }

    for (; x0 <= x1; x0++) {
        if (steep) {
            ssd1306_draw_pixel(y0, x0, color);
        }
        else {
            ssd1306_draw_pixel(x0, y0, color);
        }
        err -= dy;
        if (err < 0) {
            y0 += ystep;
            err += dx;
        }
    }
}

// Draw a rectangle
void ssd1306_draw_rect(int16_t x, int16_t y,
                       int16_t w, int16_t h,
                       uint16_t color) {
    ssd1306_draw_fast_hline(x, y, w, color);
    ssd1306_draw_fast_hline(x, y + h - 1, w, color);
    ssd1306_draw_fast_vline(x, y, h, color);
    ssd1306_draw_fast_vline(x + w - 1, y, h, color);
}

void ssd1306_fill_rect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
    // Update in subclasses if desired!
    for (int16_t i = x; i < x + w; i++) {
        ssd1306_draw_fast_vline(i, y, h, color);
    }
}

void ssd1306_fill_screen(uint16_t color) {
    ssd1306_fill_rect(0, 0, _width, _height, color);
}

// Draw a rounded rectangle
void ssd1306_draw_round_rect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color) {
    // smarter version
    ssd1306_draw_fast_hline(x + r  , y    , w - 2 * r, color); // Top
    ssd1306_draw_fast_hline(x + r  , y + h - 1, w - 2 * r, color); // Bottom
    ssd1306_draw_fast_vline(x    , y + r  , h - 2 * r, color); // Left
    ssd1306_draw_fast_vline(x + w - 1, y + r  , h - 2 * r, color); // Right
    // draw four corners
    draw_circle_helper(x + r    , y + r    , r, 1, color);
    draw_circle_helper(x + w - r - 1, y + r    , r, 2, color);
    draw_circle_helper(x + w - r - 1, y + h - r - 1, r, 4, color);
    draw_circle_helper(x + r    , y + h - r - 1, r, 8, color);
}

// Fill a rounded rectangle
void ssd1306_fill_round_rect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color) {
    // smarter version
    ssd1306_fill_rect(x + r, y, w - 2 * r, h, color);

    // draw four corners
    fill_circle_helper(x + w - r - 1, y + r, r, 1, h - 2 * r - 1, color);
    fill_circle_helper(x + r    , y + r, r, 2, h - 2 * r - 1, color);
}

// Draw a triangle
void ssd1306_draw_triangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color) {
    ssd1306_draw_line(x0, y0, x1, y1, color);
    ssd1306_draw_line(x1, y1, x2, y2, color);
    ssd1306_draw_line(x2, y2, x0, y0, color);
}

// Fill a triangle
void ssd1306_fill_triangle( int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color) {
    int16_t a, b, y, last;

    // Sort coordinates by Y order (y2 >= y1 >= y0)
    if (y0 > y1) {
        adagfxswap(y0, y1);
        adagfxswap(x0, x1);
    }
    if (y1 > y2) {
        adagfxswap(y2, y1);
        adagfxswap(x2, x1);
    }
    if (y0 > y1) {
        adagfxswap(y0, y1);
        adagfxswap(x0, x1);
    }

    if (y0 == y2) { // Handle awkward all-on-same-line case as its own thing
        a = b = x0;
        if (x1 < a)      a = x1;
        else if (x1 > b) b = x1;
        if (x2 < a)      a = x2;
        else if (x2 > b) b = x2;
        ssd1306_draw_fast_hline(a, y0, b - a + 1, color);
        return;
    }

    int16_t
        dx01 = x1 - x0,
        dy01 = y1 - y0,
        dx02 = x2 - x0,
        dy02 = y2 - y0,
        dx12 = x2 - x1,
        dy12 = y2 - y1;
    int32_t
        sa   = 0,
        sb   = 0;

    // For upper part of triangle, find scanline crossings for segments
    // 0-1 and 0-2.  If y1=y2 (flat-bottomed triangle), the scanline y1
    // is included here (and second loop will be skipped, avoiding a /0
    // error there), otherwise scanline y1 is skipped here and handled
    // in the second loop...which also avoids a /0 error here if y0=y1
    // (flat-topped triangle).
    if (y1 == y2) last = y1;  // Include y1 scanline
    else         last = y1 - 1; // Skip it

    for (y = y0; y <= last; y++) {
        a   = x0 + sa / dy01;
        b   = x0 + sb / dy02;
        sa += dx01;
        sb += dx02;
        /* longhand:
           a = x0 + (x1 - x0) * (y - y0) / (y1 - y0);
           b = x0 + (x2 - x0) * (y - y0) / (y2 - y0);
        */
        if (a > b) adagfxswap(a, b);
        ssd1306_draw_fast_hline(a, y, b - a + 1, color);
    }

    // For lower part of triangle, find scanline crossings for segments
    // 0-2 and 1-2.  This loop is skipped if y1=y2.
    sa = dx12 * (y - y1);
    sb = dx02 * (y - y0);
    for (; y <= y2; y++) {
        a   = x1 + sa / dy12;
        b   = x0 + sb / dy02;
        sa += dx12;
        sb += dx02;
        /* longhand:
           a = x1 + (x2 - x1) * (y - y1) / (y2 - y1);
           b = x0 + (x2 - x0) * (y - y0) / (y2 - y0);
        */
        if (a > b) adagfxswap(a, b);
        ssd1306_draw_fast_hline(a, y, b - a + 1, color);
    }
}


void ssd1306_draw_bitmap(int16_t x, int16_t y, const uint8_t *bitmap, int16_t w, int16_t h, uint16_t color) {
    int16_t i, j, byteWidth = (w + 7) / 8;

    for (j = 0; j < h; j++) {
        for (i = 0; i < w; i++ ) {
            if (bitmap[j * byteWidth + i / 8] & (128 >> (i & 7))) {
                ssd1306_draw_pixel(x + i, y + j, color);
            }
        }
    }
}

// Draw a 1-bit color bitmap at the specified x, y position from the
// provided bitmap buffer (must be PROGMEM memory) using color as the
// foreground color and bg as the background color.
void ssd1306_draw_bitmap_bg(int16_t x, int16_t y,
                            const uint8_t *bitmap, int16_t w, int16_t h,
                            uint16_t color, uint16_t bg) {
    int16_t i, j, byteWidth = (w + 7) / 8;

    for (j = 0; j < h; j++) {
        for (i = 0; i < w; i++ ) {
            if (bitmap[j * byteWidth + i / 8] & (128 >> (i & 7))) {
                ssd1306_draw_pixel(x + i, y + j, color);
            }
            else {
                ssd1306_draw_pixel(x + i, y + j, bg);
            }
        }
    }
}

//Draw XBitMap Files (*.xbm), exported from GIMP,
//Usage: Export from GIMP to *.xbm, rename *.xbm to *.c and open in editor.
//C Array can be directly used with this function
void ssd1306_draw_xbitmap(int16_t x, int16_t y,
                          const uint8_t *bitmap, int16_t w, int16_t h,
                          uint16_t color) {

    int16_t i, j, byteWidth = (w + 7) / 8;

    for (j = 0; j < h; j++) {
        for (i = 0; i < w; i++ ) {
            if (bitmap[j * byteWidth + i / 8] & (1 << (i % 8))) {
                ssd1306_draw_pixel(x + i, y + j, color);
            }
        }
    }
}

void scroll_up_line() {
    // Wrap, or scroll
#if 0   // Wrap
    ssd1306_clear_display();
    ssd1306_set_cursor(0, 0);
#else   // Scroll, in an extremely pokey way
    int x, y;
    for (y=textsize*lineheight; y<_height; y++)
        for (x=0; x<_width; x++)
            ssd1306_draw_pixel(x, y-(textsize*lineheight), ssd1306_get_pixel(x, y));
    ssd1306_fill_rect(0, _height-(textsize*lineheight), _width, textsize*lineheight, textcolor == SSD1306_WHITE ? SSD1306_BLACK : SSD1306_WHITE);
#endif
}

size_t ssd1306_write(uint8_t c) {

    // Merely for efficiency, exit if we're not yet initialized
    if (!twiinit || !display_initialized)
        return 0;

    if (c == '\n') {
        wrap_on_next_char = true;
        wrap_prefix = false;
    } else if (c == '\r') {
        // skip em
    } else {
        if (wrap_on_next_char) {
            cursor_y += textsize * lineheight;
            if (wrap_prefix)
                cursor_x = charwidth + charwidth;
            else
                cursor_x = 0;
            if (cursor_y >= _height) {
                cursor_y = _height-(textsize*lineheight);
                scroll_up_line();
            }
        }
        wrap_on_next_char = false;
        ssd1306_draw_char(cursor_x, cursor_y, c, textcolor, textbgcolor, textsize);
        cursor_x += textsize * charwidth;
        if (wrap && (cursor_x > (_width - textsize * charwidth))) {
            wrap_on_next_char = true;
            wrap_prefix = true;
        }
    }

    return 1;
}

// Draw a character
void ssd1306_draw_char(int16_t x, int16_t y, uint8_t c, uint16_t color, uint16_t bg, uint8_t size) {

    if ((x >= _width)            || // Clip right
        (y >= _height)           || // Clip bottom
        ((x + charwidth * size - 1) < 0) || // Clip left
        ((y + lineheight * size - 1) < 0))   // Clip top
        return;

    if (!_cp437 && (c >= 176)) c++; // Handle 'classic' charset behavior

    for (int8_t i = 0; i < charwidth; i++ ) {
        uint8_t line;
        if (i == (charwidth-1))
            line = 0x0;
        else
            line = font[(c * (charwidth-1)) + i];
        for (int8_t j = 0; j < lineheight; j++) {
            if (line & 0x1) {
                if (size == 1) // default size
                    ssd1306_draw_pixel(x + i, y + j, color);
                else {  // big size
                    ssd1306_fill_rect(x + (i * size), y + (j * size), size, size, color);
                }
            }
            else if (bg != color) {
                if (size == 1) // default size
                    ssd1306_draw_pixel(x + i, y + j, bg);
                else {  // big size
                    ssd1306_fill_rect(x + i * size, y + j * size, size, size, bg);
                }
            }
            line >>= 1;
        }
    }
}

void ssd1306_set_cursor(int16_t x, int16_t y) {
    cursor_x = x;
    cursor_y = y;
    wrap_on_next_char = false;
}

int16_t ssd1306_get_cursor_x(void) {
    return cursor_x;
}

int16_t ssd1306_get_cursor_y(void) {
    return cursor_y;
}

void ssd1306_set_textsize(uint8_t s) {
    textsize = (s > 0) ? s : 1;
}

void ssd1306_set_textcolor(uint16_t c) {
    // For 'transparent' background, we'll set the bg
    // to the same as fg instead of using a flag
    textcolor = textbgcolor = c;
}

void ssd1306_set_textcolor_bg(uint16_t c, uint16_t b) {
    textcolor   = c;
    textbgcolor = b;
}

void ssd1306_set_textwrap(bool w) {
    wrap = w;
}

uint8_t ssd1306_get_rotation(void) {
    return rotation;
}

void ssd1306_set_rotation(uint8_t x) {
    rotation = (x & 3);
    switch (rotation) {
    case 0:
    case 2:
        _width  = WIDTH;
        _height = HEIGHT;
        break;
    case 1:
    case 3:
        _width  = HEIGHT;
        _height = WIDTH;
        break;
    }
}

// Enable (or disable) Code Page 437-compatible charset.
// There was an error in glcdfont.c for the longest time -- one character
// (#176, the 'light shade' block) was missing -- this threw off the index
// of every character that followed it.  But a TON of code has been written
// with the erroneous character indices.  By default, the library uses the
// original 'wrong' behavior and old sketches will still work.  Pass 'true'
// to this function to use correct CP437 character values in your code.
void ssd1306_cp437(bool x) {
    _cp437 = x;
}


void ssd1306_putstring(char* b) {
    while (*b) {
        ssd1306_write((uint8_t)*b);
        b++;
    }
}

void ssd1306_puts(char* b) {
    ssd1306_putstring(b);
    ssd1306_write('\n');
}

#endif // SSD
