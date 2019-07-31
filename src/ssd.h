#ifndef __SSD1306_LIB_H
#define __SSD1306_LIB_H

#ifdef SSD

#define SSD1306_BLACK 0
#define SSD1306_WHITE 1
#define SSD1306_INVERSE 2

int16_t ssd1306_width(void);
int16_t ssd1306_height(void);
void set_rotation(uint8_t x);
bool ssd1306_init();
void ssd1306_force_reset();
bool ssd1306_term();
bool ssd1306_active();
void ssd1306_command(uint8_t c);
void ssd1306_draw_pixel(int16_t x, int16_t y, uint16_t color);
uint16_t ssd1306_get_pixel(int16_t x, int16_t y);
void ssd1306_invert_display(bool fInvert);
void ssd1306_reset_display();
void ssd1306_start_scroll_right(uint8_t start, uint8_t stop);
void ssd1306_start_scroll_left(uint8_t start, uint8_t stop);
void ssd1306_start_scroll_diag_right(uint8_t start, uint8_t stop);
void ssd1306_start_scroll_diag_left(uint8_t start, uint8_t stop);
void ssd1306_stop_scroll(void);
void ssd1306_dim(bool dim);
void ssd1306_display(void);
void ssd1306_clear_display(void);
void ssd1306_draw_fast_hline(int16_t x, int16_t y, int16_t w, uint16_t color);
void ssd1306_draw_fast_vline(int16_t x, int16_t y, int16_t h, uint16_t color);
void ssd1306_draw_circle(int16_t x0, int16_t y0, int16_t r, uint16_t color);
void ssd1306_fill_circle(int16_t x0, int16_t y0, int16_t r, uint16_t color);
void ssd1306_draw_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);
void ssd1306_draw_rect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
void ssd1306_fill_rect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
void ssd1306_fill_screen(uint16_t color);
void ssd1306_draw_round_rect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color);
void ssd1306_fill_round_rect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color);
void ssd1306_draw_triangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);
void ssd1306_fill_triangle( int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);
void ssd1306_draw_bitmap(int16_t x, int16_t y, const uint8_t *bitmap, int16_t w, int16_t h, uint16_t color);
void ssd1306_draw_bitmap_bg(int16_t x, int16_t y, const uint8_t *bitmap, int16_t w, int16_t h, uint16_t color, uint16_t bg);
void ssd1306_draw_xbitmap(int16_t x, int16_t y, const uint8_t *bitmap, int16_t w, int16_t h, uint16_t color);
size_t ssd1306_write(uint8_t c);
void ssd1306_draw_char(int16_t x, int16_t y, uint8_t c, uint16_t color, uint16_t bg, uint8_t size);
void ssd1306_set_cursor(int16_t x, int16_t y);
int16_t ssd1306_get_cursor_x(void);
int16_t ssd1306_get_cursor_y(void);
void ssd1306_set_textsize(uint8_t s);
void ssd1306_set_textcolor(uint16_t c);
void ssd1306_set_textcolor_bg(uint16_t c, uint16_t b);
void ssd1306_set_textwrap(bool w);
uint8_t ssd1306_get_rotation(void);
void ssd1306_set_rotation(uint8_t x);
void ssd1306_cp437(bool x);
void ssd1306_putstring(char* buffer);
void ssd1306_puts(char* buffer);

#endif // SSD

#endif /* __SSD1306_LIB_H */

