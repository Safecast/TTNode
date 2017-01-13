// Adafruit Ultimate GPS

#ifndef UGPS_H__
#define UGPS_H__

#ifdef UGPS

void ugps_received_byte(uint8_t databyte);

bool s_ugps_init(void);
bool s_ugps_term(void);
bool s_ugps_skip(void *g);
void s_ugps_poll(void *g);
void s_ugps_update(void);
void s_ugps_clear_measurement();
void s_ugps_done_settling();
void s_ugps_shutdown();
uint16_t s_ugps_get_value(float *lat, float *lon, float *alt);
void s_ugps_received_byte(uint8_t databyte);
    
#endif // UGPS

#endif // UGPS_H__
