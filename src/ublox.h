// UBLOXM8 GPS sensor

#ifndef UBLOXM8_H__
#define UBLOXM8_H__

bool s_gps_init(void);
bool s_gps_term(void);
void s_gps_poll(void *g);
void s_gps_shutdown();
uint16_t s_gps_get_value(float *lat, float *lon, float *alt);

#endif // UBLOXM8_H__
