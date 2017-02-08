// MAX17201 Fuel Gauge

#ifndef MAX17201_H__
#define MAX17201_H__

void s_max01_measure(void *s);
bool s_max01_upload_needed(void *s);
bool s_max01_get_value(float *pVoltage, float *pSOC, float *pCurrent);
void s_max01_clear_measurement();
bool s_max01_init();
bool s_max01_term();
void s_max01_poll(void *g);

#endif // MAX17201_H__
