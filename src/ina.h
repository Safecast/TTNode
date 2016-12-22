// INA219 Current Gauge

#ifndef INA219_H__
#define INA219_H__

void s_ina_measure(void *s);
bool s_ina_upload_needed(void *s);
bool s_ina_get_value(float *pBusVoltage, float *pSOC, float *pCurrent);
void s_ina_clear_measurement();
bool s_ina_init();
bool s_ina_term();
void s_ina_poll(void *g);

#endif // INA219_H__
