// MAX17043 Fuel Gauge

#ifndef MAX17043_H__
#define MAX17043_H__

void s_bat_voltage_measure(void *s);
bool s_bat_voltage_upload_needed(void *s);
bool s_bat_voltage_get_value(float *voltage);
void s_bat_voltage_clear_measurement();
uint32_t s_bat_autoadjust_seconds(uint32_t seconds);
bool s_bat_voltage_init();
bool s_bat_voltage_term();
void s_bat_soc_measure(void *s);
bool s_bat_soc_upload_needed(void *s);
bool s_bat_soc_get_value(float *soc);
void s_bat_soc_clear_measurement();
bool s_bat_soc_init();
bool s_bat_soc_term();

#endif // MAX17043_H__
