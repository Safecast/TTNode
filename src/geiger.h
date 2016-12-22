//  Geiger support

#ifndef GEIGER_H__
#define GEIGER_H__
#ifdef GEIGER

bool s_geiger_get_value(bool *pAvail0, uint32_t *pCPM0, bool *pAvail1, uint32_t *pCPM1);
void s_geiger_clear_measurement();
void geiger0_event();
void geiger1_event();

// Sensor
void s_geiger_poll(void *g);
bool s_geiger_init();
void s_geiger_measure(void *s);
bool s_geiger_upload_needed(void *s);

#endif // GEIGER
#endif // GEIGER_H__
