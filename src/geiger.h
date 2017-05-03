// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

#ifndef GEIGER_H__
#define GEIGER_H__
#ifdef GEIGERX

// Geiger types - do not assign 0 so that this can be a test if not defined
#define LND7318U    1
#define LND7318C    2
#define LND7128EC   3

bool s_geiger_get_value(bool *pAvail0, uint32_t *pCPM0, bool *pAvail1, uint32_t *pCPM1);
bool s_geiger_show_value(uint32_t when, char *buffer, uint16_t length);
void s_geiger_clear_measurement();
void geiger0_event();
void geiger1_event();
void geiger_poll();
bool g_geiger_skip(void *g);
void s_geiger_poll(void *s);
bool s_geiger_init(void *s, uint16_t param);
bool s_geiger_term();
void s_geiger_measure(void *s);
bool s_geiger_upload_needed(void *s);

#endif // GEIGERX
#endif // GEIGER_H__
