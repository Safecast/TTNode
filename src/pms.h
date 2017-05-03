// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

#ifndef PMS_H__
#define PMS_H__
#ifdef PMSX

void pms_received_byte(uint8_t databyte);
void s_pms_measure(void *s);
bool s_pms_upload_needed(void *s);
bool s_pms_init(void *s, uint16_t param);
bool s_pms_term(void);
void s_pms_poll(void *s);
void s_pms_clear_measurement();
bool s_pms_show_value(uint32_t when, char *buffer, uint16_t length);

#if defined(PMS1003) || defined(PMS5003) || defined(PMS7003)
bool s_pms_get_value(uint16_t *ppms_pm01_0, uint16_t *ppms_pm02_5, uint16_t *ppms_pm10_0,
                     float *pstd_01_0, float *pstd_02_5, float *pstd_10_0,
                     uint32_t *ppms_c00_30, uint32_t *ppms_c00_50, uint32_t *ppms_c01_00, uint32_t *ppms_c02_50, uint32_t *ppms_c05_00, uint32_t *ppms_c10_00, uint16_t *ppms_csecs);
#else
bool s_pms_get_value(uint16_t *ppms_pm01_0, uint16_t *ppms_pm02_5, uint16_t *ppms_pm10_0
                         float *pstd_01_0, float *pstd_02_5, float *pstd_10_0);
#endif
    
#endif // PMSX
#endif // PMS_H__
