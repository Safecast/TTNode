// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

#ifndef OPC_H__
#define OPC_H__

#include "nrf_drv_spi.h"

void s_opc_get_spi(uint16_t *pin, nrf_drv_spi_handler_t *handler);
bool s_opc_upload_needed(void *);
void s_opc_measure(void *);
bool s_opc_show_value(uint32_t when, char *buffer, uint16_t length);
bool s_opc_get_value(float *ppm_01_0, float *ppm_02_5, float *ppm_10_0,
                     float *pstd_01_0, float *pstd_02_5, float *pstd_10_0,
                     uint32_t *pcount_00_38, uint32_t *pcount_00_54, uint32_t *pcount_01_00,
                     uint32_t *pcount_02_10, uint32_t *pcount_05_00, uint32_t *pcount_10_00,
                     uint16_t *pcount_seconds);
void s_opc_clear_measurement();
void s_opc_poll(void *s);
bool s_opc_init(void *s, uint16_t param);
bool s_opc_term();

#endif // OPC_H__
