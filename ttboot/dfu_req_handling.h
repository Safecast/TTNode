// Portions Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

/* Portions Copyright (c) 2016 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 */

#ifndef DFU_REQ_HANDLING_H__
#define DFU_REQ_HANDLING_H__

#include "compiler_abstraction.h"

__ALIGN(4) extern const uint8_t pk[64];

nrf_dfu_res_code_t nrf_dfu_command_req(void * p_context, nrf_dfu_req_t * p_req, nrf_dfu_res_t * p_res);
nrf_dfu_res_code_t nrf_dfu_data_req(void * p_context, nrf_dfu_req_t * p_req, nrf_dfu_res_t * p_res);
void nrf_dfu_req_handler_reset_if_dfu_complete(void);

#endif // #ifndef DFU_REQ_HANDLING_H__
