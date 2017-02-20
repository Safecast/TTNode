// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

#ifndef CCCD_H__
#define CCCD_H__

uint32_t cccd_configure(uint16_t conn_handle, uint16_t handle_cccd, bool enable);
void cccd_tx_buffer_process(void);

#endif // CCCD_H__

