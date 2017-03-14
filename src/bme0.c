// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

#define BME280_I2C_ADDRESS      0x77

#ifdef bme
#undef bme
#endif
#define bme(FUNC) s_bme280_0_##FUNC
#define BMESTR "BME0"

#include "bme-c.h"
