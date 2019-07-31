// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

#if defined(TWIBME1)

#define BME280_I2C_ADDRESS      0x76

#ifdef bme
#undef bme
#endif
#define bme(FUNC) s_bme280_1_##FUNC
#define bme_error() stats()->errors_bme1++
#define BMESTR "BME1"

#include "bme-c.h"

#endif
