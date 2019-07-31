// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

#ifndef SPI_H__
#define SPI_H__

#include "nrf_drv_spi.h"

bool spi_init();
bool spi_term();
nrf_drv_spi_t *spi_context();

#endif // SPI_H__
