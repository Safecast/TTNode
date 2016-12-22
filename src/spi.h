// SPI support

#ifndef SPI_H__
#define SPI_H__

#include "nrf_drv_spi.h"

bool spi_init();
bool spi_term();
nrf_drv_spi_t *spi_context();

#endif // SPI_H__
