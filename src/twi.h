// TWI support

#ifndef TWI_H__
#define TWI_H__

#include "app_twi.h"
#include "ublox.h"
#include "hih.h"
#include "max.h"

bool twi_init();
bool twi_term();
app_twi_t *twi_context();

#endif // TWI_H__
