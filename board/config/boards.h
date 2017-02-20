// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

#ifndef BOARDS_H
#define BOARDS_H

#include "nrf_gpio.h"
#if defined(BOARD_CUSTOM)
  #include "custom_board.h"
#else
#error "Board is not defined"
#endif

#endif  // BOARDS_H
