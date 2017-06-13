// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

#ifndef TIMERS_H__
#define TIMERS_H__

#include "app_timer.h"

/// NRF parameters that we use for our app's timers
#define APP_TIMER_TICKS_PER_SECOND      32768
// Value of the RTC1 PRESCALER register.
#define APP_TIMER_PRESCALER             0
// Maximum number of timers used by the app.
#define APP_TIMER_MAX_TIMERS            10
// Size of timer operation queues.
#define APP_TIMER_OP_QUEUE_SIZE         32

// Scheduler related.  In comm, we queue only the CMDBUF_TYPE for processing.  But we also
// use the scheduler for timer interrupts.
#define SCHED_MAX_EVENT_DATA_SIZE MAX(APP_TIMER_SCHED_EVT_SIZE, sizeof(uint32_t))
#define SCHED_QUEUE_SIZE                100

// Max NRF_DELAY given that busy buffer and sched queue would otherwise overflow
#include "nrf_delay.h"
#define MAX_NRF_DELAY_MS 500

// Misc
void timer_init(void);
void timer_start();
char *time_since_boot();
void timer_update_mode();

uint32_t get_seconds_since_boot(void);
void set_timestamp(uint32_t date, uint32_t time);
bool get_current_timestamp(uint32_t *date, uint32_t *time, uint32_t *offset);

void welcome_message(void);

#endif // TIMERS_H__
