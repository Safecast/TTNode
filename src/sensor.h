// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

#ifndef SENSOR_H__
#define SENSOR_H__

#include "app_timer.h"

#define END_OF_LIST NULL
#define NO_HANDLER NULL
#define SENSOR_PIN_UNDEFINED 0xff

struct sensor_state_s {
    bool is_configured;
    bool is_requesting_deconfiguration;
    bool is_settling;
    bool is_processing;
    bool is_polling_valid;
    bool is_completed;
    bool is_being_tested;
    uint16_t init_failures;
    uint16_t term_failures;
    uint32_t last_settled;
    struct _sensor_app_timer {
        // see APP_TIMER_DEF in app_timer.h
        app_timer_t timer_data;
        app_timer_id_t timer_id;
            } sensor_timer;
};
typedef struct sensor_state_s sensor_state_t;

typedef bool (*sensor_init_handler_t) (void *s, uint16_t);
typedef bool (*sensor_term_handler_t) (void);
typedef void (*sensor_settling_handler_t) (void);
typedef void (*sensor_measure_handler_t) (void *s);
typedef bool (*sensor_upload_needed_handler_t) (void *s);
typedef void (*sensor_poll_handler_t) (void *context);
typedef bool (*sensor_value_handler_t) (uint32_t when, char *buffer, uint16_t length);

struct sensor_s {
    // Unique, for debugging only
    char *name;
    sensor_state_t state;
    // If any SENSOR_ (storage.h) flag is nonzero, then this sensor is enabled
    uint32_t storage_sensor_mask;
    // Parameter for both of the above
    uint16_t init_parameter;
    // called only once at system startup
    sensor_init_handler_t init_once;
    // called whenever power has just been applied and we're beginning the settling period
    sensor_init_handler_t init_power;
    // called whenever power is about to be removed
    sensor_term_handler_t term_power;
    // called only once at system startup
    sensor_value_handler_t show_value;
    // Poller is active only while group is active, except if poll_continuous is asserted
    uint32_t poll_repeat_milliseconds;
    // Poll continuously
    bool poll_continuously;
    // Poller is active during group settling
    bool poll_during_settling;
    // polling for this sensor
    sensor_poll_handler_t poll_handler;
    // Time to allow for settling
    uint16_t settling_seconds;
    // called when we end the sensor settling period
    sensor_settling_handler_t done_settling;
    // called when we end the group settling period
    sensor_settling_handler_t done_group_settling;
    // See if a measurement is even required    
    sensor_upload_needed_handler_t upload_needed;
    // Called to request a measurement.
    // This method must call sensor_measurement_completed() when it's done.
    sensor_measure_handler_t measure;
};
typedef struct sensor_s sensor_t;

struct group_state_s {
    bool is_configured;
    bool is_requesting_deconfiguration;
    bool is_settling;
    bool is_processing;
    bool is_polling_valid;
    bool is_powered_on;
    bool is_being_tested;
    uint32_t last_settled;
    uint32_t last_repeated;
    uint32_t repeat_seconds_override;
    struct _group_app_timer {
        // see APP_TIMER_DEF in app_timer.h
        app_timer_t timer_data;
        app_timer_id_t timer_id;
            } group_timer;
};
typedef struct group_state_s group_state_t;

typedef void (*group_power_handler_t) (uint16_t parameter, bool enable);
typedef void (*group_poll_handler_t) (void *context);
typedef bool (*group_skip_handler_t) (void *context);
typedef void (*group_settling_handler_t) (void);

struct repeat_s {
    // Valid anytime current battery status matches THIS via bitwise AND
    uint16_t active_battery_status;
    // Number of minutes to repeat if that is the case
    uint32_t repeat_seconds;
};
typedef struct repeat_s repeat_t;
    
struct group_s {
    // Unique, for debugging only
    char *name;
    // Internal state
    group_state_t state;
    // Only valid for the product PRODUCT_ (see storage.h), else entire group is ignored
    uint16_t storage_product;
    // Valid anytime current battery status matches THIS via bitwise AND
    uint16_t active_battery_status;
    // Valid anytime current comm mode matches THIS via bitwise AND
    uint16_t active_comm_mode;
    // Skip this group during polls when this is true
    group_skip_handler_t skip_handler;
    // Power on/off
    group_power_handler_t power_set;
    uint16_t power_set_parameter;
    // True if it should only be run with everything else turned OFF, because of voltage measurement
    bool exclusive;
    // True if it should only be run with everything else turned OFF, because of power draw
    bool power_exclusive;
    // True if it should only be run with if it's the only TWI device, to ensure TWI bus exclusivity
    bool twi_exclusive;
    // Poller is active only while group is active, except if poll_continuous is asserted
    uint32_t poll_repeat_milliseconds;
    // Poll continuously
    bool poll_continuously;
    // Poller is active during group settling
    bool poll_during_settling;
    // Polling for this group
    group_poll_handler_t poll_handler;
    // Timers
    uint16_t settling_seconds;
    // called when we end the settling period
    group_settling_handler_t done_settling;
    // Poll repeat minutes
    bool sense_at_boot;
    // Poll repeat minutes
    repeat_t *repeat;
    // The UART that must be selected for this sensor to be sampled
    uint16_t uart_required;
    // The UART that must be selected, but ONLY IF the uart is AVAILABLE for switching
    uint16_t uart_requested;
    // Sensors in the sensor group
    sensor_t *sensors[];
};
typedef struct group_s group_t;

// Misc
void sensor_poll();
void sensor_show_state(bool fVerbose);
void sensor_measurement_completed(sensor_t *s);
void sensor_unconfigure(sensor_t *s);
bool sensor_group_completed(group_t *g);
bool sensor_any_upload_needed(void);
void sensor_group_unconfigure(group_t *g);
bool sensor_group_powered_on(group_t *g);
bool sensor_is_polling_valid(sensor_t *g);
bool sensor_group_is_polling_valid(group_t *g);
bool sensor_group_any_exclusive_powered_on();
bool sensor_group_any_exclusive_twi_on();
bool sensor_group_any_exclusive_busy();
bool sensor_group_busy();
void sensor_set_pin_state(uint16_t pin, bool enable);
void sensor_begin_uart_sensor_scheduling();
bool sensor_currently_in_motion();
bool sensor_toggle_battery_test_mode();
bool sensor_test_mode();
void *sensor_group_name(char *name);
bool sensor_schedule_now();
bool sensor_group_schedule_now(char *gname);
void sensor_freeze(bool fFreeze);
bool sensor_is_being_tested(sensor_t *s);
void sensor_test(char *name);
void sensor_abort(sensor_t *s);
bool g_mobile_skip(void *g);

// Only one mode is ever active, however this is defined bitwise so that
// we can test using a bitwise-AND operator rather than just == or switch.
#define OPMODE_NORMAL           0
#define OPMODE_MOBILE           1
#define OPMODE_TEST_FAST        2   // Lower repeat delays and lower oneshot delays
#define OPMODE_TEST_BURN        3   // Fast mode with comms auto-swapping, for burn-in testing
#define OPMODE_TEST_SENSOR      4   // Single-sensor test mode
#define OPMODE_TEST_DEAD        5   // Dead mode, where no sensors are scheduled at all
bool sensor_set_op_mode(uint16_t mode);
void sensor_set_temporary_op_mode(uint16_t op_mode, uint32_t seconds);
uint16_t sensor_op_mode();
void sensor_show_values(bool fReset);
uint16_t sensor_get_mobile_upload_period();
void sensor_set_mobile_upload_period(uint16_t);
uint32_t sensor_get_mobile_session_id();

#endif // SENSOR_H__
