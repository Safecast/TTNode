// Sensor measurement support

#ifndef SENSOR_H__
#define SENSOR_H__

#include "app_timer.h"

#define END_OF_LIST NULL
#define NO_HANDLER NULL
#define SENSOR_PIN_UNDEFINED 0xff

struct sensor_state_s {
    bool is_configured;
    bool is_settling;
    bool is_processing;
    bool is_completed;
    uint16_t init_failures;
    uint16_t term_failures;
    uint32_t last_settled;
};
typedef struct sensor_state_s sensor_state_t;

typedef bool (*sensor_init_handler_t) (void);
typedef void (*sensor_settling_handler_t) (void);
typedef void (*sensor_measure_handler_t) (void *s);
typedef bool (*sensor_upload_needed_handler_t) (void *s);

struct sensor_s {
    // Unique, for debugging only
    char *name;
    sensor_state_t state;
    // If any SENSOR_ (storage.h) flag is nonzero, then this sensor is enabled
    uint32_t storage_sensor_mask;
    // called only once at system startup
    sensor_init_handler_t init_once;
    // called whenever power has just been applied and we're beginning the settling period
    sensor_init_handler_t init_power;
    // called whenever power is about to be removed
    sensor_init_handler_t term_power;
    // Time to allow for settling
    uint16_t settling_seconds;
    // See if a measurement is even required    
    sensor_upload_needed_handler_t upload_needed;
    // Called to request a measurement.
    // This method must call sensor_measurement_completed() when it's done.
    sensor_measure_handler_t measure;

};
typedef struct sensor_s sensor_t;

struct group_state_s {
    bool is_configured;
    bool is_settling;
    bool is_processing;
    bool is_polling_valid;
    bool is_powered_on;
    uint32_t last_settled;
    uint32_t last_repeated;
    uint16_t repeat_minutes_override;
    struct _app_timer {
        // see APP_TIMER_DEF in app_timer.h
        app_timer_t timer_data;
        app_timer_id_t timer_id;
            } group_timer;
};
typedef struct group_state_s group_state_t;

typedef void (*group_power_handler_t) (uint16_t parameter, bool init, bool enable);
typedef void (*group_poll_handler_t) (void *context);
typedef bool (*group_skip_handler_t) (void *context);

struct repeat_s {
    // Valid anytime current battery status matches THIS via bitwise AND
    uint16_t active_battery_status;
    // Number of minutes to repeat if that is the case
    uint16_t repeat_minutes;
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
    // True if it should only be run with everything else turned OFF, because of power draw
    bool power_exclusive;
    // Poller is active only while group is active, except if poll_continuous is asserted
    uint32_t poll_repeat_milliseconds;
    bool poll_continuously;
    // Poller is active during group settling
    bool poll_during_settling;
    group_poll_handler_t poll_handler;
    // Timers
    uint16_t settling_seconds;
    // called when we end the settling period
    sensor_settling_handler_t done_settling;
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
void sensor_show_state();
void sensor_measurement_completed(sensor_t *s);
void sensor_unconfigure(sensor_t *s, uint32_t err_code);
bool sensor_group_completed(group_t *g);
bool sensor_any_upload_needed(void);
void sensor_group_unconfigure(group_t *g, uint32_t err_code);
bool sensor_group_powered_on(group_t *g);
bool sensor_is_polling_valid(group_t *g);
bool sensor_group_any_exclusive_powered_on();
void sensor_set_pin_state(uint16_t pin, bool init, bool enable);
void sensor_begin_uart_sensor_scheduling();
void sensor_set_bat_soc(float SOC);
float sensor_get_bat_soc();
void sensor_set_battery_test_mode();
void sensor_set_hammer_test_mode();
bool sensor_hammer_test_mode();
void *sensor_group(char *name);
void sensor_group_schedule_now(group_t *g);

// Only one mode is ever active, however this is defined bitwise so that
// we can test using a bitwise-AND operator rather than just == or switch.
#define BAT_FULL                0x0001
#define BAT_NORMAL              0x0002
#define BAT_LOW                 0x0004
#define BAT_WARNING             0x0008
#define BAT_EMERGENCY           0x0010
#define BAT_DEAD                0x0020
#define BAT_TEST                0x0040
#define BAT_HEALTHY             (BAT_FULL|BAT_NORMAL|BAT_LOW|BAT_WARNING|BAT_TEST)
#define BAT_NOT_DEAD            (BAT_HEALTHY|BAT_EMERGENCY)
#define BAT_ALL                 (BAT_NOT_DEAD|BAT_DEAD)
uint16_t sensor_get_battery_status();

#endif // SENSOR_H__
