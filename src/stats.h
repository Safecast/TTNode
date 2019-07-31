// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

#ifndef STATS_H__
#define STATS_H__

// Stats structure
struct stats_s {
    uint32_t transmitted;
    uint32_t transmitted_today;
    uint32_t transmitted_fullday;
    uint32_t max_transmitted_today;
    uint32_t max_transmitted_fullday;
    uint32_t received;
    uint32_t received_today;
    uint32_t received_fullday;
    uint32_t messages;
    uint32_t messages_today;
    uint32_t messages_fullday;
    uint32_t joins;
    uint32_t joins_today;
    uint32_t joins_fullday;
    uint32_t denies;
    uint32_t denies_today;
    uint32_t denies_fullday;
    uint32_t resets;
    uint32_t power_fails;
    uint32_t ant_fails;
    uint32_t oneshots;
    uint32_t overcurrent_events;
    uint32_t motion_events;
    uint32_t last_minute;
    uint32_t uptime_minutes;
    uint32_t uptime_hours;
    uint32_t uptime_days;
    uint16_t oneshot_seconds;
    char cell_iccid[40];
    char cell_cpsi[128];
    char battery[64];
    char module_lora[64];
    char module_fona[64];
    uint32_t errors_opc;
    uint32_t errors_pms;
    uint32_t errors_bme0;
    uint32_t errors_bme1;
    uint32_t errors_lora;
    uint32_t errors_fona;
    uint32_t errors_geiger;
    uint32_t errors_max01;
    uint32_t errors_ugps;
    uint32_t errors_twi;
    char errors_twi_info[128];
    uint32_t errors_lis;
    uint32_t errors_spi;
    uint32_t errors_connect_lora;
    uint32_t errors_connect_fona;
    uint32_t errors_connect_gateway;
    uint32_t errors_connect_wireless;
    uint32_t errors_connect_data;
    uint32_t errors_connect_service;
    uint32_t mtu_failures;
    uint32_t seqno;
};
typedef struct stats_s stats_t;

// Exports
stats_t *stats();
void stats_update();
void stats_status_check(bool fVerbose);
void stats_io(uint16_t transmitted, uint16_t received);

#endif // STATS_H__
