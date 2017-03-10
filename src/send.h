// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

#ifndef SEND_H__
#define SEND_H__

// Statistic upload modes
#define UPDATE_NORMAL           0
#define UPDATE_STATS            1
#define UPDATE_STATS_VERSION    2
#define UPDATE_STATS_CONFIG_DEV 3
#define UPDATE_STATS_CONFIG_SVC 4
#define UPDATE_STATS_CONFIG_TTN 5
#define UPDATE_STATS_CONFIG_GPS 6
#define UPDATE_STATS_CONFIG_SEN 7
#define UPDATE_STATS_CELL1      8
#define UPDATE_STATS_CELL2      9
#define UPDATE_STATS_DFU        10
#define UPDATE_STATS_MTU_TEST   11
#define UPDATE_STATS_LABEL      12
#define UPDATE_STATS_BATTERY    13
bool send_update_to_service(uint16_t UpdateType);

// Send modes
#define SEND_1             0
#define SEND_N             1
#define REPLY_NONE         0
#define REPLY_TTGATE       1
#define REPLY_TTSERVE      2
bool send_to_service(uint8_t *buffer, uint16_t length, uint16_t RequestType, uint16_t RequestFormat);
bool send_to_service_unconditionally(uint8_t *buffer, uint16_t length, uint16_t RequestType, uint16_t RequestFormat);
bool send_ping_to_service(uint16_t RequestType);

// Exports
void stats_update();
void stats_set_cell_info(char *iccid, char *cpsi);
bool stats_set_battery_info(char *info);
void stats_add(uint16_t transmitted, uint16_t received, uint16_t resets, uint16_t powerfails, uint16_t oneshots, uint16_t motiondrops);
void stats_set(uint16_t oneshot_seconds);
bool send_mtu_test_in_progress();
void send_mtu_test(uint16_t start_length);
void mtu_status_check(bool fForce);

#endif // SEND_H__

