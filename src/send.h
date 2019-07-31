// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

#ifndef SEND_H__
#define SEND_H__

// Buffered I/O header formats.  Note that although we are now starting with version number 0, we
// special case version number 8 because of the old style "single protocl buffer" message format that
// always begins with 0x08. (see ttserve/main.go)
#define BUFF_FORMAT_PB_ARRAY        0
#define BUFF_FORMAT_SINGLE_PB       8

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
#define UPDATE_STATS_MODULES    14
#define UPDATE_STATS_ERRORS     15
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

// Misc
bool send_mtu_test_in_progress();
void send_mtu_test(uint16_t start_length);
void mtu_status_check(bool fForce);
bool send_buff_is_full(uint16_t anticipated);
bool send_buff_is_empty();

#endif // SEND_H__
