// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

#ifndef COMM_FONA_H__
#define COMM_FONA_H__
#ifdef FONA

bool fona_can_send_to_service();
bool fona_is_busy();
void fona_watchdog_reset();
void fona_gps_update();
void fona_gps_shutdown();
uint16_t fona_gps_get_value(float *lat, float *lon, float *alt);
bool fona_needed_to_be_reset();
bool fona_send_to_service(uint8_t *buffer, uint16_t length, uint16_t RequestType);
void fona_request_full_reset();
void fona_init();
void fona_term(bool fPowerdown);
void fona_process();
void fona_process_deferred();
void fona_request_state();
void fona_reset(bool Force);
void fona_send(char *msg);
void fona_received_byte(uint8_t databyte);
uint16_t fona_get_mtu();

#endif // FONA
#endif // COMM_FONA_H__
