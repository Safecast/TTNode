// Fona 3G communications definitions

#ifndef COMM_FONA_H__
#define COMM_FONA_H__
#ifdef FONA

// Public
bool fona_can_send_to_service();
bool fona_is_busy();
void fona_watchdog_reset();
void fona_gps_update();
void fona_gps_shutdown();
uint16_t fona_gps_get_value(float *lat, float *lon, float *alt);
bool fona_needed_to_be_reset();
bool fona_send_to_service(uint8_t *buffer, uint16_t length, uint16_t RequestType, uint16_t RequestFormat);
void fona_init();
void fona_term(bool fPowerdown);
void fona_process();
void fona_process_deferred();
void fona_request_state();
void fona_reset(bool Force);
void fona_send(char *msg);
void fona_received_byte(uint8_t databyte);

#endif // FONA
#endif // COMM_FONA_H__
