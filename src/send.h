// Transmit to service

#ifndef SEND_H__
#define SEND_H__

#define UPDATE_NORMAL           0
#define UPDATE_STATS            1
#define UPDATE_STATS_VERSION    2
#define UPDATE_STATS_CONFIG_DEV 3
#define UPDATE_STATS_CONFIG_SVC 4
#define UPDATE_STATS_CONFIG_GPS 5
#define UPDATE_STATS_CONFIG_SEN 6
#define UPDATE_STATS_CELL1      7
#define UPDATE_STATS_CELL2      8
#define UPDATE_STATS_DFU        9

bool send_update_to_service(uint16_t UpdateType);

#define REPLY_NONE         0
#define REPLY_TTGATE       1
#define REPLY_TTSERVE      2
bool send_to_service(char *what, uint8_t *buffer, uint16_t length, uint16_t RequestType);
bool send_to_service_unconditionally(char *what, uint8_t *buffer, uint16_t length, uint16_t RequestType);
bool send_ping_to_service(uint16_t RequestType);

void stats_update();
void stats_set_cell_info(char *iccid, char *cpsi);
void stats_add(uint16_t transmitted, uint16_t received, uint16_t resets, uint16_t powerfails, uint16_t oneshots, uint16_t motiondrops);

#endif // SEND_H__

