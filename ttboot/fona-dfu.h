
#ifndef FONA_DFU_H__
#define FONA_DFU_H__

void fona_dfu_init();
void fona_dfu_schedule_kickoff();

uint32_t fona_dfu_transport_init(void);
uint32_t fona_dfu_transport_close(void);

#endif // FONA_DFU_H__
