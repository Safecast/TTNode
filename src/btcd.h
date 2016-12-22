// CCCD configuration definitions

#ifndef CCCD_H__
#define CCCD_H__

// Public methods
uint32_t cccd_configure(uint16_t conn_handle, uint16_t handle_cccd, bool enable);
void cccd_tx_buffer_process(void);

#endif // CCCD_H__

