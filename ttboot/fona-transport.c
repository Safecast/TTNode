//
// Fona Bootloader Transport
//

#ifdef DFUFONA

#include <stddef.h>
#include "sdk_common.h"
#include "nrf_dfu_req_handler.h"
#include "nrf_dfu_transport.h"
#include "nrf_dfu_mbr.h"
#include "nrf_bootloader_info.h"
#include "boards.h"
#include "nrf_log.h"
#include "app_timer.h"
#include "softdevice_handler_appsh.h"
#include "nrf_log.h"
#include "nrf_delay.h"

#include "fona-dfu.h"

DFU_TRANSPORT_REGISTER(nrf_dfu_transport_t const dfu_trans) =
{
    .init_func =        fona_dfu_transport_init,
    .close_func =       fona_dfu_transport_close
};




uint32_t fona_dfu_transport_init(void)
{
    uint32_t err_code = NRF_SUCCESS;

    return err_code;
}


uint32_t fona_dfu_transport_close(void)
{
    uint32_t err_code = NRF_SUCCESS;

    return err_code;
}

#endif // DFUFONA
