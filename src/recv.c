// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

// Process received messages from TTServe

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "timer.h"
#include "debug.h"
#include "config.h"
#include "storage.h"
#include "sensor.h"
#include "comm.h"
#include "phone.h"
#include "misc.h"
#include "io.h"

// Commands
#define CMD_RESTART "restart"
#define CMD_REBOOT "reboot"
#define CMD_HELLO "hello"
#define CMD_BURN "burn"
#define CMD_BURNWITHMINS "burn "
#define CMD_CFGDEV "cfgdev "
#define CMD_CFGNET "cfgsvc "
#define CMD_CFGTTN "cfgttn "
#define CMD_CFGLAB "cfglab "
#define CMD_CFGGPS "cfggps "
#define CMD_CFGSEN "cfgsen "
#define CMD_CFGDFU "cfgdfu "
#define CMD_DFU "dfu"
#define CMD_DFUWITHCFG "dfu "
#define CMD_DOWN "down"

// Process a received message from the service
void recv_message_from_service(char *message) {
    DEBUG_PRINTF("RECEIVED: %s\n", message);
    if (memcmp(message, CMD_CFGDEV, strlen(CMD_CFGDEV)) == 0) {
        storage_set_device_params_as_string(&message[strlen(CMD_CFGDEV)]);
    } else if (memcmp(message, CMD_CFGGPS, strlen(CMD_CFGGPS)) == 0) {
        storage_set_gps_params_as_string(&message[strlen(CMD_CFGGPS)]);
    } else if (memcmp(message, CMD_CFGNET, strlen(CMD_CFGNET)) == 0) {
        storage_set_service_params_as_string(&message[strlen(CMD_CFGNET)]);
    } else if (memcmp(message, CMD_CFGTTN, strlen(CMD_CFGTTN)) == 0) {
        storage_set_ttn_params_as_string(&message[strlen(CMD_CFGTTN)]);
    } else if (memcmp(message, CMD_CFGLAB, strlen(CMD_CFGLAB)) == 0) {
        storage_set_device_label_as_string(&message[strlen(CMD_CFGLAB)]);
    } else if (memcmp(message, CMD_CFGSEN, strlen(CMD_CFGSEN)) == 0) {
        storage_set_sensor_params_as_string(&message[strlen(CMD_CFGSEN)]);
    } else if (memcmp(message, CMD_CFGDFU, strlen(CMD_CFGDFU)) == 0) {
        storage_set_dfu_state_as_string(&message[strlen(CMD_CFGDFU)]);
    } else if (memcmp(message, CMD_DFU, strlen(CMD_DFU)) == 0) {
#if !defined(DFU) || !defined(FONA)
        DEBUG_PRINTF("DFU Requested, but firmware is not configured for DFU\n");
        return;
#else
        if (memcmp(message, CMD_DFUWITHCFG, strlen(CMD_DFUWITHCFG)) == 0)
            storage_set_dfu_state_as_string(&message[strlen(CMD_DFUWITHCFG)]);
        DEBUG_PRINTF("Initiating DFU of '%s'.\n", storage()->dfu_filename);
        storage()->dfu_status = DFU_PENDING;
#endif
    } else if (memcmp(message, CMD_BURN, strlen(CMD_BURN)) == 0) {
        uint32_t duration = 60L;  // Default to 1 hour
        if (memcmp(message, CMD_BURNWITHMINS, strlen(CMD_BURNWITHMINS)) == 0)
            duration = atol(&message[strlen(CMD_BURNWITHMINS)]);
        DEBUG_PRINTF("Initiating temporary BURN mode for %ld minutes.\n", duration);
        sensor_set_temporary_op_mode(OPMODE_TEST_BURN, duration * 60L);
        return;
    } else if (strcmp(message, CMD_RESTART) == 0) {
        io_request_restart();
        return;
    } else if (strcmp(message, CMD_REBOOT) == 0) {
        io_request_restart();
        return;
    } else if (strcmp(message, CMD_HELLO) == 0) {
        comm_initiate_service_update(true);
        return;
    } else if (strcmp(message, CMD_DOWN) == 0) {
        comm_force_cell();
        return;
    } else {
        DEBUG_PRINTF("TTSERVE: %s\n", (char *) message);
        return;
    }
    storage_save(true);
    io_request_restart();
}
