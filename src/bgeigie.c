// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

// bGeigie-to-TTServe-via-TTNode Relay state machine processing

#ifdef BGEIGIE

#include <stdio.h>
#include <stdlib.h>
#include "debug.h"
#include "config.h"
#include "comm.h"
#include "send.h"
#include "lora.h"
#include "misc.h"
#include "timer.h"
#include "gpio.h"
#include "io.h"
#include "serial.h"
#include "twi.h"
#include "storage.h"
#include "nrf_delay.h"
#include "tt.pb.h"
#include "pb_encode.h"
#include "pb_decode.h"

// Device states
#define COMM_BGEIGIE_UNKNOWN        COMM_STATE_DEVICE_START+0
#define COMM_BGEIGIE_XMIT           COMM_STATE_DEVICE_START+1

// Command buffer
static cmdbuf_t fromGeigie;

// Suppression
static uint32_t lastGeigieTransmitTime = 0L;
static uint32_t geigieSuppressionSeconds = BGEIGIE_SUPPRESSION_SECONDS;

// Process a completed bGeigie command
void bgeigie_complete() {

    // If we're not in an idle state, let's not process commands for it
    if (fromGeigie.state != COMM_STATE_IDLE)
        return;

    // Process incoming from Instrument that we did or didn't expect in an IDLE state
    if (comm_cmdbuf_this_arg_is(&fromGeigie, "$*")) {
        // Leave nextarg set up as the entire first argument to the NMEA string, ready for parsing.
        comm_cmdbuf_this_arg_is(&fromGeigie, "*");
        fromGeigie.state = COMM_BGEIGIE_XMIT;
    } else {
        fromGeigie.state = COMM_BGEIGIE_UNKNOWN;
    }

}

// One-time init
void bgeigie_init() {
    comm_cmdbuf_init(&fromGeigie, CMDBUF_TYPE_BGEIGIE);
    comm_cmdbuf_set_state(&fromGeigie, COMM_STATE_IDLE);
}

// Primary state processing of the command buffer
void bgeigie_process() {
    uint8_t buffer[CMD_MAX_LINELENGTH];

    // If it's not complete, just exit.
    if (!fromGeigie.complete)
        return;

    // If we're idle, set the state based on buffer contents
    if (fromGeigie.state == COMM_STATE_IDLE)
        bgeigie_complete();

    switch (fromGeigie.state) {

    case COMM_BGEIGIE_UNKNOWN: {
        DEBUG_PRINTF("instr ?? %s\n", &fromGeigie.buffer[fromGeigie.args]);
        comm_cmdbuf_set_state(&fromGeigie, COMM_STATE_IDLE);
        break;
    }

    case  COMM_BGEIGIE_XMIT: {
        uint32_t DeviceID;
        char *DeviceModel = comm_cmdbuf_next_arg(&fromGeigie);
        comm_cmdbuf_this_arg_is(&fromGeigie, "*");
        char *DeviceIDString = comm_cmdbuf_next_arg(&fromGeigie);
        comm_cmdbuf_this_arg_is(&fromGeigie, "*");
        char *DateTimeISO = comm_cmdbuf_next_arg(&fromGeigie);
        comm_cmdbuf_this_arg_is(&fromGeigie, "*");
        char *RadiationCPM = comm_cmdbuf_next_arg(&fromGeigie);
        comm_cmdbuf_this_arg_is(&fromGeigie, "*");
        char *Radiation5S = comm_cmdbuf_next_arg(&fromGeigie);
        comm_cmdbuf_this_arg_is(&fromGeigie, "*");
        char *RadiationTotal = comm_cmdbuf_next_arg(&fromGeigie);
        comm_cmdbuf_this_arg_is(&fromGeigie, "*");
        char *RadiationiValid = comm_cmdbuf_next_arg(&fromGeigie);
        comm_cmdbuf_this_arg_is(&fromGeigie, "*");
        char *Latitude = comm_cmdbuf_next_arg(&fromGeigie);
        comm_cmdbuf_this_arg_is(&fromGeigie, "*");
        char *LatitudeNS = comm_cmdbuf_next_arg(&fromGeigie);
        comm_cmdbuf_this_arg_is(&fromGeigie, "*");
        char *Longitude = comm_cmdbuf_next_arg(&fromGeigie);
        comm_cmdbuf_this_arg_is(&fromGeigie, "*");
        char *LongitudeEW = comm_cmdbuf_next_arg(&fromGeigie);
        comm_cmdbuf_this_arg_is(&fromGeigie, "*");
        char *Altitude = comm_cmdbuf_next_arg(&fromGeigie);
        comm_cmdbuf_this_arg_is(&fromGeigie, "*");
        char *GPSValid = comm_cmdbuf_next_arg(&fromGeigie);
        comm_cmdbuf_this_arg_is(&fromGeigie, "*");
        char *NumSat = comm_cmdbuf_next_arg(&fromGeigie);
        comm_cmdbuf_this_arg_is(&fromGeigie, "*");
        char *HDOP = comm_cmdbuf_next_arg(&fromGeigie);
        comm_cmdbuf_this_arg_is(&fromGeigie, "*");
        uint32_t SuppressionSeconds;
        UNUSED_VARIABLE(DeviceModel);
        UNUSED_VARIABLE(Radiation5S);
        UNUSED_VARIABLE(RadiationTotal);
        UNUSED_VARIABLE(RadiationiValid);
        UNUSED_VARIABLE(NumSat);
        UNUSED_VARIABLE(HDOP);

        // Onliy do all this work if communicating wouldn't be pointless
        if (!comm_is_busy()) {

            // See if we should suppress it, else transmit it
            SuppressionSeconds = geigieSuppressionSeconds + io_get_random(DESYNCHRONIZATION_SECONDS);

            if (!ShouldSuppress(&lastGeigieTransmitTime, SuppressionSeconds)) {
                uint16_t status;
                float fLatitude, fLongitude;

                // Allocate space on the stack to store the message data.
                ttproto_Telecast message = ttproto_Telecast_init_zero;

                /* Create a stream that will write to our buffer. */
                pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

                /* Build the message */
                message.has_device_type = true;
                message.device_type = ttproto_Telecast_deviceType_BGEIGIE_NANO;

                message.has_device_id = true;
                DeviceID = atol(DeviceIDString);
                message.device_id = DeviceID;

                message.has_captured_at = true;
                strlcpy(message.captured_at, DateTimeISO, sizeof(message.captured_at));

                message.has_lnd_7318u = true;
                message.lnd_7318u = atoi(RadiationCPM);

                fLatitude = GpsEncodingToDegrees(Latitude, LatitudeNS);
                fLongitude = GpsEncodingToDegrees(Longitude, LongitudeEW);
                if (fLatitude != 0 && fLongitude != 0 && GPSValid) {
                    message.latitude = fLatitude;
                    message.longitude = fLongitude;
                    message.altitude = atoi(Altitude);
                    message.has_latitude = message.has_longitude = message.has_altitude = true;
                }

                /* Encode and transmit the message */
                status = pb_encode(&stream, ttproto_Telecast_fields, &message);
                if (!status)
                    DEBUG_PRINTF("pb_encode: %s\n", PB_GET_ERROR(&stream));
                else {
                    if (send_to_service(buffer, stream.bytes_written, REPLY_NONE, SEND_1))
                        DEBUG_PRINTF("bGeigie #%lu reports %scpm\n", DeviceID, RadiationCPM);
                }

            }

        }

        // Done
        comm_cmdbuf_set_state(&fromGeigie, COMM_STATE_IDLE);
        break;

    }

    } //switch

} // bgeigie_process()

// Process byte received from bGeigie
void bgeigie_received_byte(uint8_t databyte) {
    comm_cmdbuf_received_byte(&fromGeigie, databyte);
}

#endif // BGEIGIE
