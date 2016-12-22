// bGeigie state machine processing

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
#include "teletype.pb.h"
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
        uint32_t DeviceIDNumber;
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
                teletype_Telecast message = teletype_Telecast_init_zero;

                /* Create a stream that will write to our buffer. */
                pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

                /* Build the message */
                message.DeviceType = teletype_Telecast_deviceType_BGEIGIE_NANO;

                message.has_DeviceIDNumber = true;
                DeviceIDNumber = atol(DeviceIDString);
                message.DeviceIDNumber = DeviceIDNumber;

                message.has_CapturedAt = true;
                strncpy(message.CapturedAt, DateTimeISO, sizeof(message.CapturedAt));

                message.has_Unit = true;
                message.Unit = teletype_Telecast_unit_CPM;

                message.has_Value = true;
                message.Value = atoi(RadiationCPM);

                fLatitude = GpsEncodingToDegrees(Latitude, LatitudeNS);
                fLongitude = GpsEncodingToDegrees(Longitude, LongitudeEW);
                if (fLatitude != 0 && fLongitude != 0 && GPSValid) {
                    message.Latitude = fLatitude;
                    message.Longitude = fLongitude;
                    message.Altitude = atoi(Altitude);
                    message.has_Latitude = message.has_Longitude = message.has_Altitude = true;
                }

                /* Encode and transmit the message */
                status = pb_encode(&stream, teletype_Telecast_fields, &message);
                if (!status)
                    DEBUG_PRINTF("pb_encode: %s\n", PB_GET_ERROR(&stream));
                else {
                      if (send_to_service("bGeigie", buffer, stream.bytes_written, REPLY_NONE))
                          DEBUG_PRINTF("bGeigie #%lu reports %scpm\n", DeviceIDNumber, RadiationCPM);
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
