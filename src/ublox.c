// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

// UBLOXM8 GPS sensor

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "debug.h"
#include "boards.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_drv_twi.h"
#include "app_util_platform.h"
#include "gpio.h"
#include "config.h"
#include "timer.h"
#include "sensor.h"
#include "storage.h"
#include "comm.h"
#include "misc.h"
#include "twi.h"
#include "io.h"

#ifdef TWIUBLOXM8

// UBLOXM8 Data Sheet
// http://https://www.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_(UBX-13003221)_Public.pdf
// http://www.recoon5.ru/en/story/u-blox_EVK-7N_I2C

// UBLOXM8 7-Bit I2C Address
#define UBLOXM8_I2C_ADDRESS     0x42

// UBLOXM8 commands
// Addresses FD and FE contain the length that's readable at FF
#define UBLOXM8_GETLEN_ADDR     0xFD
#define UBLOXM8_GETLEN_LEN      2
#define UBLOXM8_GETDATA_ADDR    0xFF
#define UBLOXM8_ADDRESS_LEN     1
// Nordic SDK has max transfer of 255 bytes, even though GPS can return much more
#define UBLOXM8_MAX_DATALEN     250

// I/O buffer
typedef struct {
    // Register address for the UBLOX
    uint8_t address[UBLOXM8_ADDRESS_LEN];
    // Buffer that receives the length data
    uint8_t buffer[UBLOXM8_MAX_DATALEN];
    // Request from the outside
    bool gpsShutdown;
    // Measurement context
    bool haveLocation;
    bool haveFullLocation;
    uint32_t gpsDataAttempts;
    uint32_t gpsDataParsed;
    float gpsLatitude;
    float gpsLongitude;
    float gpsAltitude;
} ubloxm8_data_t;
static ubloxm8_data_t ioGPS;

// TWI initialization
static bool fTWIInit = false;

// Callback that's activated when TWI data has been read.
// Unlike other callbacks, GPS does its transactions inside the poller, and ensures
// that the measured data is always available in its context structure.
void gps_callback(ret_code_t result, twi_context_t *t) {
    uint16_t i, j;
#ifdef GPSDEBUG
    static uint32_t lastDebugOutput = 0;
    int d;
    bool doDebugOutput = false;
    char DebugOutput1[32], DebugOutput2[32];
    DebugOutput1[0] = DebugOutput2[0] = '\0';
#endif

    // Mark as completed
    if (!twi_completed(t))
        return;
    
    // Remember the number of times that we attempted to fetch data from the GPS
    ioGPS.gpsDataAttempts++;

    // Parse the buffer, looking for either of these.  (We'd rather have gpgga because it has altitude, but we may not lock)
    //  $GPGGA,<time>,<latitude>,<ns>,<longitude>,<ew>,<quality>,<numsat>,<hdop>,<alt>,<m>
    //  $GPGLL,<latitude>,<ns>,<longitude>,<ew>
    // https://www.u-blox.com/sites/default/files/products/documents/u-blox7-V14_ReceiverDescrProtSpec_(GPS.G7-SW-12001)_Public.pdf

    for (i = 0; i < sizeof(ioGPS.buffer); i++) {

        // Done?  This is the expected delimeter.
        if (ioGPS.buffer[i] == 0xff)
            break;

        // Remember that we've actually looked at some data.
        ioGPS.gpsDataParsed++;
        gpio_indicate(INDICATE_GPS_CONNECTING);

        // Look for the beginning of the $GPGGA NMEA string
        if ((ioGPS.buffer[i] == '$') && ((sizeof(ioGPS.buffer) - i) >= 6) && (memcmp(&ioGPS.buffer[i], "$GPGGA", 6) == 0)) {
            char *lat, *ns, *lon, *ew, *alt, *fix;
            bool haveLat, haveLon, haveNS, haveEW, haveAlt, haveFix;
            uint16_t commas;

#ifdef GPSDEBUG
            for (d=0; d<sizeof(DebugOutput1)-1; d++)
                if (ioGPS.buffer[i+d] < ' ')
                    DebugOutput1[d] = '\0'; 
                else
                    DebugOutput1[d] = ioGPS.buffer[i+d];
            DebugOutput1[sizeof(DebugOutput1)-1] = '\0';
            doDebugOutput = true;
#endif

            commas = 0;
            haveLat = haveLon = haveNS = haveEW = haveAlt = haveFix = false;
            lat = ns = lon = ew = alt = fix = "";

            for (j = i; j < sizeof(ioGPS.buffer); j++)
                if (ioGPS.buffer[j] == ',') {
                    ioGPS.buffer[j] = '\0';
                    commas++;
                    if (commas == 1) {
                        // Sitting at comma before Time
                    }
                    if (commas == 2) {
                        // Sitting at comma before Lat
                        lat = (char *) &ioGPS.buffer[j + 1];
                    }
                    if (commas == 3) {
                        // Sitting at comma before NS
                        haveLat = (lat[0] != '\0');
                        ns = (char *) &ioGPS.buffer[j + 1];
                    }
                    if (commas == 4) {
                        // Sitting at comma before Lon
                        haveNS = (ns[0] != '\0');
                        lon = (char *) &ioGPS.buffer[j + 1];
                    }
                    if (commas == 5) {
                        // Sitting at comma before EW
                        haveLon = (lon[0] != '\0');
                        ew = (char *) &ioGPS.buffer[j + 1];
                    }
                    if (commas == 6) {
                        // Sitting at comma before Quality
                        haveEW = (ew[0] != '\0');
                        fix = (char *) &ioGPS.buffer[j + 1];
                    }
                    if (commas == 7) {
                        // Sitting at comma before NumSat
                        // 1 is valid GPS fix, 2 is valid DGPS fix
                        haveFix = (fix[0] == '1' || fix[0] == '2');
                    }
                    if (commas == 8) {
                        // Sitting at comma before Hdop
                    }
                    if (commas == 9) {
                        // Sitting at comma before Alt
                        alt = (char *) &ioGPS.buffer[j + 1];
                    }
                    if (commas == 10) {
                        // Sitting at comma before M unit
                        haveAlt = (alt[0] != '\0');
                        break;
                    }
                }

            // If we've got what we need, process it and exit.
            if (haveFix && haveLat && haveNS & haveLon && haveEW) {
                float fLatitude = GpsEncodingToDegrees(lat, ns);
                float fLongitude = GpsEncodingToDegrees(lon, ew);
                if (fLatitude != 0 || fLongitude != 0) {
                    ioGPS.haveLocation = true;
                    ioGPS.gpsLatitude = fLatitude;
                    ioGPS.gpsLongitude = fLongitude;
                    if (haveAlt) {
                        ioGPS.gpsAltitude = atof(alt);
                        ioGPS.haveFullLocation = true;
                    }
                }
            }

        }   // if GPGGA

        // Look for the beginning of the $GPGLL NMEA string
        if ((ioGPS.buffer[i] == '$') && ((sizeof(ioGPS.buffer) - i) >= 6) && (memcmp(&ioGPS.buffer[i], "$GPGLL", 6) == 0)) {
            char *lat, *ns, *lon, *ew;
            bool haveLat, haveLon, haveNS, haveEW;
            uint16_t commas;

#ifdef GPSDEBUG
            for (d=0; d<sizeof(DebugOutput2)-1; d++)
                if (ioGPS.buffer[i+d] < ' ')
                    DebugOutput2[d] = '\0'; 
                else
                    DebugOutput2[d] = ioGPS.buffer[i+d];
            DebugOutput2[sizeof(DebugOutput2)-1] = '\0';
            doDebugOutput = true;
#endif

            commas = 0;
            haveLat = haveLon = haveNS = haveEW = false;
            lat = ns = lon = ew = "";

            for (j = i; j < sizeof(ioGPS.buffer); j++)
                if (ioGPS.buffer[j] == ',') {
                    ioGPS.buffer[j] = '\0';
                    commas++;
                    if (commas == 1) {
                        // Sitting at comma before Lat
                        lat = (char *) &ioGPS.buffer[j + 1];
                    }
                    if (commas == 2) {
                        // Sitting at comma before NS
                        haveLat = (lat[0] != '\0');
                        ns = (char *) &ioGPS.buffer[j + 1];
                    }
                    if (commas == 3) {
                        // Sitting at comma before Lon
                        haveNS = (ns[0] != '\0');
                        lon = (char *) &ioGPS.buffer[j + 1];
                    }
                    if (commas == 4) {
                        // Sitting at comma before EW
                        haveLon = (lon[0] != '\0');
                        ew = (char *) &ioGPS.buffer[j + 1];
                    }
                    if (commas == 5) {
                        // done
                        haveEW = (ew[0] != '\0');
                        break;
                    }
                }

            // If we've got what we need, process it and exit.
            if (haveLat && haveNS & haveLon && haveEW) {
                float fLatitude = GpsEncodingToDegrees(lat, ns);
                float fLongitude = GpsEncodingToDegrees(lon, ew);
                if (fLatitude != 0 || fLongitude != 0) {
                    ioGPS.haveLocation = true;
                    ioGPS.gpsLatitude = fLatitude;
                    ioGPS.gpsLongitude = fLongitude;
                }
            }

        }   // if GPGLL

    }   // Loop over iobuf

#ifdef GPSDEBUG
    if (ioGPS.gpsDataParsed == 0) {
        if (!ShouldSuppress(&lastDebugOutput, 5)) {
            DEBUG_PRINTF("No GPS data\n");
        }
    } else if (doDebugOutput) {
        if (!ShouldSuppress(&lastDebugOutput, 5)) {
            char DebugOutput[128];
            sprintf(DebugOutput, "GPS: %s %s\n", DebugOutput1, DebugOutput2);
            DEBUG_PRINTF(DebugOutput);
        }    
    }
#endif

}   // Notification of TWI result

void s_gps_shutdown() {
    gpio_indicator_no_longer_needed(GPS);
    ioGPS.gpsShutdown = true;
}

// GPS poller
void s_gps_poll(void *s) {

    // Exit if we're not supposed to be here
    if (!sensor_is_polling_valid(s))
        return;

    // If we've already got the full location, terminate the polling just to save battery life
    if ((comm_gps_get_value(NULL, NULL, NULL) == GPS_LOCATION_FULL) || ioGPS.gpsShutdown) {
        sensor_measurement_completed(s);
        DEBUG_PRINTF("TWI GPS acquired.\n");
        return;
    }

    // If the GPS hardware isn't even present, terminate the polling to save battery life.
    if (ioGPS.gpsDataAttempts > 100 && ioGPS.gpsDataParsed == 0) {
        sensor_measurement_completed(s);
        DEBUG_PRINTF("TWI GPS shutdown. (not found)\n");
        return;
    }

    // Make sure it appears that we are connecting to GPS
    gpio_indicate(INDICATE_GPS_CONNECTING);

    // This assumes, as in the ublox spec in 10.5.1.1, that the default address pointer at startup is 0xff, so all we need to do
    // is to start reading.  Also, prefill the buffer with ff because that represents "no more data" for the ublox.
    memset(ioGPS.buffer, 0xff, sizeof(ioGPS.buffer));
    ioGPS.address[0] = UBLOXM8_GETDATA_ADDR;
    static app_twi_transfer_t transfers[] = {
        APP_TWI_WRITE(UBLOXM8_I2C_ADDRESS, &ioGPS.address[0], UBLOXM8_ADDRESS_LEN, APP_TWI_NO_STOP),
        APP_TWI_READ(UBLOXM8_I2C_ADDRESS, &ioGPS.buffer[0], sizeof(ioGPS.buffer), 0)
    };
    static app_twi_transaction_t transaction = {
        .callback            = twi_callback,
        .p_user_data         = "UBLOX",
        .p_transfers         = transfers,
        .number_of_transfers = sizeof(transfers) / sizeof(transfers[0])
    };
    if (!twi_schedule(s, gps_callback, &transaction))
        sensor_unconfigure(s);
}

// Init GPS upon power-up
bool s_gps_init(void *s, uint16_t param) {
    ioGPS.haveLocation = false;
    ioGPS.haveFullLocation = false;
    ioGPS.gpsShutdown = false;
    ioGPS.gpsDataAttempts = 0;
    ioGPS.gpsDataParsed = 0;
    ioGPS.gpsLatitude = 0.0;
    ioGPS.gpsLongitude = 0.0;
    ioGPS.gpsAltitude = 0.0;

    if (!twi_init())
        return false;
    fTWIInit = true;
    
    return true;

}

// Init GPS
bool s_gps_term() {
    if (fTWIInit) {
        twi_term();
        fTWIInit = false;
    }
    return true;
}

// The main access method for our data
uint16_t s_gps_get_value(float *lat, float *lon, float *alt) {
    if (!ioGPS.haveLocation)
        return (ioGPS.gpsDataParsed == 0 ? GPS_NO_DATA : GPS_NO_LOCATION);
    if (lat != NULL)
        *lat = ioGPS.gpsLatitude;
    if (lon != NULL)
        *lon = ioGPS.gpsLongitude;
    if (alt != NULL)
        *alt = ioGPS.gpsAltitude;
    return (ioGPS.haveFullLocation ? GPS_LOCATION_FULL : GPS_LOCATION_PARTIAL);
}

#endif // TWIUBLOXM8
