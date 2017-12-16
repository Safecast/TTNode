// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

// Adafruit Ultimate GPS

#ifdef UGPS

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "debug.h"
#include "boards.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_drv_twi.h"
#include "app_scheduler.h"
#include "app_util_platform.h"
#include "app_twi.h"
#include "twi.h"
#include "gpio.h"
#include "config.h"
#include "timer.h"
#include "sensor.h"
#include "storage.h"
#include "comm.h"
#include "serial.h"
#include "misc.h"
#include "ugps.h"
#include "io.h"
#include "stats.h"
#include "battery.h"

// Serial I/O Buffers
#define IOBUFFERS 8
#define MAXLINE 250

typedef struct {
    uint16_t linesize;
    char linebuf[MAXLINE];
} iobuf_t;
static iobuf_t iobuf[IOBUFFERS];
static uint16_t iobuf_completed;
static uint16_t iobuf_filling;
static int completed_iobufs_available;

static float reported_latitude = 0.0;
static float reported_longitude = 0.0;
static float reported_altitude = 0.0;
static uint32_t reported_date;
static uint32_t reported_time;
static bool reported = false;
static bool reported_have_location = false;
static bool trying_to_improve_location = false;
static bool reported_have_improved_location = false;
static bool reported_have_full_location = false;
static bool reported_have_timedate = false;
static uint32_t last_sampled_loc;
static bool saved_lkg_this_session = false;

static bool initialized_ever = false;
static bool initialized = false;
static bool shutdown = false;
static uint32_t sentences_received = 0;
static uint32_t sentences_received_last_poll = 0;
static bool gps_active = false;
static uint32_t seconds = 0;
static bool skip = false;
static bool displayed_antenna_status = false;
static uint32_t last_retry = 0;

bool ugps_ok_to_update() {

    // Don't update under any circumstances if the battery is low, or if
    // we're in burn mode.  This is because a) during test we are constantly
    // tapping the unit to wake up the screen, and the GPS seek is an annoyance,
    // and b) the GPS sucks a lot of power, and this is inappropriate when
    // the battery is low.
    switch (battery_status()) {
    case BAT_BURN:
    case BAT_TEST:
    case BAT_EMERGENCY:
    case BAT_WARNING:
    case BAT_LOW:
        return false;

    }

    return true;

}

// Update net iteration
void s_ugps_update(void) {

    // Exit if it's not ok that we update GPS
    if (!ugps_ok_to_update())
        return;

    // Only do this if the GPS isn't already currently active
    if (!initialized) {
        skip = false;
        trying_to_improve_location = true;
        s_ugps_clear_measurement();
    }

}

// Whether or not the GPS is in an active state, used by mobile
bool s_ugps_active() {
    if (!initialized)
        return false;
    return gps_active;
}

// Init sensor just after each power-on
void s_ugps_done_settling() {

    // Clear out the values
    s_ugps_clear_measurement();

}

// Force the GPS to shutdown
void s_ugps_shutdown() {
    if (!shutdown && gpio_current_uart() == UART_GPS) {
        shutdown = true;
        gpio_indicator_no_longer_needed(GPS);
    }

}

// Term sensor just before each power-off
bool s_ugps_term() {

    if (!initialized)
        return false;
    initialized = false;
    return true;
}

// One-time initialization of sensor
bool s_ugps_init(void *s, uint16_t param) {

    // Exit if we're updating when it's simply inappropriate to do so
    // because of how little energy remains.
    if (initialized_ever && !ugps_ok_to_update())
        return false;

    // Defensively, exit if we're alraedy running
    if (initialized)
        return false;

    // Proceed
    completed_iobufs_available = 0;
    sentences_received = 0;
    iobuf_filling = 0;
    iobuf[0].linesize = 0;
    initialized = initialized_ever = true;
    seconds = 0;
    shutdown = false;

    // Send update rate
    DEBUG_PRINTF("s-gps initializing\n");
    serial_send_string("$PMTK300,2000,0,0,0,0*18");

    // Tell it that we'd like updates on which antenna is being used
    if (!displayed_antenna_status)
        serial_send_string("$PGCMD,33,1*6C");

    return true;
}

// Reset the buffer
void iobuf_reset() {
    iobuf[iobuf_filling].linesize = 0;
}

// Drain the completed I/O buffer and point to the next
bool iobuf_pop(iobuf_t *piobuf) {
    if (completed_iobufs_available <= 0)
        return false;
    *piobuf = iobuf[iobuf_completed];
    completed_iobufs_available--;
    if (++iobuf_completed >= IOBUFFERS)
        iobuf_completed = 0;
    return true;
}

// Process a data-received events
void gps_process_sentence(char *line, uint16_t linelen) {

    // Bump the number received
    sentences_received++;

    // For extreme debugging of GPS abort/retry logic at the comm level
#if 0
    reported_have_location = true;
    reported_latitude = 1;
    reported_longitude = 2;
    reported_altitude = 3;
    reported_have_full_location = true;
    reported_have_improved_location = true;
    trying_to_improve_location = false;
    if (!reported_have_timedate) {
        reported_time = 111111;
        reported_date = 111117;
        set_timestamp(reported_date, reported_time);
        reported_have_timedate = true;
    }
    return;
#endif

    // Process the GPS sentence
    if (debug(DBG_GPS_MAX))
        DEBUG_PRINTF("%s%s%s%s %s\n", reported_have_location ? "l" : "-", reported_have_full_location ? "L" : "-", reported_have_improved_location ? "I" : "-", reported_have_timedate ? "T" : "-", line);

    // Process $GPGGA, which should give us lat/lon/alt
    if (memcmp(line, "$GPGGA", 6) == 0) {
        int j;
        char *lat, *ns, *lon, *ew, *alt, *fix, *time;
        bool haveLat, haveLon, haveNS, haveEW, haveAlt, haveFix, haveTime;
        uint16_t commas;

        commas = 0;
        haveLat = haveLon = haveNS = haveEW = haveAlt = haveFix = haveTime = false;
        lat = ns = lon = ew = alt = time = fix = "";

        for (j = 0; j < linelen; j++)
            if (line[j] == ',') {
                line[j] = '\0';
                commas++;
                if (commas == 1) {
                    // Sitting at comma before Time
                    time = (char *) &line[j + 1];
                }
                if (commas == 2) {
                    haveTime = (time[0] != '\0');
                    // Sitting at comma before Lat
                    lat = (char *) &line[j + 1];
                }
                if (commas == 3) {
                    // Sitting at comma before NS
                    haveLat = (lat[0] != '\0');
                    ns = (char *) &line[j + 1];
                }
                if (commas == 4) {
                    // Sitting at comma before Lon
                    haveNS = (ns[0] != '\0');
                    lon = (char *) &line[j + 1];
                }
                if (commas == 5) {
                    // Sitting at comma before EW
                    haveLon = (lon[0] != '\0');
                    ew = (char *) &line[j + 1];
                }
                if (commas == 6) {
                    // Sitting at comma before Quality
                    haveEW = (ew[0] != '\0');
                    fix = (char *) &line[j + 1];
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
                    alt = (char *) &line[j + 1];
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
                last_sampled_loc++;
                reported_have_location = true;
                reported_latitude = fLatitude;
                reported_longitude = fLongitude;
                if (haveAlt) {
                    reported_altitude = atof(alt);
                    reported_have_full_location = true;
                    reported_have_improved_location = true;
                    trying_to_improve_location = false;
                }
                if (!saved_lkg_this_session) {
                    STORAGE *f = storage();
                    f->lkg_gps_latitude = reported_latitude;
                    f->lkg_gps_longitude = reported_longitude;
                    f->lkg_gps_altitude = reported_altitude;
                    storage_save(false);
                    saved_lkg_this_session = true;
                }
            }

        }

    }   // if GPGGA

    // Process $GPRMC, which should give us lat/lon and time
    if (memcmp(line, "$GPRMC", 6) == 0) {
        int j;
        char *lat, *ns, *lon, *ew, *time, *date, *valid;
        bool haveLat, haveLon, haveNS, haveEW, haveTime, haveDate, haveValid;
        uint16_t commas;

        commas = 0;
        haveLat = haveLon = haveNS = haveEW = haveTime = haveDate = haveValid = false;
        lat = ns = lon = ew = date = time = valid = "";

        for (j = 0; j < linelen; j++)
            if (line[j] == ',') {
                line[j] = '\0';
                commas++;
                if (commas == 1) {
                    // Sitting at comma before Time
                    time = (char *) &line[j + 1];
                }
                if (commas == 2) {
                    haveTime = (time[0] != '\0');
                    // Sitting at comma before Validity
                    valid = (char *) &line[j + 1];
                }
                if (commas == 3) {
                    // Sitting at comma before Lat
                    haveValid = (valid[0] == 'A');
                    lat = (char *) &line[j + 1];
                }
                if (commas == 4) {
                    // Sitting at comma before NS
                    haveLat = (lat[0] != '\0');
                    ns = (char *) &line[j + 1];
                }
                if (commas == 5) {
                    // Sitting at comma before Lon
                    haveNS = (ns[0] != '\0');
                    lon = (char *) &line[j + 1];
                }
                if (commas == 6) {
                    // Sitting at comma before EW
                    haveLon = (lon[0] != '\0');
                    ew = (char *) &line[j + 1];
                }
                if (commas == 7) {
                    // Sitting at comma before Speed
                    haveEW = (ew[0] != '\0');
                }
                if (commas == 8) {
                    // Sitting at comma before True Course
                }
                if (commas == 9) {
                    // Sitting at comma before Date
                    date = (char *) &line[j + 1];
                }
                if (commas == 10) {
                    // Sitting at comma before Variation
                    haveDate = (date[0] != '\0');
                    break;
                }
            }

        // If we've got lat/lon, process it
        if (haveValid && haveLat && haveNS & haveLon && haveEW) {
            float fLatitude = GpsEncodingToDegrees(lat, ns);
            float fLongitude = GpsEncodingToDegrees(lon, ew);
            if (fLatitude != 0 || fLongitude != 0) {
                last_sampled_loc++;
                reported_have_location = true;
                reported_latitude = fLatitude;
                reported_longitude = fLongitude;
                reported_have_improved_location = true;
                trying_to_improve_location = false;
            }
            if (!saved_lkg_this_session) {
                STORAGE *f = storage();
                f->lkg_gps_latitude = reported_latitude;
                f->lkg_gps_longitude = reported_longitude;
                f->lkg_gps_altitude = 0;
                storage_save(false);
                saved_lkg_this_session = true;
            }
        }

        // If we've got what we need, process it and exit.
        if (!reported_have_timedate && haveValid && haveTime && haveDate) {
            reported_time = atol(time);
            reported_date = atol(date);
            // Do one final check to make sure that the date is valid.
            // We do this because we've seen dates of 1980 being reported
            // by GPS chips, and it's reasonable to bracket valid year values.
            // If this source code lasts beyond this range, I'll be very happy
            // if you relax this check because maybe chips will function
            // properly by then :-)
            float reported_year = reported_date % 100;
            if (reported_year >= 17 && reported_year < 50) {
                set_timestamp(reported_date, reported_time);
                reported_have_timedate = true;
            } else {
                DEBUG_PRINTF("GPS: Invalid year: %.0f\n", reported_year);
            }
        }

    }   // if GPRMC

    // Process $PGTOP, which tells us which antenna is being used
    if (memcmp(line, "$PGTOP", 6) == 0) {
        int j;
        char *arg1, *arg2;
        bool haveArg1, haveArg2;
        uint16_t commas;

        commas = 0;
        haveArg1 = haveArg2 = false;
        arg1 = arg2 = "";

        for (j = 0; j < linelen; j++)
            if (line[j] == ',' || line[j] == '*') {
                line[j] = '\0';
                commas++;
                if (commas == 1) {
                    // Sitting at comma before Arg1
                    arg1 = (char *) &line[j + 1];
                }
                if (commas == 2) {
                    // Sitting at comma before Arg2
                    haveArg1 = (arg1[0] != '\0');
                    arg2 = (char *) &line[j + 1];
                }
                if (commas == 3) {
                    // Sitting at asterisk before checksum
                    haveArg2 = (arg2[0] != '\0');
                    break;
                }
            }

        // If we've got what we need, process it and exit.
        UNUSED_VARIABLE(haveArg1);
        if (haveArg2) {
            if (!displayed_antenna_status) {
                displayed_antenna_status = true;
                switch (arg2[0]) {
                case '1':
                    DEBUG_PRINTF("GPS antenna failure.\n");
                    break;
                case '2':
                    DEBUG_PRINTF("GPS using internal antenna.\n");
                    break;
                case '3':
                    DEBUG_PRINTF("GPS using external antenna.\n");
                    break;
                }
            }
        }

    }   // if $PGTOP

}

// Process a data-received events
void line_event_handler(void *unused1, uint16_t unused2) {
    iobuf_t iobuf_popped;
    if (!initialized)
        return;
    while (iobuf_pop(&iobuf_popped))
        gps_process_sentence(iobuf_popped.linebuf, iobuf_popped.linesize);
}

// Bump to the next I/O buffer, dropping the line if we overflow I/O buffers
bool iobuf_push() {
    bool dropped = false;

    if (completed_iobufs_available < IOBUFFERS) {
        if (++iobuf_filling >= IOBUFFERS)
            iobuf_filling = 0;
        iobuf_reset();
        completed_iobufs_available++;
        app_sched_event_put(NULL, 0, line_event_handler);
    } else {
        iobuf_reset();
        dropped = true;
    }

    return !dropped;
}

// Process byte received from gps
void s_ugps_received_byte(uint8_t databyte) {

    // Discard results if we happen to get here if not yet initialized
    if (!initialized)
        return;

    // If we get here, we're expecting to receive an ASCII text command terminated in \r\n.
    // If this is the CR of the CRLF sequence, skip it.
    if (databyte == '\r')
        return;

    // If this the final byte of the CRLF sequence, process it
    if (databyte == '\n') {

        // If it's just a blank line (or the second char of \r\n), skip it
        if (iobuf[iobuf_filling].linesize == 0)
            return;

        // Null terminate the line because it's a string
        iobuf[iobuf_filling].linebuf[iobuf[iobuf_filling].linesize] = '\0';

        // Enqueue it
        iobuf_push();

        return;
    }

    // If somehow a non-text character got here, substitute.
    if (databyte < 0x20 || databyte > 0x7f)
        databyte = '.';

    // Add the char to the line buffer
    if (iobuf[iobuf_filling].linesize < (sizeof(iobuf[iobuf_filling].linebuf)-2))
        iobuf[iobuf_filling].linebuf[iobuf[iobuf_filling].linesize++] = (char) databyte;

}

// Clear the values
void s_ugps_clear_measurement() {
    reported_have_improved_location = false;
}

// Group skip handler
bool g_ugps_skip(void *g) {

    // Skip if we've got a static value
    if (!reported && comm_gps_get_value(NULL, NULL, NULL) == GPS_LOCATION_FULL)
        return true;

    // Don't skip if we'd aborted and it's time for an hourly retry
    if (sensor_op_mode() != OPMODE_TEST_BURN)
        if (comm_gps_get_value(NULL, NULL, NULL) == GPS_LOCATION_ABORTED) {
            if (!ShouldSuppress(&last_retry, GPS_RETRY_MINUTES*60)) {
                reported = false;
                skip = false;
                return false;
            } else {
                if (debug(DBG_GPS_MAX))
                    DEBUG_PRINTF("Need to resample aborted GPS\n");
            }
        }

    // Don't let the GPS sleep if we're in mobile mode
    if (sensor_op_mode() == OPMODE_MOBILE)
        return false;

    // Skip if we've already reported
    return(skip);
}

// Poller
void s_ugps_poll(void *s) {

    // Exit if we're not supposed to be here
    if (!sensor_is_polling_valid(s))
        return;

    // Keep track of active/inactive
    if (sentences_received == sentences_received_last_poll)
        gps_active = false;
    else {
        gps_active = true;
        sentences_received_last_poll = sentences_received;
    }

    // Keep track of how long we've been waiting for lock
    seconds += GPS_POLL_SECONDS;
    if (debug(DBG_GPS_MAX)) {
        DEBUG_PRINTF("GPS poll %ds (%d)\n", seconds, comm_gps_get_value(NULL, NULL, NULL));
    }

    // If we're in DFU mode, we don't need to hold things up waiting for GPS
    if (storage()->dfu_status == DFU_PENDING && s_ugps_get_value(NULL, NULL, NULL) != GPS_NO_DATA) {
        skip = true;
        comm_gps_abort();
        sensor_measurement_completed(s);
        gpio_indicators_off();
        DEBUG_PRINTF("GPS present but aborted because of DFU mode\n");
        return;
    }

    // If we're in burn mode and we've received at least some data, short circuit it
    if (sensor_op_mode() == OPMODE_TEST_BURN && s_ugps_get_value(NULL, NULL, NULL) != GPS_NO_DATA) {
        skip = true;
        comm_gps_abort();
        sensor_measurement_completed(s);
        gpio_indicators_off();
        DEBUG_PRINTF("GPS present but aborted because of burn mode\n");
        return;
    }

    // If we're in battery test mode, short circuit all this
    if (battery_status() == BAT_TEST) {
        skip = true;
        comm_gps_abort();
        sensor_measurement_completed(s);
        gpio_indicators_off();
        DEBUG_PRINTF("GPS aborted because of test mode\n");
        return;
    }

    // If we're in mobile mode, don't ever stop unless something needs the UART to transmit.
    if (sensor_op_mode() == OPMODE_MOBILE && comm_would_be_buffered(false)) {
        if (debug(DBG_SENSOR_MAX)) {
            static uint32_t prev_last_sampled_loc = 0;
            if (last_sampled_loc != prev_last_sampled_loc) {
                prev_last_sampled_loc = last_sampled_loc;
                DEBUG_PRINTF("gps %.3f %.3f %s\n", reported_latitude, reported_longitude, gps_active ? "^" : "-");
            } else {
                DEBUG_PRINTF("gps (%s%s%s%s) %s\n", reported_have_location ? "l" : "-", reported_have_full_location ? "L" : "-", reported_have_improved_location ? "I" : "-", reported_have_timedate ? "T" : "-", gps_active ? "^" : "-");
            }
        }
        return;
    }

    // Determine whether or not it's time to stop polling
    if (reported_have_location && reported_have_timedate) {
        uint32_t abort_seconds = GPS_ABORT_FIRST_MINUTES * 60;
        if (reported_have_full_location && !reported_have_improved_location)
            abort_seconds = GPS_ABORT_IMPROVE_MINUTES * 60;
        if (seconds > abort_seconds || (reported_have_full_location && reported_have_improved_location && reported_have_timedate)) {
            reported = true;
            skip = true;
            if (debug(DBG_GPS_MAX))
                DEBUG_PRINTF("%.3f/%.3f/%.3f %lu:%lu\n", reported_latitude, reported_longitude, reported_altitude, reported_date, reported_time);
        }
    }

    // If we've been asked to shut down, terminate
    if (shutdown) {
        skip = true;
        sensor_measurement_completed(s);
        if (reported)
            DEBUG_PRINTF("GPS: %.3f %.3f\n", reported_latitude, reported_longitude);
        return;
    }

    // If we've already got the full location, terminate the polling just to save battery life
    if (!trying_to_improve_location) {
        if ((comm_gps_get_value(NULL, NULL, NULL) == GPS_LOCATION_FULL)) {
            skip = true;
            sensor_measurement_completed(s);
            if (reported)
                DEBUG_PRINTF("GPS: %.3f %.3f\n", reported_latitude, reported_longitude);
            return;
        }
    }

    // If the GPS hardware isn't even present, terminate the polling to save battery life.
    uint32_t abort_seconds = GPS_ABORT_FIRST_MINUTES * 60;
    if (reported_have_full_location && !reported_have_improved_location)
        abort_seconds = GPS_ABORT_IMPROVE_MINUTES * 60;
    if (sensor_op_mode() == OPMODE_TEST_BURN)
        abort_seconds = 30;
    if (seconds > abort_seconds) {
        skip = true;
        comm_gps_abort();
        sensor_measurement_completed(s);
        if (s_ugps_get_value(NULL, NULL, NULL) == GPS_NO_DATA) {
            stats()->errors_ugps++;
            DEBUG_PRINTF("GPS shutdown. (no data)\n");
        } else
            DEBUG_PRINTF("GPS shutdown. (couldn't lock)\n");
        return;
    }

    // Make sure it appears that we are connecting to GPS
    char extras[64];
    if (!reported_have_location && !reported_have_full_location && !reported_have_improved_location && !reported_have_timedate)
        extras[0] = '\0';
    else
        sprintf(extras, " (%s%s%s%s)", reported_have_location ? "l" : "-", reported_have_full_location ? "L" : "-", reported_have_improved_location ? "I" : "-", reported_have_timedate ? "T" : "-");
    DEBUG_PRINTF("GPS waiting for %ds%s\n", seconds, extras);
    gpio_indicate(INDICATE_GPS_CONNECTING);

}

// Get the value
uint16_t s_ugps_get_value(float *lat, float *lon, float *alt) {
    if (!reported)
        return (sentences_received < 5 ? GPS_NO_DATA : GPS_NO_LOCATION);
    if (lat != NULL)
        *lat = reported_latitude;
    if (lon != NULL)
        *lon = reported_longitude;
    if (alt != NULL)
        *alt = reported_altitude;
    return (reported_have_full_location ? GPS_LOCATION_FULL : GPS_LOCATION_PARTIAL);
}

#endif // UGPS
