// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

// Process transmission of a TTProto-formatted message to TTGate or TTServe

#include <stdio.h>
#include <stdlib.h>
#include "debug.h"
#include "config.h"
#include "comm.h"
#include "gpio.h"
#include "geiger.h"
#include "lora.h"
#include "fona.h"
#include "send.h"
#include "bgeigie.h"
#include "phone.h"
#include "misc.h"
#include "timer.h"
#include "gpio.h"
#include "pms.h"
#include "opc.h"
#include "io.h"
#include "serial.h"
#include "sensor.h"
#include "bme0.h"
#include "ina.h"
#include "twi.h"
#include "storage.h"
#include "crc32.h"
#include "nrf_delay.h"
#include "tt.pb.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "app_scheduler.h"

// Buffered I/O header formats.  Note that although we are now starting with version number 0, we
// special case version number 8 because of the old style "single protocl buffer" message format that
// always begins with 0x08. (see ttserve/main.go)
#define BUFF_FORMAT_PB_ARRAY        0
#define BUFF_FORMAT_SINGLE_PB       8

// Send buffer, for cellular use.
// The size allocated here was conservatively computed by assuming
// that generally the worst case is:
// - No more than 100 bytes generally per message, typically 25-75 bytes
// - One message sent every 10 minutes when battery is at its fullest
// - Three failed attempts to send because of acts of god
// The format of the UDP packet sent to the service is
// - One byte of count (N) of protocol buffer messages
// - A byte array of length N with one byte of length of that message, in bytes
// - The concatenated protocol buffer messages
static bool buff_initialized = false;
static uint8_t buff_hdr[250];
static uint8_t buff_data[2500];
static uint8_t *buff_pdata;
static uint16_t buff_data_left;
static uint16_t buff_data_used;
static uint8_t *buff_data_base;
static bool buff_response_type;
static uint8_t buff_pop_hdr;
static uint8_t *buff_pop_pdata;
static uint16_t buff_pop_data_left;
static uint16_t buff_pop_data_used;
static bool buff_pop_response_type;

// MTU-related
static uint16_t mtu_test = 0;
static uint32_t mtu_count = 0;
static uint16_t mtu_max = 0;
static uint32_t mtu_failures = 0;
static char mtu_failure[128] = "";

// Communications statistics
static uint32_t stats_transmitted = 0L;
static uint32_t stats_transmitted_today = 0L;
static uint32_t stats_transmitted_fullday = 0L;
static uint32_t stats_received = 0L;
static uint32_t stats_received_today = 0L;
static uint32_t stats_received_fullday = 0L;
static uint32_t stats_messages = 0L;
static uint32_t stats_messages_today = 0L;
static uint32_t stats_messages_fullday = 0L;
static uint32_t stats_joins = 0L;
static uint32_t stats_joins_today = 0L;
static uint32_t stats_joins_fullday = 0L;
static uint32_t stats_denies = 0L;
static uint32_t stats_denies_today = 0L;
static uint32_t stats_denies_fullday = 0L;
static uint32_t stats_resets = 0L;
static uint32_t stats_power_fails = 0L;
static uint32_t stats_oneshots = 0L;
static uint32_t stats_motiondrops = 0L;
static uint32_t stats_last_minute = 0L;
static uint32_t stats_uptime_minutes = 0L;
static uint32_t stats_uptime_hours = 0L;
static uint32_t stats_uptime_days = 0L;
static uint16_t stats_oneshot_seconds = 0;
static char stats_cell_iccid[40] = "";
static char stats_cell_cpsi[128] = "";
static char stats_battery[64] = "";

// Stamp-related fields
static bool stamp_message_valid = false;
static uint32_t stamp_message_id;
static ttproto_Telecast stamp_message;

// Stamp version number.  The service needs to provide
// backward compatibility forever with these versions because of
// downlevel clients that expect caching, but this client code
// can change unilaterally at any time after the service is 'live'
// with support for it.  The behavior is as follows:
// STAMP_VERSION == 1
//  Required Fields that are always cached: latitude, longitude, captured_at_date, captured_at_time
//  Optional Fields that are cached if present: altitude
#define STAMP_VERSION   1

// Is this capable of being stamped?
bool stampable(ttproto_Telecast *message) {

#ifdef NOSTAMP
    return false;
#endif

    if (!message->has_latitude
        || !message->has_longitude
        || !message->has_captured_at_date
        || !message->has_captured_at_time)
        return false;

    return true;
}

// Get the stamp ID of a message.  Don't call
// this unless it's stampable.
uint32_t stamp_id(ttproto_Telecast *message) {
    char buffer[64];

    sprintf(buffer, "%f,%f,%lu,%lu",
            message->latitude,
            message->longitude,
            message->captured_at_date,
            message->captured_at_time);

    return(crc32_compute((uint8_t *)buffer, strlen(buffer), NULL));

}

// Create a stamp from the stamp fields
bool stamp_create(ttproto_Telecast *message) {
    if (stampable(message)) {

        // Save the stamp info locally
        stamp_message = *message;
        stamp_message_id = stamp_id(message);
        stamp_message_valid = true;

        // Apply the stamp metadata so that the service stores it
        message->stamp = stamp_message_id;
        message->has_stamp = true;
        message->stamp_version = STAMP_VERSION;
        message->has_stamp_version = true;
        return true;

    }
    return false;
}

// Invalidate the saved stamp
void stamp_invalidate() {
    stamp_message_valid = false;
}

// Apply a stamp to the current message if its fields matche the last transmitted stamp,
bool stamp_apply(ttproto_Telecast *message) {

    if (stamp_message_valid && stampable(message)) {
        if (stamp_id(message) == stamp_message_id) {

            // Apply the stamp
            message->stamp = stamp_message_id;
            message->has_stamp = true;

            // Remove the fields that are cached on the service
            message->has_latitude = false;
            message->has_longitude = false;
            message->has_altitude = false;
            message->has_captured_at_date = false;
            message->has_captured_at_time = false;

            return true;

        }
    }

    return false;

}

// Reset the buffer
void send_buff_reset() {

    // No messages
    buff_hdr[0] = BUFF_FORMAT_PB_ARRAY;
    buff_hdr[1] = 0;

    // Always start filling the buffer leaving room for the header
    // which we will ultimately copy into the buffer before doing
    // the UDP I/O.
    buff_data_used = 0;
    buff_data_left = sizeof(buff_data) - sizeof(buff_hdr);
    buff_data_base = buff_data + sizeof(buff_hdr);
    buff_pdata = buff_data_base;
    buff_response_type = REPLY_NONE;

    // Done
    buff_initialized = true;

}

// Prepare the buff for writing
bool send_buff_is_empty() {

    // Initialize if we've never done so
    if (!buff_initialized)
        send_buff_reset();

    // Return whether or not there's anything yet appended into the buffer
    return (buff_hdr[1] == 0);

}


// Prepare the buff for writing
uint8_t *send_buff_prepare_for_transmit(uint16_t *lenptr, uint16_t *response_type_ptr) {

    // Copy the header to be contiguous with the data
    uint8_t count = buff_hdr[1];
    uint8_t header_size = sizeof(buff_hdr[0]) + sizeof(buff_hdr[1]) + count;
    uint8_t *header = buff_data_base - header_size;
    memcpy(header, buff_hdr, header_size);

    // Return the pointer to the buffer and length to be transmitted
    if (lenptr != NULL)
        *lenptr = header_size + buff_data_used;
    if (response_type_ptr != NULL)
        *response_type_ptr = buff_response_type;
    return header;

}

// Append a protocol buffer to the send buffer
bool send_buff_append(uint8_t *ptr, uint8_t len, uint16_t response_type) {

    // Initialize if we've never yet done so
    if (!buff_initialized)
        send_buff_reset();

    // Exit if we've appended too many
    if (buff_hdr[1] >= (sizeof(buff_hdr) - (sizeof(buff_hdr[0]) + sizeof(buff_hdr[1])) ))
        return false;

    // Exit if the body of the buffer is full
    if (buff_data_left < len)
        return false;

    // Remember these in case we need to pop this append
    buff_pop_hdr = buff_hdr[1];
    buff_pop_pdata = buff_pdata;
    buff_pop_data_used = buff_data_used;
    buff_pop_data_left = buff_data_left;
    buff_pop_response_type = buff_response_type;

    // Append to the buffer
    memcpy(buff_pdata, ptr, len);
    buff_pdata += len;
    buff_data_used += len;
    buff_data_left -= len;

    // Append to the header
    buff_hdr[1]++;
    buff_hdr[sizeof(buff_hdr[0])+buff_hdr[1]] = len;

    // Set response type, overriding NONE with what is desired
    if (response_type != REPLY_NONE)
        buff_response_type = response_type;

    // Done
    return true;

}

uint16_t send_length_buffered() {
    if (buff_hdr[1] == 0)
        return 0;
    return(sizeof(buff_hdr[0]) + sizeof(buff_hdr[1]) + buff_hdr[1] + buff_data_used);
}

// Revert the most recent successful append
void send_buff_append_revert() {

    buff_hdr[1] = buff_pop_hdr;
    buff_pdata = buff_pop_pdata;
    buff_data_used = buff_pop_data_used;
    buff_data_left = buff_pop_data_left;
    buff_response_type = buff_pop_response_type;
    DEBUG_PRINTF("Revert: %sb buffered.\n", send_length_buffered());

}

// MTU test in progress?
bool send_mtu_test_in_progress() {
    return (mtu_test != 0);
}

// Set MTU test param
void send_mtu_test(uint16_t start_length) {
    mtu_test = start_length;
}

// Transmit a  message to the service, or suppress it if too often
bool send_update_to_service(uint16_t UpdateType) {
    bool isStatsRequest = (UpdateType != UPDATE_NORMAL);
    bool fBuffered = comm_would_be_buffered();
    bool fLimitedMTU = false;
    bool fBadlyLimitedMTU = false;

    // Exit if we haven't yet completed LPWAN init or if power is turned off comms devices
    if (!comm_can_send_to_service())
        return false;

    // Determine MTU restrictions
    if (comm_get_mtu() < 128)
        fLimitedMTU = true;
    if (comm_get_mtu() < 64)
        fBadlyLimitedMTU = true;

    // Exit if this is a stats request, which must be unbuffered
    if (isStatsRequest)
        if (comm_is_deselected()) {
            return false;
            DEBUG_PRINTF("WAIT: can't send stats while deselected\n");
        }

    // Exit if we're in DFU mode, because we shouldn't be sending anything
    if (storage()->dfu_status == DFU_PENDING)
        return false;

    // Determine whether or not we'll upload particle counts
    bool fUploadParticleCounts = ((storage()->sensors & SENSOR_AIR_COUNTS) != 0);
    UNUSED_VARIABLE(fUploadParticleCounts);

    // If we're in a super low MTU mode, don't upload particle counts
    if (fBadlyLimitedMTU)
        fUploadParticleCounts = false;

    // We keep all these outside of conditional compilation purely for code readability
    bool isGPSDataAvailable = false;
    bool isGeiger0DataAvailable = false;
    bool isGeiger1DataAvailable = false;
    bool isBatteryVoltageDataAvailable = false;
    bool isBatterySOCDataAvailable = false;
    bool isBatteryCurrentDataAvailable = false;
    bool isEnvDataAvailable = false;
    bool isPMSDataAvailable = false;
    bool isOPCDataAvailable = false;

#if defined(TWIMAX17043) || defined(TWIMAX17201) || defined(TWIINA219)
    float batteryVoltage, batterySOC, batteryCurrent;
#endif

#ifdef TWIINA219
    isBatteryVoltageDataAvailable =
        isBatterySOCDataAvailable =
        isBatteryCurrentDataAvailable = s_ina_get_value(&batteryVoltage, &batterySOC, &batteryCurrent);
#endif

#ifdef TWIMAX17201
    isBatteryVoltageDataAvailable =
        isBatterySOCDataAvailable =
        isBatteryCurrentDataAvailable = s_max01_get_value(&batteryVoltage, &batterySOC, &batteryCurrent);
#endif

#ifdef TWIMAX17043
    UNUSED_VARIABLE(batteryCurrent);
    isBatteryVoltageDataAvailable = s_max43_voltage_get_value(&batteryVoltage);
    isBatterySOCDataAvailable = s_max43_soc_get_value(&batterySOC);
#endif

#ifdef TWIHIH6130
    float envTempC, envHumRH;
    isEnvDataAvailable = s_hih6130_get_value(&envTempC, &envHumRH);
#endif

#ifdef TWIBME280
    float envTempC, envHumRH, envPressPA;
    isEnvDataAvailable = s_bme280_0_get_value(&envTempC, &envHumRH, &envPressPA);
#endif

#ifdef PMSX
    uint16_t pms_pm01_0;
    uint16_t pms_pm02_5;
    uint16_t pms_pm10_0;
#if defined(PMS2003) || defined(PMS3003)
    isPMSDataAvailable = s_pms_get_value(&pms_pm01_0, &pms_pm02_5, &pms_pm10_0);
#endif
#if defined(PMS1003) || defined(PMS5003) || defined(PMS7003)
    uint32_t pms_c00_30;
    uint32_t pms_c00_50;
    uint32_t pms_c01_00;
    uint32_t pms_c02_50;
    uint32_t pms_c05_00;
    uint32_t pms_c10_00;
    uint16_t pms_csecs;
    isPMSDataAvailable = s_pms_get_value(&pms_pm01_0, &pms_pm02_5, &pms_pm10_0,
                                         &pms_c00_30, &pms_c00_50, &pms_c01_00, &pms_c02_50, &pms_c05_00, &pms_c10_00,
                                         &pms_csecs);
#endif
#endif // PMSX

#ifdef SPIOPC
    float opc_pm01_0;
    float opc_pm02_5;
    float opc_pm10_0;
    uint32_t opc_c00_38;
    uint32_t opc_c00_54;
    uint32_t opc_c01_00;
    uint32_t opc_c02_10;
    uint32_t opc_c05_00;
    uint32_t opc_c10_00;
    uint16_t opc_csecs;
    isOPCDataAvailable = s_opc_get_value(&opc_pm01_0, &opc_pm02_5, &opc_pm10_0,
                                         &opc_c00_38, &opc_c00_54, &opc_c01_00,
                                         &opc_c02_10, &opc_c05_00, &opc_c10_00,
                                         &opc_csecs);
#endif

    // Get device ID
    uint32_t deviceID = io_get_device_address();

    // Get GPS info, and exit if it's not yet available but COULD be made available
    bool haveAlt = false;
    float lat, lon, alt;
    uint16_t gps_status;
    lat = lon = alt = 0.0;
    gps_status = comm_gps_get_value(&lat, &lon, &alt);
    if (gps_status == GPS_LOCATION_FULL || gps_status == GPS_LOCATION_PARTIAL)
        isGPSDataAvailable = true;
    if (gps_status == GPS_LOCATION_FULL)
        haveAlt = true;

    if (gps_status != GPS_NOT_CONFIGURED && !isGPSDataAvailable) {
        DEBUG_PRINTF("GPS not yet available\n");
        return false;
    }

    // Don't supply altitude except on stats requests, because it's a waste of bandwidth
    if (!isStatsRequest && fLimitedMTU)
        haveAlt = false;

    // Get motion data, and (unless this is a stats request) don't upload anything if moving
    if (!isStatsRequest && sensor_currently_in_motion()) {
        DEBUG_PRINTF("WAIT: device is currently in-motion\n");
        return false;
    }

    // Get Geiger info
#ifdef GEIGERX
    uint32_t cpm0, cpm1;
    // Get the geiger values
    s_geiger_get_value(&isGeiger0DataAvailable, &cpm0, &isGeiger1DataAvailable, &cpm1);
#endif

    // Show the POTENTIAL things we might transmit
    bool wasStatsRequest = isStatsRequest;
    bool wasGeiger0DataAvailable = isGeiger0DataAvailable;
    bool wasGeiger1DataAvailable = isGeiger1DataAvailable;
    bool wasBatteryVoltageDataAvailable = isBatteryVoltageDataAvailable;
    bool wasBatterySOCDataAvailable = isBatterySOCDataAvailable;
    bool wasBatteryCurrentDataAvailable = isBatteryCurrentDataAvailable;
    bool wasEnvDataAvailable = isEnvDataAvailable;
    bool wasPMSDataAvailable = isPMSDataAvailable;
    bool wasOPCDataAvailable = isOPCDataAvailable;

    // If it's a stats request, we must carry it alone regardless of MTU
    if (isStatsRequest) {
        isGeiger0DataAvailable = false;
        isGeiger1DataAvailable = false;
        isBatteryVoltageDataAvailable = false;
        isBatterySOCDataAvailable = false;
        isBatteryCurrentDataAvailable = false;
        isEnvDataAvailable = false;
        isPMSDataAvailable = false;
        isOPCDataAvailable = false;
    }

    // If we're limited at all, don't send both environmental measurements together
    if (fLimitedMTU && fUploadParticleCounts) {
        if (isOPCDataAvailable) {
            isPMSDataAvailable = false;
        }
    }

    // If we're severly limited, strictly send things one class at a time
    while (fBadlyLimitedMTU && !isStatsRequest) {

        if (isGeiger0DataAvailable || isGeiger1DataAvailable) {
            isPMSDataAvailable = false;
            isOPCDataAvailable = false;
            isEnvDataAvailable = false;
            isBatteryVoltageDataAvailable = false;
            isBatterySOCDataAvailable = false;
            isBatteryCurrentDataAvailable = false;
            break;
        }

        if (isPMSDataAvailable) {
            isOPCDataAvailable = false;
            isEnvDataAvailable = false;
            isBatteryVoltageDataAvailable = false;
            isBatterySOCDataAvailable = false;
            isBatteryCurrentDataAvailable = false;
            break;
        }

        if (isOPCDataAvailable) {
            isEnvDataAvailable = false;
            isBatteryVoltageDataAvailable = false;
            isBatterySOCDataAvailable = false;
            isBatteryCurrentDataAvailable = false;
            break;
        }

        if (isEnvDataAvailable) {
            isBatteryVoltageDataAvailable = false;
            isBatterySOCDataAvailable = false;
            isBatteryCurrentDataAvailable = false;
            break;
        }

        break;

    }

    // If we've got the MTU to do so, prefer to piggyback env and bat data onto other messages
    if (!fBadlyLimitedMTU) {
        if (!isStatsRequest &&
            !isGeiger0DataAvailable &&
            !isGeiger1DataAvailable &&
            !isPMSDataAvailable &&
            !isOPCDataAvailable) {
            if (debug(DBG_COMM_MAX))
                DEBUG_PRINTF("WAIT: Substantive data not available\n");
            return false;
        }
    }

    // Exit if there's truly nothing to send
    if (!isStatsRequest &&
        !isGeiger0DataAvailable &&
        !isGeiger1DataAvailable &&
        !isPMSDataAvailable &&
        !isOPCDataAvailable &&
        !isBatteryVoltageDataAvailable &&
        !isBatterySOCDataAvailable &&
        !isBatteryCurrentDataAvailable &&
        !isEnvDataAvailable) {
        if (debug(DBG_COMM_MAX))
            DEBUG_PRINTF("WAIT: Data not available\n");
        return false;
    }

    // Format for transmission
    uint16_t responseType;
    uint16_t status;
    uint8_t buffer[350];
    ttproto_Telecast message = ttproto_Telecast_init_zero;
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

    // Build the message
    message.device_type = ttproto_Telecast_deviceType_SOLARCAST;

    // All messages carry the Device ID
    message.has_device_id = true;
    message.device_id = deviceID;

    // If we've got a local capture date/time from GPS, enclose it
    uint32_t date, time, offset;
    if (get_current_timestamp(&date, &time, &offset)) {
        message.captured_at_date = date;
        message.captured_at_time = time;
        message.captured_at_offset = offset;
        message.has_captured_at_date = message.has_captured_at_time = message.has_captured_at_offset = true;
    }

    // Process stats
    if (isStatsRequest) {

        message.has_reply_type = true;
        message.reply_type = ttproto_Telecast_replyType_REPLY_EXPECTED;

        switch (UpdateType) {

        case UPDATE_STATS_VERSION:
            isGPSDataAvailable = false;
            strncpy(message.stats_app_version, app_version(), sizeof(message.stats_app_version));
            message.has_stats_app_version = true;
            break;

        case UPDATE_STATS_CONFIG_DEV:
            isGPSDataAvailable = false;
            message.has_stats_device_params = storage_get_device_params_as_string(message.stats_device_params, sizeof(message.stats_device_params));
            break;

        case UPDATE_STATS_CONFIG_GPS:
            isGPSDataAvailable = false;
            message.has_stats_gps_params = storage_get_gps_params_as_string(message.stats_gps_params, sizeof(message.stats_gps_params));
            break;

        case UPDATE_STATS_CONFIG_SVC:
            isGPSDataAvailable = false;
            message.has_stats_service_params = storage_get_service_params_as_string(message.stats_service_params, sizeof(message.stats_service_params));
            break;

        case UPDATE_STATS_CONFIG_TTN:
            isGPSDataAvailable = false;
            strncpy(message.stats_ttn_params, storage()->ttn_dev_eui, sizeof(message.stats_ttn_params));
            message.has_stats_ttn_params = true;
            break;

        case UPDATE_STATS_CONFIG_SEN:
            isGPSDataAvailable = false;
            message.has_stats_sensor_params = storage_get_sensor_params_as_string(message.stats_sensor_params, sizeof(message.stats_sensor_params));
            break;

        case UPDATE_STATS_LABEL:
            isGPSDataAvailable = false;
            message.has_stats_device_label = storage_get_device_label_as_string(message.stats_device_label, sizeof(message.stats_device_label));
            break;

        case UPDATE_STATS_BATTERY:
            isGPSDataAvailable = false;
            if (stats_battery[0] != '\0') {
                strncpy(message.stats_battery, stats_battery, sizeof(message.stats_battery));
                message.has_stats_battery = true;
            }
            break;

        case UPDATE_STATS_DFU:
            isGPSDataAvailable = false;
            message.has_stats_dfu = storage_get_dfu_state_as_string(message.stats_dfu, sizeof(message.stats_dfu));
            break;

        case UPDATE_STATS_CELL1:
            isGPSDataAvailable = false;
#ifdef FONA
            if (stats_cell_iccid[0] != '\0') {
                strncpy(message.stats_iccid, stats_cell_iccid, sizeof(message.stats_iccid));
                message.has_stats_iccid = true;
            }
#endif
            break;

        case UPDATE_STATS_CELL2:
            isGPSDataAvailable = false;
#ifdef FONA
            if (stats_cell_cpsi[0] != '\0') {
                strncpy(message.stats_cpsi, stats_cell_cpsi, sizeof(message.stats_cpsi));
                message.has_stats_cpsi = true;
            }
#endif
            break;

        case UPDATE_STATS_MTU_TEST:
            isGPSDataAvailable = false;
            if (mtu_test != 0) {
                if (mtu_test >= sizeof(message.stats_cpsi)-1)
                    mtu_test = 0;
                else {
                    int i;
                    for (i=0; i<mtu_test; i++)
                        message.stats_cpsi[i] = '0' + (i % 10);
                    message.stats_cpsi[i] = '\0';
                    message.has_stats_cpsi = true;
                    mtu_test++;
                }
            }
            break;

        case UPDATE_STATS:
            message.has_stats_uptime_minutes = true;
            message.stats_uptime_minutes = (((stats_uptime_days * 24) + stats_uptime_hours) * 60) + stats_uptime_minutes;
            if (storage()->uptime_days) {
                message.stats_uptime_days = storage()->uptime_days;
                message.has_stats_uptime_days = true;
            }

            if (!fBadlyLimitedMTU) {
                message.has_stats_transmitted_bytes = true;
                message.stats_transmitted_bytes = stats_transmitted;
                message.has_stats_received_bytes = true;
                message.stats_received_bytes = stats_received;
                if (stats_resets) {
                    message.stats_comms_resets = stats_resets;
                    message.has_stats_comms_resets = true;
                }
                if (stats_power_fails) {
                    message.stats_comms_power_fails = stats_power_fails;
                    message.has_stats_comms_power_fails = true;
                }
                if (stats_oneshots) {
                    message.stats_oneshots = stats_oneshots;
                    message.has_stats_oneshots = true;
                }
                if (stats_oneshot_seconds) {
                    message.stats_oneshot_seconds = stats_oneshot_seconds;
                    message.has_stats_oneshot_seconds = true;
                }
                if (stats_motiondrops) {
                    message.stats_motiondrops = stats_motiondrops;
                    message.has_stats_motiondrops = true;
                }
            }
            break;

        }
    }

#ifdef GEIGERX
    if (isGeiger0DataAvailable) {
#if G0==LND7318U
        message.has_lnd_7318u = true;
        message.lnd_7318u = cpm0;
#elif G0==LND7318C
        message.has_lnd_7318c = true;
        message.lnd_7318c = cpm0;
#elif G0==LND7128EC
        message.has_lnd_7128ec = true;
        message.lnd_7128ec = cpm0;
#endif
    }
    if (isGeiger1DataAvailable) {
#if G1==LND7318U
        message.has_lnd_7318u = true;
        message.lnd_7318u = cpm1;
#elif G1==LND7318C
        message.has_lnd_7318c = true;
        message.lnd_7318c = cpm1;
#elif G1==LND7128EC
        message.has_lnd_7128ec = true;
        message.lnd_7128ec = cpm1;
#endif
    }
#endif

    // Strip default values and 0 values from what is transmitted
    if (lat == 0.0 && lon == 0.0)
        isGPSDataAvailable = false;
    if (isGPSDataAvailable) {
        message.latitude = lat;
        message.longitude = lon;
        message.has_latitude = message.has_longitude = true;
        if (haveAlt && !fBadlyLimitedMTU) {
            message.altitude = (int32_t) alt;
            message.has_altitude = true;
        }
    }

#if defined(TWIMAX17043) || defined(TWIMAX17201) || defined(TWIINA219)
    if (isBatteryVoltageDataAvailable) {
        message.has_bat_voltage = true;
        message.bat_voltage = batteryVoltage;
    }
    if (isBatterySOCDataAvailable) {
        message.has_bat_soc = true;
        message.bat_soc = batterySOC;
    }
#endif

#if defined(TWIINA219) || defined(TWIMAX17201)
    if (isBatteryCurrentDataAvailable) {
        message.has_bat_current = true;
        message.bat_current = batteryCurrent;
    }
#endif

#ifdef TWIHIH6130
    if (isEnvDataAvailable) {
        message.has_env_temp = true;
        message.env_temp = envTempC;
        message.has_env_humid = true;
        message.env_humid = envHumRH;
    }
#endif

#ifdef TWIBME280
    if (isEnvDataAvailable) {
        message.has_env_temp = true;
        message.env_temp = envTempC;
        message.has_env_humid = true;
        message.env_humid = envHumRH;
        message.has_env_pressure = true;
        message.env_pressure = envPressPA;
    }
#endif

#ifdef PMSX
    if (isPMSDataAvailable) {
        message.has_pms_pm01_0 = true;
        message.pms_pm01_0 = pms_pm01_0;
        message.has_pms_pm02_5 = true;
        message.pms_pm02_5 = pms_pm02_5;
        message.has_pms_pm10_0 = true;
        message.pms_pm10_0 = pms_pm10_0;
        if (fUploadParticleCounts) {
#if defined(PMS1003) || defined(PMS5003) || defined(PMS7003)
            message.has_pms_c00_30 = true;
            message.pms_c00_30 = pms_c00_30;
            message.has_pms_c00_50 = true;
            message.pms_c00_50 = pms_c00_50;
            message.has_pms_c01_00 = true;
            message.pms_c01_00 = pms_c01_00;
            message.has_pms_c02_50 = true;
            message.pms_c02_50 = pms_c02_50;
            message.has_pms_c05_00 = true;
            message.pms_c05_00 = pms_c05_00;
            message.has_pms_c10_00 = true;
            message.pms_c10_00 = pms_c10_00;
            message.has_pms_csecs = true;
            message.pms_csecs = pms_csecs;
#endif
        }
    }
#endif // PMSX

#ifdef SPIOPC
    if (isOPCDataAvailable) {
        message.has_opc_pm01_0 = true;
        message.opc_pm01_0 = opc_pm01_0;
        message.has_opc_pm02_5 = true;
        message.opc_pm02_5 = opc_pm02_5;
        message.has_opc_pm10_0 = true;
        message.opc_pm10_0 = opc_pm10_0;
        if (fUploadParticleCounts) {
            message.has_opc_c00_38 = true;
            message.opc_c00_38 = opc_c00_38;
            message.has_opc_c00_54 = true;
            message.opc_c00_54 = opc_c00_54;
            message.has_opc_c01_00 = true;
            message.opc_c01_00 = opc_c01_00;
            message.has_opc_c02_10 = true;
            message.opc_c02_10 = opc_c02_10;
            message.has_opc_c05_00 = true;
            message.opc_c05_00 = opc_c05_00;
            message.has_opc_c10_00 = true;
            message.opc_c10_00 = opc_c10_00;
            message.has_opc_csecs = true;
            message.opc_csecs = opc_csecs;
        }
    }
#endif // SPIOPC

    // If it's a stats request, add a special "stamp" that tells the service
    // to buffer the values of certain rarely-changing fields.  Otherwise,
    // examine the message to see if we can "apply" the previously-transmitted
    // stamp, thus removing fields that can be supplied by TTSERVE.  This
    // is a major bandwidth optimization, even if there is some risk that
    // if this stats message is lost we may end up discarding measurements
    // because of lack of critical fields.
    bool stamp_created = false;
    if (isStatsRequest)
        stamp_created = stamp_create(&message);
    else
        stamp_apply(&message);

    // If a stats request, make it a request/response to the service
    if (isStatsRequest)
        responseType = REPLY_TTSERVE;
    else
        responseType = REPLY_NONE;

    // Encode the message
    status = pb_encode(&stream, ttproto_Telecast_fields, &message);
    if (!status) {
        DEBUG_PRINTF("Send pb_encode: %s\n", PB_GET_ERROR(&stream));
        if (stamp_created)
            stamp_invalidate();
        return false;
    }

    // Transmit it or buffer it, and set fSent
    uint16_t bytes_written = stream.bytes_written;
    bool fSent = true;
    bool fMTUFailure = false;

    if (fBuffered) {

        // Buffer it
        fSent = send_buff_append(buffer, bytes_written, responseType);

    } else {

        if (send_buff_is_empty()) {

            // If this is larger than allowable MTU, don't bother
            if (bytes_written > comm_get_mtu()) {

                fMTUFailure = true;

            } else {

                // If the buffer is empty, just send a single PB to the service
                fSent = send_to_service(buffer, bytes_written, responseType, SEND_1);

            }

        } else {

            // Append this to the existing buffer, and transmit N
            fSent = send_buff_append(buffer, bytes_written, responseType);
            if (fSent) {

                // Prepare for transmission by fetching length and the requested response type
                uint16_t send_response_type;
                uint8_t *xmit_buff = send_buff_prepare_for_transmit(&bytes_written, &send_response_type);

                // Determine whether or not we're being instructed to choose "efficient" or "reliable"
                // transport of buffered messages.  Neither is ideal; they both have tradeoffs.
                // Buffered messages contain a LOT of information, and so if they are lost there is
                // a lot to lose.  If we send "efficiently" via UDP, there is a high risk that it
                // will be lost.  If we sent "reliably" via TCP or HTTP, the bandwidth costs more.
                // By default, we'll send them reliably.  We do so by leveraging the side-effect
                // that we know a reliable transport is used when requesting a reply from the service.
                if (send_response_type == REPLY_NONE) {
                    if (!(storage()->flags & FLAG_BUFFERED_EFFICIENT))
                        send_response_type = REPLY_TTSERVE;
                }

                // If this is longer than the allowed mtu, don't even bother.
                if (bytes_written > comm_get_mtu()) {

                    fMTUFailure = true;
                    send_buff_reset();

                } else {

                    // Transmit the entire batch of buffered samples
                    fSent = send_to_service(xmit_buff, bytes_written, send_response_type, SEND_N);
                    if (fSent)
                        send_buff_reset();
                    else
                        send_buff_append_revert();

                }
            }

        }

    }

    // Display data about message
    char buff_msg[10];
    char sent_msg[200];
    if (send_length_buffered() == 0)
        buff_msg[0] = '\0';
    else
        sprintf(buff_msg, "/%db", send_length_buffered());

    sprintf(sent_msg, "%db%s S%s G%s%s V%s%s%s E%s Pm%s Op%s",
            bytes_written, buff_msg,
            wasStatsRequest ? (isStatsRequest ? "+" : "X") : "-",
            wasGeiger0DataAvailable ? (isGeiger0DataAvailable ? "+" : "X") : "-",
            wasGeiger1DataAvailable ? (isGeiger1DataAvailable ? "+" : "X") : "-",
            wasBatteryVoltageDataAvailable ? (isBatteryVoltageDataAvailable ? "+" : "X") : "-",
            wasBatterySOCDataAvailable ? (isBatterySOCDataAvailable ? "+" : "X") : "-",
            wasBatteryCurrentDataAvailable ? (isBatteryCurrentDataAvailable ? "+" : "X") : "-",
            wasEnvDataAvailable ? (isEnvDataAvailable ? "+" : "X") : "-",
            wasPMSDataAvailable ? (isPMSDataAvailable ? "+" : "X") : "-",
            wasOPCDataAvailable ? (isOPCDataAvailable ? "+" : "X") : "-");

    if (fMTUFailure) {
        mtu_failures++;
        strncpy(mtu_failure, sent_msg, sizeof(mtu_failure)-1);
        DEBUG_PRINTF("** Message length %d > %d max MTU\n", bytes_written, comm_get_mtu());
    } else {
        if (fSent) {
            mtu_count++;
            if (bytes_written > mtu_max)
                mtu_max = bytes_written;
        }
    }

    DEBUG_PRINTF("%s %s\n", fMTUFailure ? "FAIL" : (fSent ? (fBuffered ? "BUFF" : "SENT") : "WAIT"), sent_msg);

    // Exit if we shouldn't retry
    if (!fSent) {
        if (stamp_created)
            stamp_invalidate();
        return false;
    }

    // Clear them once transmitted successfully
#ifdef GEIGERX
    if (isGeiger0DataAvailable || isGeiger1DataAvailable)
        s_geiger_clear_measurement();
#endif
#ifdef PMSX
    if (isPMSDataAvailable)
        s_pms_clear_measurement();
#endif
#ifdef SPIOPC
    if (isOPCDataAvailable)
        s_opc_clear_measurement();
#endif
#ifdef TWIHIH6130
    if (isEnvDataAvailable)
        s_hih6130_clear_measurement();
#endif
#ifdef TWIBME280
    if (isEnvDataAvailable)
        s_bme280_0_clear_measurement();
#endif
#ifdef TWIMAX17043
    if (isBatteryVoltageDataAvailable)
        s_max43_voltage_clear_measurement();
    if (isBatterySOCDataAvailable)
        s_max43_soc_clear_measurement();
#endif
#ifdef TWIINA219
    if (isBatteryCurrentDataAvailable)
        s_ina_clear_measurement();
#endif
#ifdef TWIMAX17201
    if (isBatteryCurrentDataAvailable)
        s_max01_clear_measurement();
#endif
#if defined (GEIGERX)
    if (debug(DBG_SENSOR)) {
        if (isGeiger0DataAvailable && isGeiger1DataAvailable)
            DEBUG_PRINTF("Counters reported 0:%lucpm 1:%lucpm\n", cpm0, cpm1);
        else if (isGeiger0DataAvailable)
            DEBUG_PRINTF("Counter reported 0:%lucpm\n", cpm0);
        else if (isGeiger1DataAvailable)
            DEBUG_PRINTF("Counter reported 1:%lucpm\n", cpm1);
    }
#endif

    return true;

}

// Display failures that should come to our attention
void mtu_status_check(bool fVerbose) {
    if (fVerbose)
        DEBUG_PRINTF("MTU max%d of limit %d in %ld messages\n", mtu_max, comm_get_mtu(), mtu_count);
    if (mtu_failures) {
        DEBUG_PRINTF("** %ld MTU fails: %s\n", mtu_failures, mtu_failure);
    }
}

// Transmit a binary message to the service on the currently-active transport, even
// if the transport is busy or if we're in init.
bool send_to_service_unconditionally(uint8_t *buffer, uint16_t length, uint16_t RequestType, uint16_t RequestFormat) {
    static int32_t transmitInProgress = 0;
    bool fTransmitted = false;

    // Ensure that we don't go recursive.  This can happen if we try to transmit
    // something from a timer interrupt or bluetooth interrupt, when in fact
    // we are already trying to transmit something.  Rather than queue it,
    // we just drop it on the floor.  This is acceptable for the type of things
    // that we transmit.
    if (transmitInProgress++ != 0) {
        transmitInProgress--;
        return false;
    }

    // Transmit it
    switch (comm_mode()) {
#ifdef LORA
    case COMM_LORA:
        fTransmitted = lora_send_to_service(buffer, length, RequestType, RequestFormat);
        break;
#endif
#ifdef FONA
    case COMM_FONA:
        fTransmitted = fona_send_to_service(buffer, length, RequestType, RequestFormat);
        break;
#endif
    default:
        fTransmitted = false;
        break;
    }

    // Done
    transmitInProgress--;
    return fTransmitted;

}

// Transmit a binary message to the service on the currently-active transport,
// failing the send if we aren't through with init.
bool send_to_service(uint8_t *buffer, uint16_t length, uint16_t RequestType, uint16_t RequestFormat) {

    // Exit if we aren't in a state where we can transmit
    if (!comm_can_send_to_service())
        return false;

    // Send it.
    return(send_to_service_unconditionally(buffer, length, RequestType, RequestFormat));

}

// Send a ping message to the LPWAN.  This may be done to
// see if we get a reply from TTGATE, or TTSERVE (via a TTNGATE),
// or potentially nothing if we're out of range.
bool send_ping_to_service(uint16_t pingtype) {
    uint16_t status;
    uint8_t buffer[64];

    // Allocate space on the stack to store the message data.
    ttproto_Telecast message = ttproto_Telecast_init_zero;

    /* Create a stream that will write to our buffer. */
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

    // Build the message in a very minimal way, assigning the device type
    // as an indicator of the kind of a ping.
    if (pingtype == REPLY_NONE)
        message.device_type = ttproto_Telecast_deviceType_TTAPP;
    else if (pingtype == REPLY_TTSERVE)
        message.device_type = ttproto_Telecast_deviceType_TTSERVE;
    else if (pingtype == REPLY_TTGATE)
        message.device_type = ttproto_Telecast_deviceType_TTGATE;

    // Use our device ID, so we know when it comes back that it was from us
    message.device_id = io_get_device_address();
    message.has_device_id = true;

    // encode it and transmit it
    status = pb_encode(&stream, ttproto_Telecast_fields, &message);
    if (!status) {
        DEBUG_PRINTF("pb_encode: %s\n", PB_GET_ERROR(&stream));
        return false;
    }

    // Transmit it, even if we're in the middle of init
    if (!send_to_service_unconditionally(buffer, stream.bytes_written, pingtype, SEND_1))
        return false;

    // Successfully transmitted
    return (true);

}

// Update uptime stats
void stats_update() {
    if (!ShouldSuppress(&stats_last_minute, 60)) {
        stats_uptime_minutes++;
        if (stats_uptime_minutes >= 60) {
            stats_uptime_minutes = 0;
            stats_uptime_hours++;
            if (stats_uptime_hours >= 24) {
                stats_uptime_hours = 0;
                stats_uptime_days++;
                // Reset and update daily stats
                if (stats_uptime_days > 1) {
                    stats_transmitted_fullday = stats_transmitted_today;
                    stats_received_fullday = stats_received_today;
                    stats_messages_fullday = stats_messages_today;
                    stats_joins_fullday = stats_joins_today;
                    stats_denies_fullday = stats_denies_today;
                }
                stats_transmitted_today = 0;
                stats_received_today = 0;
                stats_messages_today = 0;
                stats_joins_today = 0;
                stats_denies_today = 0;
                // Restart the device when appropriate
                if (storage()->restart_days != 0 && stats_uptime_days >= storage()->restart_days) {
                    storage()->uptime_days += stats_uptime_days;
                    storage_save();
                    io_request_restart();
                }
            }
        }
    }
}

// Set the cell info
void stats_set_cell_info(char *iccid, char *cpsi) {
    if (iccid != NULL)
        strncpy(stats_cell_iccid, iccid, sizeof(stats_cell_iccid)-1);
    if (cpsi != NULL)
        strncpy(stats_cell_cpsi, cpsi, sizeof(stats_cell_cpsi)-1);
}

// Set the cell info
bool stats_set_battery_info(char *info) {
    if (info != NULL)
        strncpy(stats_battery, info, sizeof(stats_battery)-1);
    return (stats_battery[0] != '\0');
}

// Set statistics
void stats_set(uint16_t oneshot_seconds) {
    stats_oneshot_seconds = oneshot_seconds;
}

// Bump critical statistics
void stats_add(uint16_t transmitted, uint16_t received, uint16_t resets, uint16_t powerfails, uint16_t oneshots, uint16_t motiondrops, uint16_t messages, uint16_t joins, uint16_t denies) {
    stats_transmitted += transmitted;
    stats_transmitted_today += transmitted;
    stats_received += received;
    stats_received_today += received;
    stats_resets += resets;
    stats_power_fails += powerfails;
    stats_oneshots += oneshots;
    stats_motiondrops += motiondrops;
    stats_messages += messages;
    stats_messages_today += messages;
    stats_joins += joins;
    stats_joins_today += joins;
    stats_denies += denies;
    stats_denies_today += denies;
}

// Quick status check
void stats_status_check(bool fVerbose) {
    if (fVerbose) {
        DEBUG_PRINTF("TODAY: xmt:%d rcv:%d cnt:%d j:%d d:%d\n",
                     stats_transmitted_today, stats_received_today, stats_messages_today,
                     stats_joins_today, stats_denies_today);
        if (stats_received_fullday)
            DEBUG_PRINTF("FULLDAY: xmt:%d rcv:%d cnt:%d j:%d d:%d\n",
                         stats_transmitted_fullday, stats_received_fullday, stats_messages_fullday,
                         stats_joins_fullday, stats_denies_fullday);
    }
}
