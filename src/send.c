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
#include "bme1.h"
#include "ina.h"
#include "twi.h"
#include "storage.h"
#include "crc32.h"
#include "nrf_delay.h"
#include "tt.pb.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "app_scheduler.h"
#include "stats.h"
#include "battery.h"

#ifndef FONA
#define TINYBUFFERS
#endif

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
#ifdef TINYBUFFERS
static uint8_t buff_hdr[25];
static uint8_t buff_data[250];
#else
static uint8_t buff_hdr[250];
static uint8_t buff_data[2500];
#endif
static bool buff_initialized = false;
static uint8_t *buff_pdata;
static uint16_t buff_hdr_used;
static uint16_t buff_data_left;
static uint16_t buff_data_used;
static uint8_t *buff_data_base;
static uint16_t buff_response_type;
static uint8_t buff_pop_hdr;
static uint8_t *buff_pop_pdata;
static uint16_t buff_pop_data_left;
static uint16_t buff_pop_data_used;
static uint16_t buff_pop_hdr_used;
static uint16_t buff_pop_response_type;

// MTU-related
static uint16_t mtu_test = 0;
static uint32_t mtu_count = 0;
static uint16_t mtu_max = 0;
static char mtu_failure[128] = "";

// Stamp-related fields
static bool stamp_message_valid = false;
static uint32_t stamp_message_id;
static ttproto_Telecast stamp_message;
static uint32_t mobile_session_time_offset;     // uses the same base date/time as captured_date

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

    if (!message->has_captured_at_date || !message->has_captured_at_time)
        return false;

    return true;
}

// Get the stamp ID of a message.  Don't call this unless it's stampable.
uint32_t stamp_id(ttproto_Telecast *message) {
    char buffer[64];

    // Only include lat/lon when present and NOT in mobile mode
    if (message->has_latitude && message->has_longitude && sensor_op_mode() != OPMODE_MOBILE)
        sprintf(buffer, "%f,%f,%lu,%lu",
                message->latitude,
                message->longitude,
                message->captured_at_date,
                message->captured_at_time);
    else
        sprintf(buffer, "%lu,%lu",
                message->captured_at_date,
                message->captured_at_time);

    return(crc32_compute((uint8_t *)buffer, strlen(buffer), NULL));

}

// Create a stamp from the stamp fields
bool stamp_create(ttproto_Telecast *message) {
    static uint32_t mobile_session_id = 12345;          // init to something unlikely

    if (stampable(message)) {

        // If we're in mobile mode, do special processing
        if (sensor_op_mode() == OPMODE_MOBILE) {

            // First, REMOVE the location info so that
            // we can stamp messages that are in motion.
            message->has_latitude = false;
            message->has_longitude = false;
            message->has_altitude = false;

            // Next, update the motion time offset ("Drive ID") if we've started a new session
            if (mobile_session_id != sensor_get_mobile_session_id()) {
                mobile_session_id = sensor_get_mobile_session_id();
                mobile_session_time_offset = message->motion_began_offset = message->captured_at_offset;
            }

        }

        // Create the stamp repeatedly even if it has changed, just to
        // cover the (very rare) case where the server admin purges stamps
        uint32_t message_id = stamp_id(message);

        // Save the stamp info locally
        stamp_message_id = message_id;
        stamp_message = *message;
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
            message->has_captured_at_date = false;
            message->has_captured_at_time = false;
            if (stamp_message.has_latitude || stamp_message.has_longitude) {
                message->has_latitude = false;
                message->has_longitude = false;
                message->has_altitude = false;
            }
            if (stamp_message.has_motion_began_offset)
                message->has_motion_began_offset = false;
            if (stamp_message.has_test)
                message->has_test = false;

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
    buff_hdr_used = sizeof(buff_hdr[0]) + sizeof(buff_hdr[1]);

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

// Determine if we should avoid filling any more, out of caution
bool send_buff_is_full(uint16_t anticipated) {

    // Initialize if we've never done so
    if (!buff_initialized)
        send_buff_reset();

    // The "buffer left" is just a bit smaller than the
    // full buffer, just to be conservative.
    uint16_t max_buffer_size = sizeof(buff_data) - 250;

    // If we've already overflowed, indicate so
    uint16_t hdr_anticipated = buff_hdr_used + (anticipated ? 1 : 0);
    if ((hdr_anticipated + buff_data_used + anticipated) > max_buffer_size)
        return true;

    // There's room
    return false;

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

    // Exit if we've exceeded MTU
    if (send_buff_is_full(len))
        return false;

    // Exit if we've appended too many
    if (buff_hdr[1] >= (sizeof(buff_hdr) - (sizeof(buff_hdr[0]) + sizeof(buff_hdr[1])) ))
        return false;

    // Exit if the body of the buffer is full
    if (buff_data_left < len)
        return false;

    // Remember these in case we need to pop this append
    buff_pop_hdr = buff_hdr[1];
    buff_pop_pdata = buff_pdata;
    buff_pop_hdr_used = buff_hdr_used;
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
    buff_hdr_used++;

    // Set response type, overriding NONE with what is desired
    if (response_type != REPLY_NONE)
        buff_response_type = response_type;

    // If we've buffered at least 3 items, force a reply response type simply because
    // this means that we've been offline for quite a while and it would be good to give
    // the service a chance to send us a command.
    if (buff_hdr[1] > 3)
        buff_response_type = REPLY_TTSERVE;

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
    buff_hdr_used = buff_pop_hdr_used;
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
    char *StatType = "";
    bool isTestMeasurement = false;
    bool isStatsRequest = (UpdateType != UPDATE_NORMAL);
    bool fBuffered = comm_would_be_buffered(false);
    bool fLimitedMTU = false;
    bool fBadlyLimitedMTU = false;
    stats_t *stp = stats();

    // Hard-wired for our test devices
    if ((storage()->flags & FLAG_TEST) != 0)
        isTestMeasurement = true;

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

    if (sensor_op_mode() == OPMODE_TEST_BURN)
        fUploadParticleCounts = true;

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
    bool isEncDataAvailable = false;
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

#ifdef TWIBME0
    float envTempC, envHumRH, envPressPA;
    isEnvDataAvailable = s_bme280_0_get_value(&envTempC, &envHumRH, &envPressPA);
#endif

#ifdef TWIBME1
    float encTempC, encHumRH, encPressPA;
    isEncDataAvailable = s_bme280_1_get_value(&encTempC, &encHumRH, &encPressPA);
#endif

#ifdef PMSX
    uint16_t pms_pm01_0;
    uint16_t pms_pm02_5;
    uint16_t pms_pm10_0;
    float pms_std01_0;
    float pms_std02_5;
    float pms_std10_0;
#if defined(PMS2003) || defined(PMS3003)
    isPMSDataAvailable = s_pms_get_value(&pms_pm01_0, &pms_pm02_5, &pms_pm10_0, &pms_std01_0, &pms_std02_5, &pms_std10_0);
#endif
#if defined(PMS1003) || defined(PMS5003) || defined(PMS7003)
    uint32_t pms_c00_30;
    uint32_t pms_c00_50;
    uint32_t pms_c01_00;
    uint32_t pms_c02_50;
    uint32_t pms_c05_00;
    uint32_t pms_c10_00;
    uint16_t pms_csecs;
    isPMSDataAvailable = s_pms_get_value(&pms_pm01_0, &pms_pm02_5, &pms_pm10_0, &pms_std01_0, &pms_std02_5, &pms_std10_0,
                                         &pms_c00_30, &pms_c00_50, &pms_c01_00, &pms_c02_50, &pms_c05_00, &pms_c10_00,
                                         &pms_csecs);
#endif
#endif // PMSX

#ifdef SPIOPC
    float opc_pm01_0;
    float opc_pm02_5;
    float opc_pm10_0;
    float opc_std01_0;
    float opc_std02_5;
    float opc_std10_0;
    uint32_t opc_c00_38;
    uint32_t opc_c00_54;
    uint32_t opc_c01_00;
    uint32_t opc_c02_10;
    uint32_t opc_c05_00;
    uint32_t opc_c10_00;
    uint16_t opc_csecs;
    isOPCDataAvailable = s_opc_get_value(&opc_pm01_0, &opc_pm02_5, &opc_pm10_0,
                                         &opc_std01_0, &opc_std02_5, &opc_std10_0,
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

    // Don't supply altitude if limited MTU in cases where it's a waste of bandwidth
    if (fLimitedMTU || sensor_op_mode() == OPMODE_MOBILE)
        haveAlt = false;

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
    bool wasEncDataAvailable = isEncDataAvailable;
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
        isEncDataAvailable = false;
        isPMSDataAvailable = false;
        isOPCDataAvailable = false;
    }

    // If we're limited at all, don't send both environmental measurements together
    if (fLimitedMTU && fUploadParticleCounts) {

        // Only send one sensor or the other
        if (isPMSDataAvailable) {
            isOPCDataAvailable = false;
        }

        // Don't package other things with it.  We've encountered MTU issues
        // when motion happens (and thus we don't have the benefit of stamp
        // optimization).
        if (isOPCDataAvailable || isPMSDataAvailable) {
            isGeiger0DataAvailable = false;
            isGeiger1DataAvailable = false;
            isBatteryVoltageDataAvailable = false;
            isBatterySOCDataAvailable = false;
            isBatteryCurrentDataAvailable = false;
            isEncDataAvailable = false;
            isEnvDataAvailable = false;
        }

    }

    // If we're severly limited, strictly send things one class at a time
    while (fBadlyLimitedMTU && !isStatsRequest) {

        if (isGeiger0DataAvailable || isGeiger1DataAvailable) {
            isPMSDataAvailable = false;
            isOPCDataAvailable = false;
            isEnvDataAvailable = false;
            isEncDataAvailable = false;
            isBatteryVoltageDataAvailable = false;
            isBatterySOCDataAvailable = false;
            isBatteryCurrentDataAvailable = false;
            break;
        }

        if (isPMSDataAvailable) {
            isOPCDataAvailable = false;
            isEnvDataAvailable = false;
            isEncDataAvailable = false;
            isBatteryVoltageDataAvailable = false;
            isBatterySOCDataAvailable = false;
            isBatteryCurrentDataAvailable = false;
            break;
        }

        if (isOPCDataAvailable) {
            isEnvDataAvailable = false;
            isEncDataAvailable = false;
            isBatteryVoltageDataAvailable = false;
            isBatterySOCDataAvailable = false;
            isBatteryCurrentDataAvailable = false;
            break;
        }

        if (isEnvDataAvailable) {
            isEncDataAvailable = false;
            isBatteryVoltageDataAvailable = false;
            isBatterySOCDataAvailable = false;
            isBatteryCurrentDataAvailable = false;
            break;
        }

        if (isEncDataAvailable) {
            isBatteryVoltageDataAvailable = false;
            isBatterySOCDataAvailable = false;
            isBatteryCurrentDataAvailable = false;
            break;
        }

        break;

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
        !isEnvDataAvailable &&
        !isEncDataAvailable) {
        if (debug(DBG_COMM_MAX))
            DEBUG_PRINTF("SEND: (nothing to send)\n");
        comm_oneshot_completed();
        return false;
    }

    // Format for transmission
    uint16_t responseType;
    uint16_t status;
    uint8_t buffer[350];
    ttproto_Telecast message = ttproto_Telecast_init_zero;
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

    // As of 2017-03-24, now that we send everything through buffered message
    // format, this field is optional because TTSERVE defaults to SOLARCAST if not present.
#if 0
    message.has_device_type = true;
    message.device_type = ttproto_Telecast_deviceType_SOLARCAST;
#endif

    // Build the message
    message.has_device_id = true;
    message.device_id = deviceID;

    // If we've got a local capture date/time from GPS, enclose it
    uint32_t date, time, offset;
    if (get_current_timestamp(&date, &time, &offset)) {
        message.captured_at_date = date;
        message.captured_at_time = time;
        message.captured_at_offset = offset;
        message.has_captured_at_date = message.has_captured_at_time = message.has_captured_at_offset = true;
        // Also, add the motion offset (which shares the same captured_at date/time) if we're mobile.
        // This is interpreted by the service as the "drive ID", which groups related measurements together.
        if (sensor_op_mode() == OPMODE_MOBILE) {
            message.motion_began_offset = mobile_session_time_offset;
            message.has_motion_began_offset = true;
        }

    }

    // Process stats
    if (isStatsRequest) {

        switch (UpdateType) {

        case UPDATE_STATS_VERSION:
            strlcpy(message.stats_app_version, app_version(), sizeof(message.stats_app_version));
            message.has_stats_app_version = true;
            StatType = "version";
            break;

        case UPDATE_STATS_CONFIG_DEV:
            message.has_stats_device_params = storage_get_device_params_as_string(message.stats_device_params, sizeof(message.stats_device_params));
            StatType = "device";
            break;

        case UPDATE_STATS_CONFIG_GPS:
            message.has_stats_gps_params = storage_get_gps_params_as_string(message.stats_gps_params, sizeof(message.stats_gps_params));
            StatType = "gps";
            break;

        case UPDATE_STATS_CONFIG_SVC:
            message.has_stats_service_params = storage_get_service_params_as_string(message.stats_service_params, sizeof(message.stats_service_params));
            StatType = "service";
            break;

        case UPDATE_STATS_CONFIG_TTN:
            strlcpy(message.stats_ttn_params, storage()->ttn_dev_eui, sizeof(message.stats_ttn_params));
            message.has_stats_ttn_params = true;
            StatType = "ttn";
            break;

        case UPDATE_STATS_CONFIG_SEN:
            message.has_stats_sensor_params = storage_get_sensor_params_as_string(message.stats_sensor_params, sizeof(message.stats_sensor_params));
            StatType = "sensor";
            break;

        case UPDATE_STATS_LABEL:
            message.has_stats_device_label = storage_get_device_label_as_string(message.stats_device_label, sizeof(message.stats_device_label));
            StatType = "label";
            break;

        case UPDATE_STATS_BATTERY:
            if (!fLimitedMTU && stp->battery[0] != '\0') {
                strlcpy(message.stats_battery, stp->battery, sizeof(message.stats_battery));
                message.has_stats_battery = true;
            }
            StatType = "battery";
            break;

        case UPDATE_STATS_DFU:
            message.has_stats_dfu = storage_get_dfu_state_as_string(message.stats_dfu, sizeof(message.stats_dfu));
            StatType = "dfu";
            break;

        case UPDATE_STATS_MODULES:
            if (stp->module_lora[0] != '\0') {
                strlcpy(message.stats_module_lora, stp->module_lora, sizeof(message.stats_module_lora));
                message.has_stats_module_lora = true;
            }
            if (stp->module_fona[0] != '\0') {
                strlcpy(message.stats_module_fona, stp->module_fona, sizeof(message.stats_module_fona));
                message.has_stats_module_fona = true;
            }
            StatType = "module";
            break;

        case UPDATE_STATS_CELL1:
#ifdef FONA
            if (stp->cell_iccid[0] != '\0') {
                strlcpy(message.stats_iccid, stp->cell_iccid, sizeof(message.stats_iccid));
                message.has_stats_iccid = true;
            }
#endif
            StatType = "iccid";
            break;

        case UPDATE_STATS_CELL2:
#ifdef FONA
            if (stp->cell_cpsi[0] != '\0') {
                strlcpy(message.stats_cpsi, stp->cell_cpsi, sizeof(message.stats_cpsi));
                message.has_stats_cpsi = true;
            }
#endif
            StatType = "cell";
            break;

        case UPDATE_STATS_MTU_TEST:
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

        case UPDATE_STATS_ERRORS:
            if (stp->errors_opc != 0) {
                message.errors_opc = stp->errors_opc;
                message.has_errors_opc = true;
            }
            if (stp->errors_pms != 0) {
                message.errors_pms = stp->errors_pms;
                message.has_errors_pms = true;
            }
            if (stp->errors_bme0 != 0) {
                message.errors_bme0 = stp->errors_bme0;
                message.has_errors_bme0 = true;
            }
            if (stp->errors_bme1 != 0) {
                message.errors_bme1 = stp->errors_bme1;
                message.has_errors_bme1 = true;
            }
            if (stp->errors_lora != 0) {
                message.errors_lora = stp->errors_lora;
                message.has_errors_lora = true;
            }
            if (stp->errors_fona != 0) {
                message.errors_fona = stp->errors_fona;
                message.has_errors_fona = true;
            }
            if (stp->errors_geiger != 0) {
                message.errors_geiger = stp->errors_geiger;
                message.has_errors_geiger = true;
            }
            if (stp->errors_max01 != 0) {
                message.errors_max01 = stp->errors_max01;
                message.has_errors_max01 = true;
            }
            if (stp->errors_ugps != 0) {
                message.errors_ugps = stp->errors_ugps;
                message.has_errors_ugps = true;
            }
            if (stp->errors_lis != 0) {
                message.errors_lis = stp->errors_lis;
                message.has_errors_lis = true;
            }
            if (stp->errors_twi != 0) {
                message.errors_twi = stp->errors_twi;
                message.has_errors_twi = true;
            }
            // fails on Lora during burn mode with bad twi, where errors accumulate
            if (!fLimitedMTU || strlen(stp->errors_twi_info) < 48) {
                strlcpy(message.errors_twi_info, stp->errors_twi_info, sizeof(message.errors_twi_info));
                message.has_errors_twi_info = true;
            }
            if (stp->errors_spi != 0) {
                message.errors_spi = stp->errors_spi;
                message.has_errors_spi = true;
            }
            if (stp->mtu_failures != 0) {
                message.errors_mtu = stp->mtu_failures;
                message.has_errors_mtu = true;
            }
            if (stp->errors_connect_lora != 0) {
                message.errors_connect_lora = stp->errors_connect_lora;
                message.has_errors_connect_lora = true;
            }
            if (stp->errors_connect_fona != 0) {
                message.errors_connect_fona = stp->errors_connect_fona;
                message.has_errors_connect_fona = true;
            }
            if (stp->errors_connect_gateway != 0) {
                message.errors_connect_gateway = stp->errors_connect_gateway;
                message.has_errors_connect_gateway = true;
            }
            if (stp->errors_connect_wireless != 0) {
                message.errors_connect_wireless = stp->errors_connect_wireless;
                message.has_errors_connect_wireless = true;
            }
            if (stp->errors_connect_data != 0) {
                message.errors_connect_data = stp->errors_connect_data;
                message.has_errors_connect_data = true;
            }
            if (stp->errors_connect_service != 0) {
                message.errors_connect_service = stp->errors_connect_service;
                message.has_errors_connect_service = true;
            }
            StatType = "errors";
            break;

            // For LoraWAN we need to transmit next to NOTHING to fit into the MTU, because each _STATS message
            // carries the stamp information and it is super critical that it makes it to the service.
        case UPDATE_STATS:
            if (!fBadlyLimitedMTU) {
                message.has_stats_uptime_minutes = true;
                message.stats_uptime_minutes = (((stp->uptime_days * 24) + stp->uptime_hours) * 60) + stp->uptime_minutes;
                if (storage()->uptime_days) {
                    message.stats_uptime_days = storage()->uptime_days;
                    message.has_stats_uptime_days = true;
                }
                message.has_stats_transmitted_bytes = true;
                message.stats_transmitted_bytes = stp->transmitted;
                message.has_stats_received_bytes = true;
                message.stats_received_bytes = stp->received;
                if (stp->resets) {
                    message.stats_comms_resets = stp->resets;
                    message.has_stats_comms_resets = true;
                }
                if (stp->power_fails) {
                    message.stats_comms_power_fails = stp->power_fails;
                    message.has_stats_comms_power_fails = true;
                }
                if (stp->overcurrent_events) {
                    message.stats_overcurrent_events = stp->overcurrent_events;
                    message.has_stats_overcurrent_events = true;
                }
                if (stp->ant_fails) {
                    message.stats_comms_ant_fails = stp->ant_fails;
                    message.has_stats_comms_ant_fails = true;
                }
                if (stp->oneshots) {
                    message.stats_oneshots = stp->oneshots;
                    message.has_stats_oneshots = true;
                }
                if (stp->oneshot_seconds) {
                    message.stats_oneshot_seconds = stp->oneshot_seconds;
                    message.has_stats_oneshot_seconds = true;
                }
                if (stp->motion_events) {
                    message.stats_motion_events = stp->motion_events;
                    message.has_stats_motion_events = true;
                }
            }
            StatType = "stats";
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
#elif G0==LND712U
        message.has_lnd_712u = true;
        message.lnd_712u = cpm0;
#elif G0==LND78017W
        message.has_lnd_78017w = true;
        message.lnd_78017w = cpm0;
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
#elif G1==LND712U
        message.has_lnd_712u = true;
        message.lnd_712u = cpm1;
#elif G1==LND78017W
        message.has_lnd_78017w = true;
        message.lnd_78017w = cpm1;
#endif
    }
#endif

    // Strip default values and 0 values from what is transmitted
    if (lat == 0.0 && lon == 0.0)
        isGPSDataAvailable = false;
    // If for any reason we get in here with our special gps loc, flag it as a test
    if (lat == 1.0 && lon == 1.0) {
        isGPSDataAvailable = false;
        isTestMeasurement = true;
    }
    if (isGPSDataAvailable) {
        message.latitude = lat;
        message.longitude = lon;
        message.has_latitude = message.has_longitude = true;
        if (haveAlt) {
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

#ifdef TWIBME0
    if (isEnvDataAvailable) {
        message.has_env_temp = true;
        message.env_temp = envTempC;
        message.has_env_humid = true;
        message.env_humid = envHumRH;
        message.has_env_pressure = true;
        message.env_pressure = envPressPA;
    }
#endif

#ifdef TWIBME1
    if (isEncDataAvailable) {
        message.has_enc_temp = true;
        message.enc_temp = encTempC;
        message.has_enc_humid = true;
        message.enc_humid = encHumRH;
        message.has_enc_pressure = true;
        message.enc_pressure = encPressPA;
    }
#endif

#ifdef PMSX
    if (isPMSDataAvailable) {
        if (pms_std01_0 != 0) {
            message.has_pms_std01_0 = true;
            message.pms_std01_0 = pms_std01_0;
        } else {
            message.has_pms_pm01_0 = true;
            message.pms_pm01_0 = pms_pm01_0;
        }
        if (pms_std02_5 != 0) {
            message.has_pms_std02_5 = true;
            message.pms_std02_5 = pms_std02_5;
        } else {
            message.has_pms_pm02_5 = true;
            message.pms_pm02_5 = pms_pm02_5;
        }
        if (pms_std10_0 != 0) {
            message.has_pms_std10_0 = true;
            message.pms_std10_0 = pms_std10_0;
        } else {
            message.has_pms_pm10_0 = true;
            message.pms_pm10_0 = pms_pm10_0;
        }
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
        if (opc_std01_0 != 0) {
            message.has_opc_std01_0 = true;
            message.opc_std01_0 = opc_std01_0;
        } else {
            message.has_opc_pm01_0 = true;
            message.opc_pm01_0 = opc_pm01_0;
        }
        if (opc_std02_5 != 0) {
            message.has_opc_std02_5 = true;
            message.opc_std02_5 = opc_std02_5;
        } else {
            message.has_opc_pm02_5 = true;
            message.opc_pm02_5 = opc_pm02_5;
        }
        if (opc_std10_0 != 0) {
            message.has_opc_std10_0 = true;
            message.opc_std10_0 = opc_std10_0;
        } else {
            message.has_opc_pm10_0 = true;
            message.opc_pm10_0 = opc_pm10_0;
        }
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

    // Determine based on operating mode
    switch (sensor_op_mode()) {

        // While we're developing mobile mode, flag the measurement
    case OPMODE_MOBILE:
        isTestMeasurement = true;
        break;

        // In all test modes, flag the measurement
    case OPMODE_TEST_FAST:
    case OPMODE_TEST_BURN:
    case OPMODE_TEST_SENSOR:
    case OPMODE_TEST_DEAD:
        isTestMeasurement = true;
        break;
    }

    // Mark the message if for any reason this is a test measurement
    if (!fBadlyLimitedMTU && isTestMeasurement) {
        message.test = true;
        message.has_test = true;
    }

    // Mark messages containing data to indicate whether we're currently in-motion,
    // to give the service an option to ignore the value when generating visualizations
    if (!fBadlyLimitedMTU && !isStatsRequest && gpio_in_motion()) {
        message.motion = true;
        message.has_motion = true;
    }

    // If we're debugging, add a message sequence number so we can trace data loss
    uint16_t bat_status = battery_status();
    if (bat_status == BAT_TEST || bat_status == BAT_BURN) {
        message.has_stats_seqno = true;
        message.stats_seqno = stats()->seqno++;
    }

    // If a stats request, make it a request/response to the service
    if (isStatsRequest || ((storage()->flags & FLAG_CONFIRM_ALL) != 0)) {
        message.has_reply_type = true;
        message.reply_type = ttproto_Telecast_replyType_ALLOWED;
        responseType = REPLY_TTSERVE;
    } else {
        responseType = REPLY_NONE;
    }

    // If it's a stats request, add a special "stamp" that tells the service
    // to buffer the values of certain rarely-changing fields.  Otherwise,
    // examine the message to see if we can "apply" the previously-transmitted
    // stamp, thus removing fields that can be supplied by TTSERVE.  This
    // is a major bandwidth optimization, even if there is some risk that
    // if this stats message is lost we may end up discarding measurements
    // because of lack of critical fields.
    bool stamp_created = false;
    if (isStatsRequest && UpdateType == UPDATE_STATS) {
        stamp_created = stamp_create(&message);
        if (stamp_created && debug(DBG_COMM_MAX))
            DEBUG_PRINTF("Stamp created: %lu\n", stamp_message_id);
    } else {
        bool fStamped = stamp_apply(&message);
        if (!fStamped && debug(DBG_COMM_MAX))
            DEBUG_PRINTF("*** Not stamped!\n");
    }

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

    if (fBuffered && !send_buff_is_full(bytes_written)) {

        // Buffer it
        fSent = send_buff_append(buffer, bytes_written, responseType);

    } else {

        if (send_buff_is_empty()) {

            // If this is larger than allowable MTU, don't bother
            if (bytes_written > comm_get_mtu() && !send_mtu_test_in_progress()) {

                fMTUFailure = true;

            } else {

                // If the buffer is empty, just send a single PB to the service
                fSent = send_to_service(buffer, bytes_written, responseType, SEND_1);

            }

        } else {
            // Append this to the existing buffer, remembering whether or not it succeeded.
            // Ultimately, if it isn't sent, we'll come back here to retry sending it.
            fSent = send_buff_append(buffer, bytes_written, responseType);

            // Regardless of whether or not it succeeded, we must transmit what's in the buffer
            // so that we don't get stuck forever with a full buffer.  Prepare for transmission
            // by fetching length and the requested response type.
            uint16_t send_response_type;
            uint8_t *xmit_buff = send_buff_prepare_for_transmit(&bytes_written, &send_response_type);

            // Determine whether or not we're being instructed to choose "efficient" or "reliable"
            // transport of buffered messages.  Neither is ideal; they both have tradeoffs.
            // Buffered messages contain a LOT of information, and so if they are lost there is
            // a lot to lose.  If we send "efficiently" via UDP, there is a high risk that it
            // will be lost  If we sent "reliably" via TCP or HTTP, the bandwidth costs more.
            // By default, we'll send them reliably.  We do so by leveraging the side-effect
            // that we know a reliable transport is used when requesting a reply from the service.
            if (send_response_type == REPLY_NONE) {
                if (!(storage()->flags & FLAG_BUFFERED_EFFICIENT))
                    send_response_type = REPLY_TTSERVE;
            }

            // If this is longer than the allowed mtu, don't even bother.
            if (bytes_written > comm_get_mtu() && !send_mtu_test_in_progress()) {

                fMTUFailure = true;
                send_buff_reset();

            } else {

                // Transmit the entire batch of buffered samples
                if (send_to_service(xmit_buff, bytes_written, send_response_type, SEND_N)) {
                    send_buff_reset();
                } else {
                    // If error AND if the append had succeeded, revert it
                    if (fSent)
                        send_buff_append_revert();
                }
            }

        }

    }

    // Display data about message
    char buff_msg[10];
    char sent_msg[300];
    char sb[32];
    if (send_length_buffered() == 0)
        buff_msg[0] = '\0';
    else {
        uint16_t messages_waiting_in_nvram = db_get(NULL, NULL, NULL);
        if (messages_waiting_in_nvram)
            sprintf(buff_msg, "/%db/%dm", send_length_buffered(), messages_waiting_in_nvram);
        else
            sprintf(buff_msg, "/%db", send_length_buffered());
    }

    sprintf(sent_msg, "%db%s", bytes_written, buff_msg);
    if (message.has_latitude || message.has_captured_at_date) {
        sprintf(sb, " M%s%s",
                message.has_captured_at_date ? "+" : "-",
                message.has_latitude ? "+" : "-");
        strcat(sent_msg, sb);
    }
    if (message.has_captured_at_date || message.has_captured_at_time) {
        strcat(sent_msg, " C+");
    }
    if (wasStatsRequest) {
        sprintf(sb, " S%s (%s)", wasStatsRequest ? (isStatsRequest ? "+" : "X") : "-", StatType);
        strcat(sent_msg, sb);
    }
    if (wasGeiger0DataAvailable || wasGeiger1DataAvailable) {
        sprintf(sb, " G%s%s",
                wasGeiger0DataAvailable ? (isGeiger0DataAvailable ? "+" : "X") : "-",
                wasGeiger1DataAvailable ? (isGeiger1DataAvailable ? "+" : "X") : "-");
        strcat(sent_msg, sb);
    }
    if (wasBatteryVoltageDataAvailable || wasBatterySOCDataAvailable || wasBatteryCurrentDataAvailable) {
        sprintf(sb, " V%s%s%s",
                wasBatteryVoltageDataAvailable ? (isBatteryVoltageDataAvailable ? "+" : "X") : "-",
                wasBatterySOCDataAvailable ? (isBatterySOCDataAvailable ? "+" : "X") : "-",
                wasBatteryCurrentDataAvailable ? (isBatteryCurrentDataAvailable ? "+" : "X") : "-");
        strcat(sent_msg, sb);
    }
    if (wasEnvDataAvailable || wasEncDataAvailable) {
        sprintf(sb, " E%s%s",
                wasEnvDataAvailable ? (isEnvDataAvailable ? "+" : "X") : "-",
                wasEncDataAvailable ? (isEncDataAvailable ? "+" : "X") : "-");
        strcat(sent_msg, sb);
    }
    if (wasPMSDataAvailable) {
        sprintf(sb, " Pm%s", wasPMSDataAvailable ? (isPMSDataAvailable ? "+" : "X") : "-");
        strcat(sent_msg, sb);
    }
    if (wasOPCDataAvailable) {
        sprintf(sb, " Op%s", wasOPCDataAvailable ? (isOPCDataAvailable ? "+" : "X") : "-");
        strcat(sent_msg, sb);
    }

    if (fMTUFailure) {
        stats()->mtu_failures++;
        strlcpy(mtu_failure, sent_msg, sizeof(mtu_failure)-1);
        DEBUG_PRINTF("** Message length %d > %d max MTU\n", bytes_written, comm_get_mtu());
    } else {
        if (fSent) {
            mtu_count++;
            if (bytes_written > mtu_max)
                mtu_max = bytes_written;
        }
    }

    if (fMTUFailure || fSent || debug(DBG_COMM_MAX))
        DEBUG_PRINTF("%s %s\n", fMTUFailure ? "FAIL" : (fSent ? (fBuffered ? "BUFF" : "SENT") : "WAIT"), sent_msg);

    // If we exceeded MTU with this message, DISCARD the data because otherwise we will be stuck
    // in an infinite retry loop trying to send the same data over and over.
    if (fMTUFailure)
        fSent = true;

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
#ifdef TWIBME0
    if (isEnvDataAvailable)
        s_bme280_0_clear_measurement();
#endif
#ifdef TWIBME1
    if (isEncDataAvailable)
        s_bme280_1_clear_measurement();
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

    return true;

}

// Display failures that should come to our attention
void mtu_status_check(bool fVerbose) {
    if (fVerbose)
        DEBUG_PRINTF("MTU max%d of limit %d in %ld messages\n", mtu_max, comm_get_mtu(), mtu_count);
    if (stats()->mtu_failures) {
        DEBUG_PRINTF("** %ld MTU fails: %s\n", stats()->mtu_failures, mtu_failure);
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

    // If the request format is to "send 1", we need to reformat it to be a "batch send" request because
    // that's the only format that we support as of March 2017.  We do this by using the send buffer, which
    // we know isn't in use.
    if (RequestFormat == SEND_1) {
        send_buff_reset();
        send_buff_append(buffer, length, RequestType);
        buffer = send_buff_prepare_for_transmit(&length, &RequestType);
    }

    // Transmit or buffer it.  Just for flash write-leveling wear and tear, we only do this right now
    // when in mobile mode, although it works well in any mode.
    if (comm_db_is_active() && sensor_op_mode() == OPMODE_MOBILE)
        fTransmitted = db_put(buffer, length, RequestType);
    else
        fTransmitted = comm_send_to_service(buffer, length, RequestType);

    // If we buffered this because of a SEND_1, reset the buffer
    if (RequestFormat == SEND_1)
        send_buff_reset();

    // Done
    transmitInProgress--;
    return fTransmitted;

}

// Transmit a binary message to the service on the currently-active transport,
// failing the send if we aren't through with init.
bool send_to_service(uint8_t *buffer, uint16_t length, uint16_t RequestType, uint16_t RequestFormat) {

    // Exit if we aren't in a state where we can transmit and we aren't prepared to buffer it
    if (!comm_can_send_to_service() && !comm_db_is_active())
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
    if (pingtype == REPLY_NONE) {
        message.has_device_type = true;
        message.device_type = ttproto_Telecast_deviceType_TTAPP;
    } else if (pingtype == REPLY_TTSERVE) {
        message.has_device_type = true;
        message.device_type = ttproto_Telecast_deviceType_TTSERVE;
    } else if (pingtype == REPLY_TTGATE) {
        message.has_device_type = true;
        message.device_type = ttproto_Telecast_deviceType_TTGATEPING;
    }

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
