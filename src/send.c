// Communications state machine processing

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
#include "bme.h"
#include "ina.h"
#include "twi.h"
#include "storage.h"
#include "nrf_delay.h"
#include "teletype.pb.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "app_scheduler.h"

// Communications statistics
static uint32_t stats_transmitted = 0L;
static uint32_t stats_received = 0L;
static uint32_t stats_resets = 0L;
static uint32_t stats_power_fails = 0L;
static uint32_t stats_oneshots = 0L;
static uint32_t stats_motiondrops = 0L;
static uint32_t stats_last_minute = 0L;
static uint32_t stats_uptime_minutes = 0L;
static uint32_t stats_uptime_hours = 0L;
static uint32_t stats_uptime_days = 0L;
static char stats_cell_iccid[40] = "";
static char stats_cell_cpsi[128] = "";

// Transmit a  message to the service, or suppress it if too often
bool send_update_to_service(uint16_t UpdateType) {
    bool isStatsRequest = (UpdateType != UPDATE_NORMAL);
    bool fSent;

    // Exit if we haven't yet completed LPWAN init or if power is turned off comms devices
    if (!comm_can_send_to_service())
        return false;

    // Exit if we're in DFU mode, because we shouldn't be sending anything
    if (storage()->dfu_status == DFU_PENDING)
        return false;
    
    // Determine whether or not we'll upload particle counts
    bool fUploadParticleCounts = ((storage()->sensors & SENSOR_AIR_COUNTS) != 0);
    UNUSED_VARIABLE(fUploadParticleCounts);

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

#if defined(TWIMAX17043) || defined(TWIINA219)
    float batteryVoltage, batterySOC;
#endif

#ifdef TWIINA219
    float batteryCurrent;
    isBatteryVoltageDataAvailable =
        isBatterySOCDataAvailable =
        isBatteryCurrentDataAvailable = s_ina_get_value(&batteryVoltage, &batterySOC, &batteryCurrent);
#endif

#ifdef TWIMAX17043
    isBatteryVoltageDataAvailable = s_bat_voltage_get_value(&batteryVoltage);
    isBatterySOCDataAvailable = s_bat_soc_get_value(&batterySOC);
#endif

#ifdef TWIHIH6130
    float envTempC, envHumRH;
    isEnvDataAvailable = s_hih6130_get_value(&envTempC, &envHumRH);
#endif

#ifdef TWIBME280
    float envTempC, envHumRH, envPressPA;
    isEnvDataAvailable = s_bme280_get_value(&envTempC, &envHumRH, &envPressPA);
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

    // Get motion data, and (unless this is a stats request) don't upload anything if moving
    if (!isStatsRequest && gpio_motion_sense(MOTION_QUERY)) {
        DEBUG_PRINTF("DROP: device is currently in-motion\n");
        return false;
    }

    // Get Geiger info
#ifdef GEIGER
    uint32_t cpm0, cpm1;
    // Get the geiger values
    s_geiger_get_value(&isGeiger0DataAvailable, &cpm0, &isGeiger1DataAvailable, &cpm1);
#endif

    // Show the POTENTIAL things we might transmit
#ifndef SUPPRESSSENDDEBUG
    bool wasStatsRequest = isStatsRequest;
    bool wasGeiger0DataAvailable = isGeiger0DataAvailable;
    bool wasGeiger1DataAvailable = isGeiger1DataAvailable;
    bool wasBatteryVoltageDataAvailable = isBatteryVoltageDataAvailable;
    bool wasBatterySOCDataAvailable = isBatterySOCDataAvailable;
    bool wasBatteryCurrentDataAvailable = isBatteryCurrentDataAvailable;
    bool wasEnvDataAvailable = isEnvDataAvailable;
    bool wasPMSDataAvailable = isPMSDataAvailable;
    bool wasOPCDataAvailable = isOPCDataAvailable;
#endif

    // Prio #1: If it's a stats request (which is large)
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

    // If we're on the LPWAN AND uploading counts, do some optimization to ensure that we don't generate a huge message
#ifdef LORA
    if (comm_mode() == COMM_LORA && fUploadParticleCounts) {

        // Prio #2: If it's a OPC request, suppress some things in an attempt to reduce size
        if (isOPCDataAvailable) {
            isBatteryVoltageDataAvailable = false;
            isBatterySOCDataAvailable = false;
            isBatteryCurrentDataAvailable = false;
            isEnvDataAvailable = false;
            isPMSDataAvailable = false;
        }

        // Prio #3: If it's an PMS request, suppress some things in an attempt to reduce size
        if (isPMSDataAvailable) {
            isBatteryVoltageDataAvailable = false;
            isBatterySOCDataAvailable = false;
            isBatteryCurrentDataAvailable = false;
            isEnvDataAvailable = false;
            isOPCDataAvailable = false;
        }

    }
#endif

    // If nothing is available that would motivate us to send a standalone message, then exit.
    // Note that other things may be avail, but we only "piggyback" them onto these reports.
#if defined(GEIGER) || defined(SPIOPC) || defined(PMSX) || defined(AIRX)
    if (!isStatsRequest &&
        !isGeiger0DataAvailable &&
        !isGeiger1DataAvailable &&
        !isPMSDataAvailable &&
        !isOPCDataAvailable) {
        if (debug(DBG_COMM_MAX))
            DEBUG_PRINTF("DROP: Substantive data not available\n");
        return false;
    }
#else
    // In the case where those sensors aren't configured, bail if truly nothing to send
    if (!isStatsRequest &&
        !isBatteryVoltageDataAvailable &&
        !isBatterySOCDataAvailable &&
        !isBatteryCurrentDataAvailable &&
        !isEnvDataAvailable) {
        if (debug(DBG_COMM_MAX))
            DEBUG_PRINTF("DROP: Substantive bat/env data not available\n");
        return false;
    }
#endif

    // Format for transmission
    uint16_t responseType;
    uint16_t status;
    uint8_t buffer[350];
    teletype_Telecast message = teletype_Telecast_init_zero;
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

    // Build the message
    message.DeviceType = teletype_Telecast_deviceType_SOLARCAST;

    // All messages carry the Device ID
    message.has_DeviceIDNumber = true;
    message.DeviceIDNumber = deviceID;

    // If we've got a local capture date/time from GPS, enclose it
    uint32_t date, time, offset;
    if (get_current_timestamp(&date, &time, &offset)) {
        message.CapturedAtDate = date;
        message.CapturedAtTime = time;
        message.CapturedAtOffset = offset;
        message.has_CapturedAtDate = message.has_CapturedAtTime = message.has_CapturedAtOffset = true;
    }
    
    // Process stats
    if (isStatsRequest) {

        message.stats_uptime_minutes = (((stats_uptime_days * 24) + stats_uptime_hours) * 60) + stats_uptime_minutes;
        message.has_stats_uptime_minutes = true;

        switch (UpdateType) {

        case UPDATE_STATS_VERSION:
            strncpy(message.stats_app_version, app_version(), sizeof(message.stats_app_version));
            message.has_stats_app_version = true;
            break;

        case UPDATE_STATS_CONFIG_DEV:
            message.has_stats_device_params = storage_get_device_params_as_string(message.stats_device_params, sizeof(message.stats_device_params));
            break;

        case UPDATE_STATS_CONFIG_GPS:
            message.has_stats_device_params = storage_get_gps_params_as_string(message.stats_device_params, sizeof(message.stats_device_params));
            break;

        case UPDATE_STATS_CONFIG_SVC:
            message.has_stats_device_params = storage_get_service_params_as_string(message.stats_device_params, sizeof(message.stats_device_params));
            break;

        case UPDATE_STATS_CONFIG_SEN:
            message.has_stats_device_params = storage_get_sensor_params_as_string(message.stats_device_params, sizeof(message.stats_device_params));
            break;

        case UPDATE_STATS_DFU:
            message.has_stats_dfu = storage_get_dfu_state_as_string(message.stats_dfu, sizeof(message.stats_dfu));
            break;

        case UPDATE_STATS_CELL1:
#ifdef FONA
            if (stats_cell_iccid[0] != '\0') {
                strncpy(message.stats_cell, stats_cell_iccid, sizeof(message.stats_cell));
                message.has_stats_cell = true;
            }
#endif
            break;

        case UPDATE_STATS_CELL2:
#ifdef FONA
            if (stats_cell_iccid[0] != '\0') {
                strncpy(message.stats_cell, stats_cell_cpsi, sizeof(message.stats_cell));
                message.has_stats_cell = true;
            }
#endif
            break;

        case UPDATE_STATS:
            message.stats_transmitted_bytes = stats_transmitted;
            message.has_stats_transmitted_bytes = true;
            message.stats_received_bytes = stats_received;
            message.has_stats_received_bytes = true;
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
            if (stats_motiondrops) {
                message.stats_motiondrops = stats_motiondrops;
                message.has_stats_motiondrops = true;
            }
            break;

        }
    }

#ifdef GEIGER
    if (isGeiger0DataAvailable) {
        message.has_cpm0 = true;
        message.cpm0 = cpm0;
    }
    if (isGeiger1DataAvailable) {
        message.has_cpm1 = true;
        message.cpm1 = cpm1;
    }
#endif

    if (isGPSDataAvailable) {
        message.Latitude = lat;
        message.Longitude = lon;
        message.has_Latitude = message.has_Longitude = true;
        if (haveAlt) {
            message.Altitude = (uint32_t) alt;
            message.has_Altitude = true;
        }
    }

#if defined(TWIMAX17043) || defined(TWIINA219)
    if (isBatteryVoltageDataAvailable) {
        message.has_BatteryVoltage = true;
        message.BatteryVoltage = batteryVoltage;
    }
    if (isBatterySOCDataAvailable) {
        message.has_BatterySOC = true;
        message.BatterySOC = batterySOC;
    }
#endif

#ifdef TWIINA219
    if (isBatteryCurrentDataAvailable) {
        message.has_BatteryCurrent = true;
        message.BatteryCurrent = batteryCurrent;
    }
#endif

#ifdef TWIHIH6130
    if (isEnvDataAvailable) {
        message.has_envTemperature = true;
        message.envTemperature = envTempC;
        message.has_envHumidity = true;
        message.envHumidity = envHumRH;
    }
#endif

#ifdef TWIBME280
    if (isEnvDataAvailable) {
        message.has_envTemperature = true;
        message.envTemperature = envTempC;
        message.has_envHumidity = true;
        message.envHumidity = envHumRH;
        message.has_envPressure = true;
        message.envPressure = envPressPA;
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

// If a stats request, make it a request/response to the service
    if (isStatsRequest)
        responseType = REPLY_TTSERVE;
    else
        responseType = REPLY_NONE;

// Encode the message
    status = pb_encode(&stream, teletype_Telecast_fields, &message);
    if (!status) {
        DEBUG_PRINTF("Send pb_encode: %s\n", PB_GET_ERROR(&stream));
        return false;
    }

// Transmit it
    fSent = send_to_service("data", buffer, stream.bytes_written, responseType);

// Debug
#ifndef SUPPRESSSENDDEBUG
    DEBUG_PRINTF("%s %db S%s G%s%s V%s%s%s E%s Pm%s Op%s\n",
                 fSent ? "SENT" : "DROP",
                 stream.bytes_written,
                 wasStatsRequest ? (isStatsRequest ? "+" : "X") : "-",
                 wasGeiger0DataAvailable ? (isGeiger0DataAvailable ? "+" : "X") : "-",
                 wasGeiger1DataAvailable ? (isGeiger1DataAvailable ? "+" : "X") : "-",
                 wasBatteryVoltageDataAvailable ? (isBatteryVoltageDataAvailable ? "+" : "X") : "-",
                 wasBatterySOCDataAvailable ? (isBatterySOCDataAvailable ? "+" : "X") : "-",
                 wasBatteryCurrentDataAvailable ? (isBatteryCurrentDataAvailable ? "+" : "X") : "-",
                 wasEnvDataAvailable ? (isEnvDataAvailable ? "+" : "X") : "-",
                 wasPMSDataAvailable ? (isPMSDataAvailable ? "+" : "X") : "-",
                 wasOPCDataAvailable ? (isOPCDataAvailable ? "+" : "X") : "-");
#endif

// Exit if it didn't get out
    if (!fSent)
        return false;

// Clear them once transmitted successfully
#ifdef GEIGER
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
        s_bme280_clear_measurement();
#endif
#ifdef TWIMAX17043
    if (isBatteryVoltageDataAvailable)
        s_bat_voltage_clear_measurement();
    if (isBatterySOCDataAvailable)
        s_bat_soc_clear_measurement();
#endif
#ifdef TWIINA219
    if (isBatteryCurrentDataAvailable)
        s_ina_clear_measurement();
#endif
#if defined (GEIGER)
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

// Transmit a binary message to the service on the currently-active transport, even
// if the transport is busy or if we're in init.
bool send_to_service_unconditionally(char *what, uint8_t *buffer, uint16_t length, uint16_t RequestType) {
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
        fTransmitted = lora_send_to_service(what, buffer, length, RequestType);
        break;
#endif
#ifdef FONA
    case COMM_FONA:
        fTransmitted = fona_send_to_service(what, buffer, length, RequestType);
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
bool send_to_service(char *what, uint8_t *buffer, uint16_t length, uint16_t RequestType) {

    // Exit if we aren't in a state where we can transmit
    if (!comm_can_send_to_service())
        return false;

    // Send it.
    return(send_to_service_unconditionally(what, buffer, length, RequestType));

}

// Send a ping message to the LPWAN.  This may be done to
// see if we get a reply from TTGATE, or TTSERVE (via a TTNGATE),
// or potentially nothing if we're out of range.
bool send_ping_to_service(uint16_t pingtype) {
    uint16_t status;
    uint8_t buffer[64];

    // Allocate space on the stack to store the message data.
    teletype_Telecast message = teletype_Telecast_init_zero;

    /* Create a stream that will write to our buffer. */
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

    // Build the message in a very minimal way, assigning the device type
    // as an indicator of the kind of a ping.
    if (pingtype == REPLY_NONE)
        message.DeviceType = teletype_Telecast_deviceType_TTAPP;
    else if (pingtype == REPLY_TTSERVE)
        message.DeviceType = teletype_Telecast_deviceType_TTSERVE;
    else if (pingtype == REPLY_TTGATE)
        message.DeviceType = teletype_Telecast_deviceType_TTGATE;

    // Use our device ID, so we know when it comes back that it was from us
    message.DeviceIDNumber = io_get_device_address();
    message.has_DeviceIDNumber = true;

    // encode it and transmit it
    status = pb_encode(&stream, teletype_Telecast_fields, &message);
    if (!status) {
        DEBUG_PRINTF("pb_encode: %s\n", PB_GET_ERROR(&stream));
        return false;
    }

    // Transmit it, even if we're in the middle of init
    if (!send_to_service_unconditionally("ping", buffer, stream.bytes_written, pingtype))
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
                // Restart the device when appropriate
                if (storage()->restart_days != 0 && stats_uptime_days >= storage()->restart_days)
                    io_request_restart();
            }
        }
    }
}

// Set the cell info
void stats_set_cell_info(char *iccid, char *cpsi) {
    if (iccid != NULL)
        strncpy(stats_cell_iccid, iccid, sizeof(stats_cell_iccid));
    if (cpsi != NULL)
        strncpy(stats_cell_cpsi, cpsi, sizeof(stats_cell_cpsi));
}

// Bump critical statistics
void stats_add(uint16_t transmitted, uint16_t received, uint16_t resets, uint16_t powerfails, uint16_t oneshots, uint16_t motiondrops) {
    stats_transmitted += transmitted;
    stats_received += received;
    stats_resets += resets;
    stats_power_fails += powerfails;
    stats_oneshots += oneshots;
    stats_motiondrops += motiondrops;
}
