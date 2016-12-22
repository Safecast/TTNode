// Phone state machine processing

#include <stdio.h>
#include <stdlib.h>
#include "debug.h"
#include "bt.h"
#include "config.h"
#include "comm.h"
#include "lora.h"
#include "fona.h"
#include "send.h"
#include "recv.h"
#include "misc.h"
#include "timer.h"
#include "sensor.h"
#include "gpio.h"
#include "geiger.h"
#include "io.h"
#include "serial.h"
#include "twi.h"
#include "ublox.h"
#include "storage.h"
#include "bme.h"
#include "ina.h"
#include "nrf_delay.h"
#include "teletype.pb.h"
#include "pb_encode.h"
#include "pb_decode.h"

// Device states
#define CMD_STATE_XMIT_PHONE_TEXT       COMM_STATE_DEVICE_START+0

// Command buffer
static cmdbuf_t fromPhone;

// Transmit text to the phone
void phone_send(char *msg) {

    // If not initialized, get out
    if (!comm_is_initialized())
        return;

    // If we're optimizing power, we don't even want to come down this path
    // because we're wasting time.
    if (io_optimize_power())
        return;

    // Send it
    while (*msg != '\0')
        send_byte_to_bluetooth(*msg++);

    // Send terminating newline
    send_byte_to_bluetooth('\r');
    send_byte_to_bluetooth('\n');

}

// Process a "complete" command buffer, and if idle parse it to determine its first state
void phone_complete() {

    // If we're not in an idle state, let's not process commands for it
    if (fromPhone.state != COMM_STATE_IDLE)
        return;

    for (;;) {

        // If it begins with a slash, transmit this oon the wire as a pb-formatted "text message"
        if (comm_cmdbuf_this_arg_is(&fromPhone, "/*")) {
            // Skip to the actual text to be transmitted
            comm_cmdbuf_next_arg(&fromPhone);
            fromPhone.state = CMD_STATE_XMIT_PHONE_TEXT;
            break;
        }

        // Process our hard-wired commands

#ifdef LORA
        // Commands to be passed-through to the LPWAN chip - used for testing
        if (comm_cmdbuf_this_arg_is(&fromPhone, "sys") || comm_cmdbuf_this_arg_is(&fromPhone, "mac") || comm_cmdbuf_this_arg_is(&fromPhone, "radio")) {
            // Convert to lowercase because the LPWAN chip requires this
            int i;
            for (i = 0; i < fromPhone.length; i++)
                if (fromPhone.buffer[i] >= 'A' && fromPhone.buffer[i] <= 'Z')
                    fromPhone.buffer[i] += 'a' - 'A';
            // Enter command mode.  This may fail the first time because it's busy,
            // but then after the next timeout we will be  in that state.
            lora_enter_command_mode();
            // Send to the LPWAN chip, even if it may fail because of the state we're in
            lora_send((char *)&fromPhone.buffer[1]);
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }
#endif // LORA

#ifdef FONA
        // Commands to be passed-through to the chip - used for testing
        if (comm_cmdbuf_this_arg_is(&fromPhone, "at+*")) {
            // Send to the chip, even if it may fail because of the state we're in
            fona_send((char *)&fromPhone.buffer[1]);
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }
#endif

        // Geiger request
#ifdef GEIGER
        if (comm_cmdbuf_this_arg_is(&fromPhone, "rad") || comm_cmdbuf_this_arg_is(&fromPhone, "cpm")) {
            uint32_t cpm0, cpm1;
            s_geiger_get_value(NULL, &cpm0, NULL, &cpm1);
            DEBUG_PRINTF("0:%lucpm 1:%lucpm\n", cpm0, cpm1);
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }
#endif

        // Battery level request
#if defined(TWIMAX17043) || defined(TWIINA219)
        float batteryVoltage, batterySOC;
        if (comm_cmdbuf_this_arg_is(&fromPhone, "bat")) {
            gpio_power_set(POWER_PIN_BASICS, true);
#if defined(TWIMAX17043)
            s_bat_voltage_init();
            s_bat_voltage_measure(NULL);
            s_bat_voltage_get_value(&batteryVoltage);
            s_bat_voltage_term();
            s_bat_soc_init();
            s_bat_soc_measure(NULL);
            s_bat_soc_get_value(&batterySOC);
            s_bat_soc_term();
            DEBUG_PRINTF("MAX: %fV %.3f%%\n", batteryVoltage, batterySOC);
#endif
#if defined(TWIINA219)
            float batteryCurrent;
            s_ina_init();
            s_ina_measure(NULL);
            s_ina_get_value(&batteryVoltage, &batterySOC, &batteryCurrent);
            s_ina_term();
            if (debug(DBG_SENSOR))
                DEBUG_PRINTF("INA: %fV %.3f%% %fmA\n", batteryVoltage, batterySOC, batteryCurrent);
#endif
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }
#endif // TWIMAX17043

        // Force cellular to test failover behavior
        if (comm_cmdbuf_this_arg_is(&fromPhone, "fail")) {
            comm_force_cell();
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }

        // Force cellular
#if defined(FONA)
        if (comm_cmdbuf_this_arg_is(&fromPhone, "fona")) {
            comm_select(COMM_FONA);
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }
#endif

        // Force lora
#if defined(LORA)
        if (comm_cmdbuf_this_arg_is(&fromPhone, "lora")) {
            comm_select(COMM_LORA);
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }
#endif

        // Force none
        if (comm_cmdbuf_this_arg_is(&fromPhone, "none")) {
            comm_select(COMM_NONE);
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }

        // Show Sensor State request
        if (comm_cmdbuf_this_arg_is(&fromPhone, "sss")) {
            comm_show_state();
            sensor_show_state();
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }

        // GPS "set to fake data" request
        if (comm_cmdbuf_this_arg_is(&fromPhone, "gx")) {
            char buffer[256];
            storage_set_gps_params_as_string("1.23/4.56/7.89");
            storage_save();
            storage_get_gps_params_as_string(buffer, sizeof(buffer));
            DEBUG_PRINTF("Now %s\n", buffer);
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }

        // GPS abort and go to "last known Good GPS"
        if (comm_cmdbuf_this_arg_is(&fromPhone, "gg")) {
            comm_gps_abort();
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }

        // GPS request
        if (comm_cmdbuf_this_arg_is(&fromPhone, "gps")) {
            comm_cmdbuf_next_arg(&fromPhone);
            switch (fromPhone.buffer[fromPhone.args]) {
            case '\0': {
                float lat, lon, alt;
                uint16_t status = comm_gps_get_value(&lat, &lon, &alt);
                if (status == GPS_LOCATION_FULL)
                    DEBUG_PRINTF("Have lat+lon+alt.\n");
                else if (status == GPS_LOCATION_PARTIAL)
                    DEBUG_PRINTF("Have lat+lon, no alt.\n");
                else if (status == GPS_NO_LOCATION)
                    DEBUG_PRINTF("No location.\n");
                else
                    DEBUG_PRINTF("No data.\n");
                break;
            }
            }
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }

        // GPIO test request
        if (comm_cmdbuf_this_arg_is(&fromPhone, "gpio")) {
            comm_cmdbuf_next_arg(&fromPhone);
            if (fromPhone.buffer[fromPhone.args] != '\0') {
                uint16_t num = atoi((char *)&fromPhone.buffer[fromPhone.args]);
                uint16_t pin = num/10;
                bool fOn = ((num & 0x01) != 0);
                gpio_power_init(pin, fOn);
            }
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }

        // Temperature/Humidity request
#if defined(TWIHIH6130) || defined(TWIBME280)
        if (comm_cmdbuf_this_arg_is(&fromPhone, "temp") || comm_cmdbuf_this_arg_is(&fromPhone, "env")) {
            float envTempC, envHumRH;
            float envPress = 0.0;
#ifdef TWIHIH6130
            s_hih6130_get_value(&envTempC, &envHumRH);
#endif
#ifdef TWIBME280
            s_bme280_get_value(&envTempC, &envHumRH, &envPress);
#endif
            DEBUG_PRINTF("%f degC, %f pctRH, %f Pa\n", envTempC, envHumRH, envPress);
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }
#endif

        // Request state
        if (comm_cmdbuf_this_arg_is(&fromPhone, "state")) {
            comm_request_state();
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }

        // Request statistics
        if (comm_cmdbuf_this_arg_is(&fromPhone, "stats")) {
            send_update_to_service(UPDATE_STATS);
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }

        // Soft reset request
        if (comm_cmdbuf_this_arg_is(&fromPhone, "reset")) {
            comm_reset(true);
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }

        // Force us to drop everything power hungry and optimize power
        if (comm_cmdbuf_this_arg_is(&fromPhone, "drop")) {
            io_force_optimize_power();
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }

        // Turn off indicators, usually when measuring power
        if (comm_cmdbuf_this_arg_is(&fromPhone, "ind")) {
            gpio_indicators_off();
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }

        // Force us to drop everything power hungry and optimize power
        if (comm_cmdbuf_this_arg_is(&fromPhone, "expire")) {
#ifdef TWIUBLOXM8
            s_gps_shutdown();
#endif
#ifdef FONA
            fona_gps_shutdown();
#endif
            force_all_timer_expiration();
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }

#ifdef LORA
        // Request to change into "listen" mode so we can recive messages
        if (comm_cmdbuf_this_arg_is(&fromPhone, "listen")) {
            comm_cmdbuf_next_arg(&fromPhone);
            lora_set_listen_tags((char *) &fromPhone.buffer[fromPhone.args]);
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }
#endif

        // Request to enable verbose LPWAN rx/tx logging
        if (comm_cmdbuf_this_arg_is(&fromPhone, "d")) {
            char flags[40];
            strcpy(flags, "");
            if (debug(DBG_RX) && debug(DBG_TX))
                strcat(flags, "C ");
            else if (!debug(DBG_RX) && !debug(DBG_TX))
                strcat(flags, "c ");
            else {
                strcat(flags, debug(DBG_RX) ? "R " : "r ");
                strcat(flags, debug(DBG_TX) ? "T " : "t ");
            }
            strcat(flags, debug(DBG_COMM_MAX) ? "CX " : "cx ");
            strcat(flags, debug(DBG_SENSOR) ? "S " : "s ");
            strcat(flags, debug(DBG_SENSOR_MAX) ? "SX " : "sx ");
            strcat(flags, debug(DBG_SENSOR_SUPERMAX) ? "SXX " : "sxx ");
            strcat(flags, debug(DBG_GPS_MAX) ? "GX " : "gx ");
            strcat(flags, debug(DBG_AIR) ? "A " : "a ");
            DEBUG_PRINTF("DEBUG: %s\n", flags);
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }
        if (comm_cmdbuf_this_arg_is(&fromPhone, "0")) {
            debug_flags_set(DBG_NONE);
            DEBUG_PRINTF("ALL debug OFF\n");
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }
        if (comm_cmdbuf_this_arg_is(&fromPhone, "1") || comm_cmdbuf_this_arg_is(&fromPhone, ".")) {
            debug_flags_set(DBG_COMMON);
            DEBUG_PRINTF("Common debug ON\n");
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }
        if (comm_cmdbuf_this_arg_is(&fromPhone, "x")) {
            debug_flags_set(DBG_COMMON|DBG_GPS_MAX|DBG_SENSOR_MAX|DBG_COMM_MAX);
            DEBUG_PRINTF("ALL debug ON\n");
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }
        if (comm_cmdbuf_this_arg_is(&fromPhone, "r")) {
            DEBUG_PRINTF("RX toggled to %s\n", debug_flag_toggle(DBG_RX) ? "ON" : "OFF");
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }
        if (comm_cmdbuf_this_arg_is(&fromPhone, "t")) {
            DEBUG_PRINTF("TX toggled to %s\n", debug_flag_toggle(DBG_TX) ? "ON" : "OFF");
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }
        if (comm_cmdbuf_this_arg_is(&fromPhone, "c")) {
            if (debug(DBG_RX) != debug(DBG_TX))
                debug_flags_set(DBG_RX|DBG_TX);
            DEBUG_PRINTF("COMM toggled to %s\n", debug_flag_toggle(DBG_RX|DBG_TX) ? "ON" : "OFF");
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }
        if (comm_cmdbuf_this_arg_is(&fromPhone, "cx")) {
            debug_flags_set(DBG_RX|DBG_TX|DBG_COMM_MAX);
            DEBUG_PRINTF("COMMMAX toggled to %s\n", debug_flag_toggle(DBG_COMM_MAX) ? "ON" : "OFF");
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }
        if (comm_cmdbuf_this_arg_is(&fromPhone, "a")) {
            DEBUG_PRINTF("AIR toggled to %s\n", debug_flag_toggle(DBG_AIR) ? "ON" : "OFF");
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }
        if (comm_cmdbuf_this_arg_is(&fromPhone, "s")) {
            DEBUG_PRINTF("SENSOR toggled to %s\n", debug_flag_toggle(DBG_SENSOR) ? "ON" : "OFF");
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }
        if (comm_cmdbuf_this_arg_is(&fromPhone, "sx")) {
            DEBUG_PRINTF("SENSORMAX toggled to %s\n", debug_flag_toggle(DBG_SENSOR_MAX) ? "ON" : "OFF");
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }
        if (comm_cmdbuf_this_arg_is(&fromPhone, "sxx")) {
            DEBUG_PRINTF("SENSORSUPERMAX toggled to %s\n", debug_flag_toggle(DBG_SENSOR_SUPERMAX) ? "ON" : "OFF");
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }
        if (comm_cmdbuf_this_arg_is(&fromPhone, "g")) {
            DEBUG_PRINTF("GPSMAX toggled to %s\n", debug_flag_toggle(DBG_GPS_MAX) ? "ON" : "OFF");
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }
        if (comm_cmdbuf_this_arg_is(&fromPhone, "gx")) {
            DEBUG_PRINTF("GPSMAX toggled to %s\n", debug_flag_toggle(DBG_GPS_MAX) ? "ON" : "OFF");
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }

        // Get version
        if (comm_cmdbuf_this_arg_is(&fromPhone, "ver")) {
            DEBUG_PRINTF("%s\n", app_version());
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }

        // Emulate what would happen if we got a service message directed at us
        if (comm_cmdbuf_this_arg_is(&fromPhone, "recv")) {
            comm_cmdbuf_next_arg(&fromPhone);
            recv_message_from_service((char *)&fromPhone.buffer[fromPhone.args]);
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }            

        // Get/Set Device Parameters
        if (comm_cmdbuf_this_arg_is(&fromPhone, "cfgdev") || comm_cmdbuf_this_arg_is(&fromPhone, "config")) {
            char buffer[256];
            comm_cmdbuf_next_arg(&fromPhone);
            comm_cmdbuf_this_arg_is(&fromPhone, "*");
            if (fromPhone.buffer[fromPhone.args] == '\0') {
                storage_get_device_params_as_string(buffer, sizeof(buffer));
                DEBUG_PRINTF("%s %s\n", buffer, storage_get_device_params_as_string_help());
            } else {
                if (comm_cmdbuf_this_arg_is(&fromPhone,"l")) {
                    storage_load();
                } else if (comm_cmdbuf_this_arg_is(&fromPhone,"d")) {
                    storage_set_to_default();
                    storage_save();
                } else if (comm_cmdbuf_this_arg_is(&fromPhone,"test")) {
                    storage_set_device_params_as_string("0.1.0.10");
                    storage_save();
                } else {
                    storage_set_device_params_as_string((char *)&fromPhone.buffer[fromPhone.args]);
                    storage_save();
                }
                storage_get_device_params_as_string(buffer, sizeof(buffer));
                DEBUG_PRINTF("Now %s\n", buffer);
            }
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }

        // Get/Set Service Parameters
        if (comm_cmdbuf_this_arg_is(&fromPhone, "cfgnet")) {
            char buffer[256];
            comm_cmdbuf_next_arg(&fromPhone);
            comm_cmdbuf_this_arg_is(&fromPhone, "*");
            if (fromPhone.buffer[fromPhone.args] == '\0') {
                storage_get_service_params_as_string(buffer, sizeof(buffer));
                DEBUG_PRINTF("%s %s\n", buffer, storage_get_service_params_as_string_help());
            } else {
                storage_set_service_params_as_string((char *)&fromPhone.buffer[fromPhone.args]);
                storage_save();
                storage_get_service_params_as_string(buffer, sizeof(buffer));
                DEBUG_PRINTF("Now %s\n", buffer);
            }
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }

        // Get/Set Sensor Parameters
        if (comm_cmdbuf_this_arg_is(&fromPhone, "cfgsen")) {
            char buffer[256];
            comm_cmdbuf_next_arg(&fromPhone);
            comm_cmdbuf_this_arg_is(&fromPhone, "*");
            if (fromPhone.buffer[fromPhone.args] == '\0') {
                storage_get_sensor_params_as_string(buffer, sizeof(buffer));
                DEBUG_PRINTF("Current: '%s' Help: '%s'\n", buffer, storage_get_sensor_params_as_string_help());
            } else {
                storage_set_sensor_params_as_string((char *)&fromPhone.buffer[fromPhone.args]);
                storage_save();
                storage_get_sensor_params_as_string(buffer, sizeof(buffer));
                DEBUG_PRINTF("Now %s\n", buffer);
            }
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }

        // Get/Set GPS Parameters
        if (comm_cmdbuf_this_arg_is(&fromPhone, "cfggps")) {
            char buffer[256];
            comm_cmdbuf_next_arg(&fromPhone);
            comm_cmdbuf_this_arg_is(&fromPhone, "*");
            if (fromPhone.buffer[fromPhone.args] == '\0') {
                storage_get_gps_params_as_string(buffer, sizeof(buffer));
                DEBUG_PRINTF("%s %s\n", buffer, storage_get_gps_params_as_string_help());
            } else {
                storage_set_gps_params_as_string((char *)&fromPhone.buffer[fromPhone.args]);
                storage_save();
                storage_get_gps_params_as_string(buffer, sizeof(buffer));
                DEBUG_PRINTF("Now %s\n", buffer);
            }
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }

        // Restart
        if (comm_cmdbuf_this_arg_is(&fromPhone, "reboot") || comm_cmdbuf_this_arg_is(&fromPhone, "restart")) {
            io_request_restart();
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }

        // Echo locally and on server, just for connectivity testing
        if (comm_cmdbuf_this_arg_is(&fromPhone, "echo") || comm_cmdbuf_this_arg_is(&fromPhone, "hi") || comm_cmdbuf_this_arg_is(&fromPhone, "hello")) {
            comm_cmdbuf_next_arg(&fromPhone);
            if (fromPhone.buffer[fromPhone.args] == '\0')
                DEBUG_PRINTF("@device: Hello.\n");
            else
                DEBUG_PRINTF("@device: %s\n", &fromPhone.buffer[fromPhone.args]);
            // Transmit command, including slash, to the server, to continue echoing
            fromPhone.args = 0;
            fromPhone.state = CMD_STATE_XMIT_PHONE_TEXT;
            break;
        }

        // Unknown commands, including slash, are passed through to TTSERVE
        fromPhone.args = 0;
        fromPhone.state = CMD_STATE_XMIT_PHONE_TEXT;
        break;

    }

}

// One-time init
void phone_init() {
    comm_cmdbuf_init(&fromPhone, CMDBUF_TYPE_PHONE);
    comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
}

// Process byte received from phone
void phone_received_byte(uint8_t databyte) {
    comm_cmdbuf_received_byte(&fromPhone, databyte);
}

// Primary state processing of the command buffer
void phone_process() {

    // If it's not complete, just exit.
    if (!fromPhone.complete)
        return;

    // If we're idle, set the state based on buffer contents
    if (fromPhone.state == COMM_STATE_IDLE)
        phone_complete();

    switch (fromPhone.state) {

    case  CMD_STATE_XMIT_PHONE_TEXT: {
        uint16_t status;
        uint8_t buffer[CMD_MAX_LINELENGTH];

        // Allocate space on the stack to store the message data.
        teletype_Telecast message = teletype_Telecast_init_zero;

        /* Create a stream that will write to our buffer. */
        pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

        /* Build the message */
        message.DeviceType = teletype_Telecast_deviceType_TTAPP;

        message.has_Message = true;
        strncpy(message.Message, (char *) &fromPhone.buffer[0], fromPhone.length);

        message.has_DeviceIDNumber = true;
        message.DeviceIDNumber = io_get_device_address();

        // encode it and transmit it to TTSERVE
        status = pb_encode(&stream, teletype_Telecast_fields, &message);
        if (!status)
            DEBUG_PRINTF("pb_encode: %s\n", PB_GET_ERROR(&stream));
        else
            send_to_service("text", buffer, stream.bytes_written, REPLY_TTSERVE);

        comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
        break;
    }

    } // switch

} // phone_process()
