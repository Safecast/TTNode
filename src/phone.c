// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

// Phone "management UI via Bluetooth" state machine processing

#include <stdio.h>
#include <stdlib.h>
#include "debug.h"
#include "bt.h"
#include "config.h"
#include "boards.h"
#include "comm.h"
#include "lora.h"
#include "fona.h"
#include "send.h"
#include "stats.h"
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
#include "ugps.h"
#include "storage.h"
#include "bme0.h"
#include "ina.h"
#include "nrf_delay.h"
#include "tt.pb.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "ssd.h"

// Device states
#define CMD_STATE_XMIT_PHONE_TEXT       COMM_STATE_DEVICE_START+0

// Command buffer
static cmdbuf_t fromPhone;

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
            lora_send((char *)&fromPhone.buffer[0]);
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }
#endif // LORA

#ifdef FONA
        // Commands to be passed-through to the chip - used for testing
        if (comm_cmdbuf_this_arg_is(&fromPhone, "at+*")) {
            // Send to the chip, even if it may fail because of the state we're in
            fona_send((char *)&fromPhone.buffer[0]);
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }
#endif

        // Turn the display on or off
#ifdef SSD
        if (comm_cmdbuf_this_arg_is(&fromPhone, "ssd")) {
            comm_cmdbuf_next_arg(&fromPhone);
            if (comm_cmdbuf_this_arg_is(&fromPhone,"on")) {
                ssd1306_init();
            } else if (comm_cmdbuf_this_arg_is(&fromPhone,"off")) {
                ssd1306_term();
            } else {
                if (!ssd1306_active())
                    DEBUG_PRINTF("ssd <on/off>\n");
                else 
                    ssd1306_reset_display();
            }
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }
#endif

        // Battery level request
        if (comm_cmdbuf_this_arg_is(&fromPhone, "bat")) {
            if (sensor_group_schedule_now("g-basics")) {
                DEBUG_PRINTF("Starting g-basics with max debugging enabled.\n");
                debug_flags_set(DBG_SENSOR|DBG_SENSOR_MAX);
            }
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }

        // CPM measurement request
#ifdef GEIGERX
        if (comm_cmdbuf_this_arg_is(&fromPhone, "rad") || comm_cmdbuf_this_arg_is(&fromPhone, "cpm") || comm_cmdbuf_this_arg_is(&fromPhone, "geiger")) {
            if (sensor_group_schedule_now("g-geiger"))
                DEBUG_PRINTF("Starting g-geiger\n");
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }
#endif

        // Toggle a mode that makes it appear that no sensors are enabled
        if (comm_cmdbuf_this_arg_is(&fromPhone, "dead")) {
            uint16_t op_mode = sensor_op_mode();
            if (op_mode != OPMODE_TEST_DEAD)
                op_mode = OPMODE_TEST_DEAD;
            else
                op_mode = OPMODE_NORMAL;
            sensor_set_op_mode(op_mode);
            DEBUG_PRINTF("Sensor scheduling now %s\n", op_mode == OPMODE_NORMAL ? "ON" : "OFF");
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }

        // Force sensor scheduling now
        if (comm_cmdbuf_this_arg_is(&fromPhone, "q") || comm_cmdbuf_this_arg_is(&fromPhone, "sample") || comm_cmdbuf_this_arg_is(&fromPhone, "measure")) {
            sensor_schedule_now();
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }

        // Flip the display
        if (comm_cmdbuf_this_arg_is(&fromPhone, "flip")) {
            storage()->flags ^= FLAG_FLIP;
            storage_save(true);
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
#ifdef SSD
            ssd1306_term();
            ssd1306_init();
#endif
            DEBUG_PRINTF("Display flipped.\n");
            break;
        }
        
        // Force cellular to test failover behavior
        if (comm_cmdbuf_this_arg_is(&fromPhone, "fail")) {
            comm_force_cell();
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }

        // Force cellular
#if defined(FONA)
        if (comm_cmdbuf_this_arg_is(&fromPhone, "fona")) {
            comm_request_mode_on_reselect(COMM_FONA);
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }
#endif

        // Force lora
#if defined(LORA)
        if (comm_cmdbuf_this_arg_is(&fromPhone, "lora")) {
            comm_request_mode_on_reselect(COMM_LORA);
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }
#endif

        // Cancel a pending WAN request
        if (comm_cmdbuf_this_arg_is(&fromPhone, "none")) {
            comm_request_mode_on_reselect(COMM_NONE);
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }

        // Comms Sensor State request
        if (comm_cmdbuf_this_arg_is(&fromPhone, "ccc")) {
            comm_show_state();
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }

        // Show Sensor State request
        if (comm_cmdbuf_this_arg_is(&fromPhone, "sss")) {
            sensor_show_state(true);
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }

        // Cause GPS to have a forced update, simulating motion
        if (comm_cmdbuf_this_arg_is(&fromPhone, "gupdate") || comm_cmdbuf_this_arg_is(&fromPhone, "grefresh")) {
            storage()->gps_latitude = storage()->gps_longitude = 0.0;
            comm_gps_update();
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }

        // GPS "set to fake data" request
        if (comm_cmdbuf_this_arg_is(&fromPhone, "gfake")) {
            char buffer[256];
            storage_set_gps_params_as_string("1/1/1");
            storage_save(true);
            storage_get_gps_params_as_string(buffer, sizeof(buffer));
            DEBUG_PRINTF("Now %s\n", buffer);
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }

        // GPS abort and go to "last known Good GPS"
        if (comm_cmdbuf_this_arg_is(&fromPhone, "glkg") || comm_cmdbuf_this_arg_is(&fromPhone, "gabort")) {
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
                    DEBUG_PRINTF("%.3f/%.3f/%.3f\n", lat, lon, alt);
                else if (status == GPS_LOCATION_PARTIAL)
                    DEBUG_PRINTF("%.3f/%.3f\n", lat, lon);
                else if (status == GPS_NO_LOCATION)
                    DEBUG_PRINTF("No location.\n");
                else if (status == GPS_LOCATION_ABORTED)
                    DEBUG_PRINTF("Location aborted.\n");
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
                gpio_power_set(pin, fOn);
            }
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }

        // MTU test request
        if (comm_cmdbuf_this_arg_is(&fromPhone, "mtu")) {
            comm_cmdbuf_next_arg(&fromPhone);
            if (fromPhone.buffer[fromPhone.args] != '\0') {
                send_mtu_test(atoi((char *)&fromPhone.buffer[fromPhone.args]));
                debug_flags_set(DBG_RX|DBG_TX);
            }
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }

        // Temperature/Humidity request
#if defined(TWIHIH6130) || defined(TWIBME0)
        if (comm_cmdbuf_this_arg_is(&fromPhone, "temp") || comm_cmdbuf_this_arg_is(&fromPhone, "env")) {
            float envTempC, envHumRH;
            float envPress = 0.0;
#ifdef TWIHIH6130
            s_hih6130_get_value(&envTempC, &envHumRH);
#endif
#ifdef TWIBME0
            s_bme280_0_get_value(&envTempC, &envHumRH, &envPress);
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
        if (comm_cmdbuf_this_arg_is(&fromPhone, "comms") || comm_cmdbuf_this_arg_is(&fromPhone, "comm")) {
            stats_status_check(true);
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }

        // Request statistics
        if (comm_cmdbuf_this_arg_is(&fromPhone, "stats")) {
            comm_initiate_service_update(false);
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }

        // Request statistics
        if (comm_cmdbuf_this_arg_is(&fromPhone, "buff")) {
            comm_would_be_buffered(true);
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }

        // Request full statistics
        if (comm_cmdbuf_this_arg_is(&fromPhone, "hello")) {
            comm_initiate_service_update(true);
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

        // Force us to drop gps
        if (comm_cmdbuf_this_arg_is(&fromPhone, "nogps")) {
#ifdef TWIUBLOXM8
            s_gps_shutdown();
#endif
#ifdef FONAGPS
            fona_gps_shutdown();
#endif
#ifdef UGPS
            s_ugps_shutdown();
#endif
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }

        // Request to enable verbose LPWAN rx/tx logging
        if (comm_cmdbuf_this_arg_is(&fromPhone, "d")) {
            char flags[40];
            strcpy(flags, "");
            if (debug(DBG_RX) && debug(DBG_TX))
                strcat(flags, "C ");
            else if (!debug(DBG_RX) && !debug(DBG_TX))
                strcat(flags, "c ");
            else {
                strcat(flags, debug(DBG_RX) ? "RX " : "rx ");
                strcat(flags, debug(DBG_TX) ? "TX " : "tx ");
            }
            strcat(flags, debug(DBG_COMM_MAX) ? "CX " : "cx ");
            strcat(flags, debug(DBG_SENSOR) ? "S " : "s ");
            strcat(flags, debug(DBG_SENSOR_MAX) ? "SX " : "sx ");
            strcat(flags, debug(DBG_SENSOR_SUPERMAX) ? "SXX " : "sxx ");
            strcat(flags, debug(DBG_SENSOR_SUPERDUPERMAX) ? "SXXX " : "sxxx ");
            strcat(flags, debug(DBG_SENSOR_POLL) ? "SP " : "sp ");
            strcat(flags, debug(DBG_GPS_MAX) ? "GX " : "gx ");
            strcat(flags, debug(DBG_AIR) ? "A " : "a ");
            strcat(flags, debug(DBG_BT) ? "B " : "b ");
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
        if (comm_cmdbuf_this_arg_is(&fromPhone, "1")) {
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
        if (comm_cmdbuf_this_arg_is(&fromPhone, "rx")) {
            DEBUG_PRINTF("RX toggled to %s\n", debug_flag_toggle(DBG_RX) ? "ON" : "OFF");
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }
        if (comm_cmdbuf_this_arg_is(&fromPhone, "tx")) {
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
            DEBUG_PRINTF("COMMMAX toggled to %s\n", debug_flag_toggle(DBG_COMM_MAX) ? "ON" : "OFF");
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }
        if (comm_cmdbuf_this_arg_is(&fromPhone, "a")) {
            DEBUG_PRINTF("AIR toggled to %s\n", debug_flag_toggle(DBG_AIR) ? "ON" : "OFF");
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }
        if (comm_cmdbuf_this_arg_is(&fromPhone, "b")) {
            DEBUG_PRINTF("BT debug toggled to %s\n", debug_flag_toggle(DBG_BT) ? "ON" : "OFF");
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
        if (comm_cmdbuf_this_arg_is(&fromPhone, "sxxx")) {
            DEBUG_PRINTF("SENSORSUPERDUPERMAX toggled to %s\n", debug_flag_toggle(DBG_SENSOR_SUPERDUPERMAX) ? "ON" : "OFF");
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }
        if (comm_cmdbuf_this_arg_is(&fromPhone, "sp")) {
            DEBUG_PRINTF("SENSORPOLL toggled to %s\n", debug_flag_toggle(DBG_SENSOR_POLL) ? "ON" : "OFF");
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

        // Set test mode
        if (comm_cmdbuf_this_arg_is(&fromPhone, "test") || comm_cmdbuf_this_arg_is(&fromPhone, "t")) {
            // Abort GPS if we're still waiting, as a convenience
            if (!comm_gps_completed())
                storage_set_gps_params_as_string("1/1/1");
            comm_cmdbuf_next_arg(&fromPhone);
            comm_cmdbuf_this_arg_is(&fromPhone, "*");
            if (fromPhone.buffer[fromPhone.args] == '\0') {
                DEBUG_PRINTF("test <on/off> (now %s), or test <sensor-name>\n", sensor_op_mode() == OPMODE_TEST_FAST ? "ON" : "OFF");
            } else {
                if (comm_cmdbuf_this_arg_is(&fromPhone,"on")) {
                    sensor_test("");
                    debug_flags_set(DBG_SENSOR|DBG_SENSOR_MAX);
                    sensor_set_op_mode(OPMODE_TEST_FAST);
                    DEBUG_PRINTF("Rapid-cycling test mode ON\n");
                } else if (comm_cmdbuf_this_arg_is(&fromPhone,"off")) {
                    debug_flags_set(DBG_SENSOR_MAX|DBG_GPS_MAX);
                    debug_flag_toggle(DBG_SENSOR_MAX);
                    debug_flag_toggle(DBG_GPS_MAX);
                    sensor_test("");
                    DEBUG_PRINTF("Sensor test mode now disabled\n");
                } else {
                    debug_flags_set(DBG_SENSOR|DBG_SENSOR_MAX|DBG_GPS_MAX);
                    sensor_test((char *)&fromPhone.buffer[fromPhone.args]);
                }
            }
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }

        // Set power debug mode
        if (comm_cmdbuf_this_arg_is(&fromPhone, "pwr")) {
            comm_cmdbuf_next_arg(&fromPhone);
            comm_cmdbuf_this_arg_is(&fromPhone, "*");
            if (fromPhone.buffer[fromPhone.args] == '\0') {
                gpio_power_debug_mode(false, false);
                DEBUG_PRINTF("Power debug mode turned OFF\n");
            } else {
                if (comm_cmdbuf_this_arg_is(&fromPhone,"off")) {
                    gpio_power_debug_mode(false, true);
                    DEBUG_PRINTF("Power debug mode turned OFF - power will never be enabled\n");
                } else if (comm_cmdbuf_this_arg_is(&fromPhone,"on")) {
                    gpio_power_debug_mode(true, false);
                    DEBUG_PRINTF("Power debug mode turned on - power will never be disabled\n");
                }
            }
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }

        // Set mobile mode
        if (comm_cmdbuf_this_arg_is(&fromPhone, "mobile")) {
            comm_cmdbuf_next_arg(&fromPhone);
            comm_cmdbuf_this_arg_is(&fromPhone, "*");
            if (fromPhone.buffer[fromPhone.args] == '\0') {
                DEBUG_PRINTF("mobile <on/off> (now %s), or mobile <sample-period-secs>\n", sensor_op_mode() == OPMODE_MOBILE ? "ON" : "OFF");
            } else {
                if (comm_cmdbuf_this_arg_is(&fromPhone,"on")) {
                    sensor_set_op_mode(OPMODE_MOBILE);
                    DEBUG_PRINTF("mobile now ON\n");
                } else if (comm_cmdbuf_this_arg_is(&fromPhone,"off")) {
                    sensor_set_op_mode(OPMODE_NORMAL);
                    DEBUG_PRINTF("mobile now OFF\n");
                } else {
                    uint16_t num = atoi((char *)&fromPhone.buffer[fromPhone.args]);
                    sensor_set_mobile_upload_period(num);
                }

            }
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }

        // Set burn mode
        if (comm_cmdbuf_this_arg_is(&fromPhone, "burn")) {
            comm_cmdbuf_next_arg(&fromPhone);
            comm_cmdbuf_this_arg_is(&fromPhone, "*");
            if (fromPhone.buffer[fromPhone.args] == '\0') {
                DEBUG_PRINTF("burn <on/off> (currently %s)\n", sensor_op_mode() == OPMODE_TEST_BURN ? "ON" : "OFF");
            } else {
                if (comm_cmdbuf_this_arg_is(&fromPhone,"on")) {
                    sensor_set_op_mode(OPMODE_TEST_BURN);
                    DEBUG_PRINTF("burn now ON\n");
                }
                if (comm_cmdbuf_this_arg_is(&fromPhone,"off")) {
                    sensor_set_op_mode(OPMODE_NORMAL);
                    DEBUG_PRINTF("burn now OFF\n");
                }
            }
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }

        // Set blink mode, so we can identify a specific device
        if (comm_cmdbuf_this_arg_is(&fromPhone, "blink") || comm_cmdbuf_this_arg_is(&fromPhone, "id")) {
            gpio_indicate(INDICATE_BLINKY);
            DEBUG_PRINTF("%lu LEDs are now flashing quickly.\n", io_get_device_address());
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

        // Force an upload now, for debugging
        if (comm_cmdbuf_this_arg_is(&fromPhone, "upload")) {
            comm_call_now();
            DEBUG_PRINTF("Upload will be initiated ASAP\n");
        }

        // Shortcut to set to cell-only for testing
        if (comm_cmdbuf_this_arg_is(&fromPhone, "ct")) {
            char buffer[256];
            storage_set_device_params_as_string("cell");
            storage_save(true);
            storage_get_device_params_as_string(buffer, sizeof(buffer));
            DEBUG_PRINTF("Now %s\n", buffer);
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }

        // Shortcut to set device to be a test device
        if (comm_cmdbuf_this_arg_is(&fromPhone, "dt")) {
            STORAGE *f = storage();
            f->flags ^= FLAG_TEST;
            storage_save(true);
            DEBUG_PRINTF("Test Device flag toggled to %s\n", (f->flags & FLAG_TEST) != 0 ? "ON" : "OFF");
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }

        // Get/Set Device Parameters
        if (comm_cmdbuf_this_arg_is(&fromPhone, "cfgdev") || comm_cmdbuf_this_arg_is(&fromPhone, "o") || comm_cmdbuf_this_arg_is(&fromPhone, "op")) {
            char buffer[256];
            comm_cmdbuf_next_arg(&fromPhone);
            comm_cmdbuf_this_arg_is(&fromPhone, "*");
            if (fromPhone.buffer[fromPhone.args] == '\0') {
                storage_get_device_params_as_string(buffer, sizeof(buffer));
                DEBUG_PRINTF("%s %s\n", buffer, storage_get_device_params_as_string_help());
                DEBUG_PRINTF("wan: AUTO=%d LORA=%d TTN=%d FONA=%d F+M=%d\n", WAN_AUTO, WAN_LORA, WAN_LORAWAN, WAN_FONA, WAN_FONA_PLUS_MOBILE);
            } else {
                storage_set_device_params_as_string((char *)&fromPhone.buffer[fromPhone.args]);
                storage_save(true);
                storage_get_device_params_as_string(buffer, sizeof(buffer));
                DEBUG_PRINTF("Now %s\n", buffer);
            }
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }

        // Toggle BT configuration flag
        if (comm_cmdbuf_this_arg_is(&fromPhone, "bt")) {
            STORAGE *f = storage();
            f->flags ^= FLAG_BTKEEPALIVE;
            storage_save(true);
            DEBUG_PRINTF("BT Keepalive toggled to %s\n", (f->flags & FLAG_BTKEEPALIVE) != 0 ? "ON" : "OFF");
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }

        // Toggle CONFIRM ALL configuration flag (generally used as a "stats test")
        if (comm_cmdbuf_this_arg_is(&fromPhone, "cnf") || comm_cmdbuf_this_arg_is(&fromPhone, "st")) {
            STORAGE *f = storage();
            f->flags ^= FLAG_CONFIRM_ALL;
            storage_save(true);
            DEBUG_PRINTF("CONFIRM ALL toggled to %s\n", (f->flags & FLAG_CONFIRM_ALL) != 0 ? "ON" : "OFF");
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }

        // Get/Set Service Parameters
        if (comm_cmdbuf_this_arg_is(&fromPhone, "cfgsvc")) {
            char buffer[256];
            comm_cmdbuf_next_arg(&fromPhone);
            comm_cmdbuf_this_arg_is(&fromPhone, "*");
            if (fromPhone.buffer[fromPhone.args] == '\0') {
                storage_get_service_params_as_string(buffer, sizeof(buffer));
                DEBUG_PRINTF("%s %s\n", buffer, storage_get_service_params_as_string_help());
            } else {
                storage_set_service_params_as_string((char *)&fromPhone.buffer[fromPhone.args]);
                storage_save(true);
                storage_get_service_params_as_string(buffer, sizeof(buffer));
                DEBUG_PRINTF("Now %s\n", buffer);
            }
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }

        // Get/Set DFU Parameters
        if (comm_cmdbuf_this_arg_is(&fromPhone, "cfgdfu")) {
            char buffer[256];
            comm_cmdbuf_next_arg(&fromPhone);
            comm_cmdbuf_this_arg_is(&fromPhone, "*");
            if (fromPhone.buffer[fromPhone.args] == '\0') {
                storage_get_dfu_state_as_string(buffer, sizeof(buffer));
                DEBUG_PRINTF("%s %s\n", buffer, storage_get_dfu_state_as_string_help());
            } else {
                storage_set_dfu_state_as_string((char *)&fromPhone.buffer[fromPhone.args]);
                storage_save(true);
                storage_get_dfu_state_as_string(buffer, sizeof(buffer));
                DEBUG_PRINTF("Now %s\n", buffer);
            }
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }

        // Get/Set TTN Parameters
        if (comm_cmdbuf_this_arg_is(&fromPhone, "cfgttn")) {
            char buffer[256];
            comm_cmdbuf_next_arg(&fromPhone);
            comm_cmdbuf_this_arg_is(&fromPhone, "*");
            if (fromPhone.buffer[fromPhone.args] == '\0') {
                storage_get_ttn_params_as_string(buffer, sizeof(buffer));
                DEBUG_PRINTF("%s %s\n", buffer, storage_get_ttn_params_as_string_help());
            } else {
                storage_set_ttn_params_as_string((char *)&fromPhone.buffer[fromPhone.args]);
                storage_save(true);
                storage_get_ttn_params_as_string(buffer, sizeof(buffer));
                DEBUG_PRINTF("Now %s\n", buffer);
            }
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }

        // Get/Set device label
        if (comm_cmdbuf_this_arg_is(&fromPhone, "cfglab") || comm_cmdbuf_this_arg_is(&fromPhone, "l")) {
            char buffer[256];
            comm_cmdbuf_next_arg(&fromPhone);
            if (fromPhone.buffer[fromPhone.args] == '\0') {
                storage_get_device_label_as_string(buffer, sizeof(buffer));
                DEBUG_PRINTF("%s %s\n", buffer, storage_get_device_label_as_string_help());
            } else {
                storage_set_device_label_as_string((char *)&fromPhone.buffer[fromPhone.args]);
                storage_save(true);
                storage_get_device_label_as_string(buffer, sizeof(buffer));
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
                storage_save(true);
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
                storage_save(true);
                storage_get_gps_params_as_string(buffer, sizeof(buffer));
                DEBUG_PRINTF("Now %s\n", buffer);
            }
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }

        // TWI status
#ifdef TWIX
        if (comm_cmdbuf_this_arg_is(&fromPhone, "twi")) {
            twi_status_check(true);
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }
#endif

        // TWI status
        if (comm_cmdbuf_this_arg_is(&fromPhone, "mtu")) {
            mtu_status_check(true);
            comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
            break;
        }

        // Get time of day
        if (comm_cmdbuf_this_arg_is(&fromPhone, "time")) {
            DEBUG_PRINTF("%s\n", time_since_boot());
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
        ttproto_Telecast message = ttproto_Telecast_init_zero;

        /* Create a stream that will write to our buffer. */
        pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

        /* Build the message */
        message.has_device_type = true;
        message.device_type = ttproto_Telecast_deviceType_TTAPP;

        message.has_message = true;
        strlcpy(message.message, (char *) &fromPhone.buffer[0], fromPhone.length);

        message.has_device_id = true;
        message.device_id = io_get_device_address();

        // encode it and transmit it to TTSERVE
        status = pb_encode(&stream, ttproto_Telecast_fields, &message);
        if (!status)
            DEBUG_PRINTF("pb_encode: %s\n", PB_GET_ERROR(&stream));
        else
            send_to_service(buffer, stream.bytes_written, REPLY_TTSERVE, SEND_1);

        comm_cmdbuf_set_state(&fromPhone, COMM_STATE_IDLE);
        break;
    }

    } // switch

} // phone_process()
