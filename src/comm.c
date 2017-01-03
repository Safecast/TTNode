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
#include "bgeigie.h"
#include "phone.h"
#include "misc.h"
#include "send.h"
#include "timer.h"
#include "gpio.h"
#include "pms.h"
#include "opc.h"
#include "io.h"
#include "serial.h"
#include "sensor.h"
#include "bme.h"
#include "twi.h"
#include "storage.h"
#include "nrf_delay.h"
#include "teletype.pb.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "app_scheduler.h"

// Initialization-related
static bool commWaitingForFirstSelect = false;
static bool commInitialized = false;
static bool commEverInitialized = false;
static bool commForceCell = false;
static uint16_t active_comm_mode = COMM_NONE;
static uint16_t currently_deselected = false;

// Last Known Good GPS info
static bool overrideLocationWithLastKnownGood = false;

// Service suppression info
static uint32_t lastServicePingTime = 0L;
static bool oneshotCompleted = false;
static uint32_t lastOneshotTime = 0L;
static uint32_t oneshotPoweredUp = 0L;
static bool oneshotDisabled = false;

// Suppression
static uint32_t lastServiceUpdateTime = 0L;

// Initialize a command buffer
void comm_cmdbuf_init(cmdbuf_t *cmd, uint16_t type) {
    cmd->type = type;
    cmd->busy_length = 0;
    cmd->busy_nextput = 0;
    cmd->busy_nextget = 0;
    comm_cmdbuf_reset(cmd);
}

// Receive a byte into the cmdbuf
bool comm_cmdbuf_received_byte(cmdbuf_t *cmd, uint8_t databyte) {
    if (comm_cmdbuf_append(cmd, databyte)) {
        comm_enqueue_complete(cmd->type);
        return true;
    }
    return false;
}

// Reset a command buffer
void comm_cmdbuf_reset(cmdbuf_t *cmd) {
    uint8_t databyte;

    // Reset the buffer
    cmd->length = 0;
    cmd->buffer[0] = '\0';
    cmd->args = 0;
    cmd->complete = false;

    // If there was anything waiting in the busy buffer, process it.
    while (cmd->busy_length) {
        cmd->busy_length--;
        databyte = cmd->busy_buffer[cmd->busy_nextget++];
        if (cmd->busy_nextget >= sizeof(cmd->busy_buffer))
            cmd->busy_nextget = 0;
        if (comm_cmdbuf_received_byte(cmd, databyte))
            break;
    }

}

// Set command buffer state
void comm_cmdbuf_set_state(cmdbuf_t *cmd, uint16_t newstate) {

    // Do watchdog processing on the LPWAN receive buffer.
    // Remember the last time we changed the state of the
    // LPWAN, and set the watchdog.  If we fail to change
    // state it means that we're stuck for some reason.
    // For example, we've observed that sometimes a
    // communications error will cause a failure between
    // the tx1 and tx2 state, in which case we will
    // never proceed.  Since there is ALWAYS activity
    // that should be occurring on this port, i.e.
    // timeouts, we just use a sledgehammer if the
    // watchdog expires.
    if (cmd->state != newstate) {
#ifdef LORA
        if (cmd->type == CMDBUF_TYPE_LORA)
            lora_watchdog_reset();
#endif
#ifdef FONA
        if (cmd->type == CMDBUF_TYPE_FONA)
            fona_watchdog_reset();
#endif
    }

    // Reset the buffer and set its new state
    comm_cmdbuf_reset(cmd);
    cmd->state = newstate;

    // For multi-reply commands, reset what we've received
    cmd->recognized = 0;

}

// Append a byte to a command buffer
bool comm_cmdbuf_append(cmdbuf_t *cmd, uint8_t databyte) {

    // Exit if not even initialized yet
    if (!commInitialized)
        return false;

    // If we're already complete, we need to buffer it.
    if (cmd->complete) {
        if (cmd->busy_length < sizeof(cmd->busy_buffer)) {
            cmd->busy_buffer[cmd->busy_nextput++] = databyte;
            if (cmd->busy_nextput >= sizeof(cmd->busy_buffer))
                cmd->busy_nextput = 0;
            cmd->busy_length++;
            return false;
        } else {
            DEBUG_PRINTF("Busy buffer overflow!\n");
            return false;
        }
    }

    // if it's a newline, we're done
    if (databyte == '\n') {
        if (cmd->length != 0) {
            cmd->complete = true;
            return(true);
        }
        return(false);
    }

    // Only append it if it's simple ASCII
    if (databyte >= 0x20 && databyte < 0x7f) {
        cmd->buffer[cmd->length++] = databyte;

        // Always leave the buffer null-terminated, so we can use string operations during parsing
        cmd->buffer[cmd->length] = '\0';

        // If we've overflowed the buffer, we're done.
        if (cmd->length >= CMD_MAX_LINELENGTH) {
            cmd->complete = true;
            return(true);
        }

    }

    // Not yet complete
    return(false);

}

// Test to see if a character is a separator for the purpose of args
bool isArgSeparator(uint8_t databyte, bool embeddedSpaces) {

    // Obvious separators
    if (!embeddedSpaces && databyte == ' ')
        return true;
    if (databyte == ',' || databyte == ';')
        return true;

    // Treat trash as a separator
    if (databyte < 0x20 || databyte >= 0x7f)
        return (true);

    // Not a separator
    return (false);

}

// Test to see if the current command argument matches a specific lowercase string.
// The behavior is normally to look for a full delimited word, but there are two special modes:
//  1) a "xxx*" wildcard mode that matches anything beginning with that string
//  2) a "*" 'token mode' that matches any whole word up to the next delimeter, null-terminating it
bool comm_cmdbuf_this_arg_is(cmdbuf_t *cmd, char *testCmd) {
    bool testForWord, tokenMode, embeddedSpaces;
    int i, testLen;

    // First, see if there are embedded spaces in testCmd.
    testLen = strlen(testCmd);
    embeddedSpaces = false;
    for (i=0; i<testLen; i++) {
        if (testCmd[i] == ' ') {
            embeddedSpaces = true;
            break;
        }
    }

    // See if we need to test for a space afterward
    testForWord = true;
    tokenMode = false;
    if (testCmd[testLen - 1] == '*') {
        testLen--;
        // Does the asterisk mean 'wildcard mode', or 'token mode'?
        if (testLen != 0)
            testForWord = false;
        else
            tokenMode = true;
    }

    // This method always leaves 'nextarg' pointing at the next thing to parse, or end-of-buffer
    cmd->nextarg = cmd->args;

    // If looking for a testcmd match, start by scanning for testcmd in a case-insensitive matter
    if (!tokenMode) {

        // Not enough room left in the command buffer.
        if (testLen > (cmd->length - cmd->args)) {
            return (false);
        }

        // Iterate doing the comparison, with case folding.
        // (It is assumed that the testCmd is lower-case.)
        for (i = 0; i < testLen; i++) {
            char testChar = (char) cmd->buffer[cmd->args + i];
            if (testChar >= 'A' && testChar <= 'Z')
                testChar += 'a' - 'A';
            if (testCmd[i] != testChar) {
                return (false);
            }
        }

        // We've now at least matched the full pattern, and so
        // regardless of ultimate match conclusion, we've we should point at what's next
        cmd->nextarg += testLen;

        // If we tested the full string, we're done.
        if (testLen == (cmd->length - cmd->args)) {
            return (true);
        }

    }

    // In our special token mode, match ANY non-separator up to the next space or end-of-buffer
    if (tokenMode)
        while (cmd->nextarg < cmd->length && !isArgSeparator(cmd->buffer[cmd->nextarg], embeddedSpaces))
            cmd->nextarg++;

    // If we need to test for a word delimiter and there is more left, look at it
    if (testForWord && (cmd->nextarg < cmd->length)) {

        // Ensure there is at least one separator following the command
        if (!isArgSeparator(cmd->buffer[cmd->nextarg], embeddedSpaces)) {
            return (false);
        }

        // Skip past contiguous separators
        for (i = cmd->nextarg; i < cmd->length; i++) {
            if (!isArgSeparator(cmd->buffer[i], embeddedSpaces))
                break;
            if (tokenMode)
                cmd->buffer[i] = '\0';
        }
        cmd->nextarg = i;

    }

    // We've got us a match
    return (true);

}

// Based on having done comm_cmdbuf_this_arg_is(), move to the next argument
char *comm_cmdbuf_next_arg(cmdbuf_t *cmd) {
    // This method returns the pointer to the current arg,
    // while bumping the pointer to the next one.
    char *thisarg = (char *) &cmd->buffer[cmd->args];
    cmd->args = cmd->nextarg;
    return (thisarg);
}

// Reset all watchdogs
void comm_watchdog_reset() {
    switch (comm_mode()) {
#ifdef LORA
    case COMM_LORA:
        lora_watchdog_reset();
        break;
#endif
#ifdef FONA
    case COMM_FONA:
        fona_watchdog_reset();
        break;
#endif
    }
}

// Incoming data processing
void comm_reset(bool fForce) {
    switch (comm_mode()) {
#ifdef LORA
    case COMM_LORA:
        lora_reset(fForce);
        break;
#endif
#ifdef FONA
    case COMM_FONA:
        fona_reset(fForce);
        break;
#endif
    }
}

// Request state, for debugging
void comm_request_state() {
    switch (comm_mode()) {
#ifdef LORA
    case COMM_LORA:
        lora_request_state();
        break;
#endif
#ifdef FONA
    case COMM_FONA:
        fona_request_state();
        break;
#endif
    }

}

// Mark one-shot transmission as having been completed
void comm_oneshot_completed() {

    // Give pending transmits a chance to flush
    nrf_delay_ms(MAX_NRF_DELAY_MS);

    // Mark as completed
    oneshotCompleted = true;
    if (debug(DBG_COMM_MAX))
        DEBUG_PRINTF("Marking oneshot completed\n");
}

// Force entry of oneshot mode
void comm_force_cell() {

    // If we have something to fail over to, do it
#ifdef CELLX
    commForceCell = true;
    DEBUG_PRINTF("*** Network down - forcing cellular comms ***\n");
#endif

}

// For auto operating mode, see what phase we're in
uint16_t comm_autowan_mode() {

    // Do we have GPS yet?
    if (comm_gps_get_value(NULL, NULL, NULL) != GPS_LOCATION_FULL)
        return AUTOWAN_GPS_WAIT;

    // Check if in auto mode at all
    if (storage()->wan != WAN_AUTO)
        return AUTOWAN_NORMAL;

    // If we haven't failed over, we're still normal
    if (!commForceCell)
        return AUTOWAN_NORMAL;

    // We're in failover mode
    return AUTOWAN_FAILOVER;

}

// See if we should  process oneshots
bool comm_oneshot_currently_enabled() {

    // If we're in fona mode and we don't yet have the GPS, stay in continuous mode
#ifdef FONA
    if (comm_mode() == COMM_FONA && comm_gps_get_value(NULL, NULL, NULL) != GPS_LOCATION_FULL)
        return false;
#endif

    // If we're in fona mode and DFU is pending, stay in continuous mode
    if (storage()->dfu_status == DFU_PENDING)
        return false;
    
    // Determine whether enabled or not based on whether uart switching is allowed
    return (comm_uart_switching_allowed());

}

// Set a flag
void comm_disable_oneshot_mode() {
    oneshotDisabled = true;
}

// See if the UART is configured for switching
bool comm_uart_switching_allowed() {

    // If it's manually disabled AND we haven't failed over to cell, where we NEED oneshots
    if (oneshotDisabled && comm_autowan_mode() != AUTOWAN_FAILOVER)
        return false;

    // If the config value is nonzero, it means that we will constantly be using the UART
    // to communicate with one of our comms modules.  In that case, it isn't available
    // for UART-based sensors because it can't be switched away.
    if (storage()->oneshot_minutes == 0)
        return false;

    // We're configured for oneshots to be allowed
    return true;

}

// Primary comms-related poller, called from our app timer
uint32_t comm_get_oneshot_interval() {
    uint32_t suppressionSeconds;

    // Depending upon battery level, optionally slow down the one-shot uploader.
    // Note that if we fail to upload, this will naturally stop the sensors from
    // re-sampling - and thus it will slow down the entire oneshot process as a
    // desperate way of keeping the battery level in a reasonable state
    switch (sensor_get_battery_status()) {
        // If battery is dead, only one daily update
    case BAT_DEAD:
        suppressionSeconds = 24 * 60 * 60;
        break;
        // Only update hourly if in danger
    case BAT_EMERGENCY:
        suppressionSeconds = 1 * 60 * 60;
        break;
        // Update 30m if just a warning
    case BAT_WARNING:
        suppressionSeconds = 30 * 60;
        break;
        // Full battery
    case BAT_FULL:
        suppressionSeconds = ONESHOT_FAST_MINUTES * 60;
        break;
        // Normal
    default:
        suppressionSeconds  = storage()->oneshot_minutes * 60;
        break;
    }

    return(suppressionSeconds);

}

// Display current comm state
void comm_show_state() {
    if (!comm_oneshot_currently_enabled())
        DEBUG_PRINTF("Oneshot disabled\n");
    else {
        if (!currently_deselected) {
            DEBUG_PRINTF("Oneshot(%d) currently selected\n", comm_mode());
        } else {
            bool fOverdue = false;
            int nextsecs = comm_get_oneshot_interval() - (get_seconds_since_boot()-lastOneshotTime);
            if (nextsecs < 0) {
                nextsecs = -nextsecs;
                fOverdue = true;
            }
            int nextmin = nextsecs/60;
            nextsecs -= nextmin*60;
            char buff[128];
            if (fOverdue)
                sprintf(buff, "is overdue by %dm%ds", nextmin, nextsecs);
            else
                sprintf(buff, "will begin in %dm%ds", nextmin, nextsecs);
            DEBUG_PRINTF("Oneshot(%d) currently deselected\n", comm_mode());
            DEBUG_PRINTF("  uart %s, svc %s, svc %s, power %s, %s uploads\n",
                         gpio_current_uart() == UART_NONE ? "avail" : "busy",
                         comm_can_send_to_service() ? "avail" : "unavail",
                         comm_is_busy() ? "busy" : "not busy",
                         sensor_group_any_exclusive_powered_on() ? "in-use" : "avail",
                         sensor_any_upload_needed() ? "pending" : "no");
            DEBUG_PRINTF("  Next reselect %s\n", buff);
        }
    }

}

// Primary comms-related poller, called from our app timer
void comm_poll() {

    // Exit if the basic comms package has never yet been initialized.
    if (!commEverInitialized)
        return;

    // If we're waiting for our first select, process it.
    if (commWaitingForFirstSelect) {
        uint16_t wan;
        
        // Exit if we're still too early
        if (get_seconds_since_boot() < BOOT_DELAY_UNTIL_INIT)
            return;

        // Override WAN if doing DFU
        wan = storage()->wan;
#ifdef FONA
        if (storage()->dfu_status == DFU_PENDING) {
            wan = WAN_FONA;
            DEBUG_PRINTF("DFU %s\n", storage()->dfu_filename);
        }
#endif
            
        // Select as appropriate
        switch (wan) {

            // We use this when debugging sensors
        case WAN_NONE:
            comm_select(COMM_NONE);
            break;

            // If we're explicitly doing any form of LORA,
            // we go to COMM_LORA unless we also have
            // a FONA configured to get the GPS.
#ifdef LORA
        case WAN_LORA_THEN_LORAWAN:
        case WAN_LORAWAN_THEN_LORA:
        case WAN_LORA:
        case WAN_LORAWAN:
#ifdef FONA
            // Start with Fona to get the GPS
            // (that is, unless we already have it because of GPS being statically configured
            if (comm_gps_get_value(NULL, NULL, NULL) != GPS_LOCATION_FULL)
                comm_select(COMM_FONA);
            else
                comm_select(COMM_LORA);
#else
            comm_select(COMM_LORA);
#endif
            break;
#endif

            // Explicitly wanting FONA
#ifdef FONA
        case WAN_FONA:
            comm_select(COMM_FONA);
            break;
#endif

            // Auto starting with LORA, but we must start with cellular first to get the GPS
            // (that is, unless we already have it because of GPS being statically configured
        case WAN_AUTO:
#ifdef FONA
            if (comm_gps_get_value(NULL, NULL, NULL) != GPS_LOCATION_FULL)
                comm_select(COMM_FONA);
            else {
#ifdef LORA
                comm_select(COMM_LORA);
#else
                comm_select(COMM_NONE);
#endif
            }
#else
#ifdef LORA
            comm_select(COMM_LORA);
#else
            comm_select(COMM_NONE);
#endif
#endif
            break;
        }

        // Done witih initial select
        commWaitingForFirstSelect = false;
        return;

    }

    // Handle failover mode
#if defined(CELLX)
    static bool restartAfterFailover = false;
    static uint32_t failoverTime;

    // If we've entered failover mode but we're not yet in cell mode, perform the switch
    if (comm_autowan_mode() == AUTOWAN_FAILOVER && comm_mode() != COMM_FONA) {
        failoverTime = get_seconds_since_boot();
        restartAfterFailover = true;
        comm_select(COMM_FONA);
        return;
    }

    // If we've performed the switch, periodically restart the device in case the network has returned
    if (restartAfterFailover && !ShouldSuppress(&failoverTime, FAILOVER_RESTART_MINUTES * 60L)) {
        io_request_restart();
        return;
    }

#endif

    // If we're in oneshot mode, see if it's time to turn off or wake up the hardware.
    if (comm_oneshot_currently_enabled()) {

        // If comms is active
        if (!currently_deselected) {

            // If we're hung in init, presumably waiting for service, abort after a while
            // because aborting is preferable to hanging here forever and draining the battery.
            if (!comm_can_send_to_service() && oneshotPoweredUp != 0) {
                if (!ShouldSuppress(&oneshotPoweredUp, ONESHOT_ABORT_SECONDS)) {
                    if (comm_deselect())
                        if (debug(DBG_COMM_MAX))
                            DEBUG_PRINTF("Deselecting comms (oneshot aborted)\n");
                    return;
                }
                if (debug(DBG_COMM_MAX))
                    DEBUG_PRINTF("Oneshot waiting for comms init...\n");
                return;
            }

            // If the transaction completed, try again until there's nothing else to transmit
            if (oneshotCompleted && !comm_is_busy()) {
                oneshotCompleted = false;
                if (!comm_oneshot_service_update()) {
                    if (comm_deselect())
                        if (debug(DBG_COMM_MAX))
                            DEBUG_PRINTF("Deselecting comms (no work)\n");
                }
                return;
            }

            // If we're at our maximum for staying powered up, kill it in case we're wedged
            // Note that we only do this if we're in a "can send to service" state, because
            // we don't want to kill the power while it's still searching for carrier.
            // Also, note that if a service update is due, we process it before shutting down.
            if (comm_can_send_to_service()
                && !comm_is_busy()
                && !ShouldSuppress(&oneshotPoweredUp, ONESHOT_UPDATE_SECONDS)) {
                if (!comm_oneshot_service_update()) {
                    if (comm_deselect())
                        if (debug(DBG_COMM_MAX))
                            DEBUG_PRINTF("Deselecting comms (oneshot)\n");
                }
                return;
            }

        }

        // If it's time to power-up, do so, but only if the UART isn't already in use by someone else,
        // and if we're not measuring something that is sucking power, and if there are some pending
        // measurements waiting to go out.
        if (currently_deselected
            && (gpio_current_uart() == UART_NONE)
            && !comm_can_send_to_service()
            && !sensor_group_any_exclusive_powered_on()
            && sensor_any_upload_needed()) {

            // Check to see if it's time to reselect
            uint32_t suppressionSeconds = comm_get_oneshot_interval();
            if (suppressionSeconds != 0 && !ShouldSuppressConsistently(&lastOneshotTime, suppressionSeconds)) {
                stats_add(0, 0, 0, 0, 1, 0);
                if (debug(DBG_COMM_MAX))
                    DEBUG_PRINTF("Reselecting comms\n");
                oneshotPoweredUp = get_seconds_since_boot();
                comm_reselect();
                // The reselect() will start the fona_init() et al, and
                // the actual com_oneshot_service_update will
                // occur on the NEXT poll interval.
            }
        }
    }

    // Exit if we needed to reset the network
    if (!comm_is_deselected()) {

        switch (comm_mode()) {
#ifdef LORA
        case COMM_LORA:
            if (lora_needed_to_be_reset()) {
                if (debug(DBG_COMM_MAX))
                    DEBUG_PRINTF("LORA needed to be reset\n");
                return;
            }
            break;
#endif
#ifdef FONA
        case COMM_FONA:
            if (fona_needed_to_be_reset()) {
                if (debug(DBG_COMM_MAX))
                    DEBUG_PRINTF("FONA needed to be reset\n");
                return;
            }
            break;
#endif
        case COMM_NONE:
            return;
        }
    }

    // Do TTN pings if we're configured to do so, and if it's time
    if ((storage()->flags & FLAG_PING) != 0)
        if (!ShouldSuppress(&lastServicePingTime, PING_SERVICE_SECONDS)) {
            DEBUG_PRINTF("Ping.\n");
            send_ping_to_service(REPLY_NONE);
            return;
        }

    // Send our periodic updates to the service
    comm_oneshot_service_update();

    // Update our uptime stats
    stats_update();

}

// Force a stats update on the next opportunity to talk with service
void comm_service_update() {
    if (comm_oneshot_currently_enabled())
        lastServiceUpdateTime = 0;
    else
        send_update_to_service(UPDATE_STATS);
}

// If it's time, do a single transaction with the service to keep it up-to-date
bool comm_oneshot_service_update() {

    // Because it happens so seldomoly, give priority to periodically sending our version # to the service,
    // and receiving service policy updates back (processed in receive processing)
    if (comm_can_send_to_service())
        if (!ShouldSuppress(&lastServiceUpdateTime, SERVICE_UPDATE_HOURS*60*60)) {
            static bool fSentConfigDEV, fSentConfigSVC, fSentConfigGPS, fSentConfigSEN, fSentDFU;
            static bool fSentVersion = false;
            static bool fSentCell1, fSentCell2;
            bool fSentStats = false;
            bool fSentSomething = false;
            // On first iteration, initialize statics based on whether strings are non-null
            if (!fSentVersion) {
                fSentConfigDEV = !storage_get_device_params_as_string(NULL, 0);
                fSentConfigSVC = !storage_get_service_params_as_string(NULL, 0);
                fSentConfigGPS = !storage_get_gps_params_as_string(NULL, 0);
                fSentConfigSEN = !storage_get_sensor_params_as_string(NULL, 0);
                fSentDFU = !storage_get_dfu_state_as_string(NULL, 0);
                fSentCell1 = fSentCell2 = true;
#ifdef FONA
                if (comm_mode() == COMM_FONA)
                    fSentCell1 = fSentCell2 = false;
#endif
            }
            // Send each one in sequence
            if (!fSentVersion)
                fSentSomething = fSentVersion = send_update_to_service(UPDATE_STATS_VERSION);
            else if (!fSentConfigDEV)
                fSentSomething = fSentConfigDEV = send_update_to_service(UPDATE_STATS_CONFIG_DEV);
            else if (!fSentConfigGPS)
                fSentSomething = fSentConfigGPS = send_update_to_service(UPDATE_STATS_CONFIG_GPS);
            else if (!fSentConfigSVC)
                fSentSomething = fSentConfigSVC = send_update_to_service(UPDATE_STATS_CONFIG_SVC);
            else if (!fSentConfigSEN)
                fSentSomething = fSentConfigSEN = send_update_to_service(UPDATE_STATS_CONFIG_SEN);
            else if (!fSentDFU)
                fSentSomething = fSentDFU = send_update_to_service(UPDATE_STATS_DFU);
            else if (!fSentCell1)
                fSentSomething = fSentCell1 = send_update_to_service(UPDATE_STATS_CELL1);
            else if (!fSentCell2)
                fSentSomething = fSentCell2 = send_update_to_service(UPDATE_STATS_CELL2);
            else
                fSentSomething = fSentStats = send_update_to_service(UPDATE_STATS);
            // Come back here immediately if the message couldn't make it out or we have stuff left to do
            if (!fSentVersion
                || !fSentConfigDEV
                || !fSentConfigGPS
                || !fSentConfigSVC
                || !fSentConfigSEN
                || !fSentDFU
                || !fSentCell1
                || !fSentCell2
                || !fSentStats)
                lastServiceUpdateTime = 0L;
            if (debug(DBG_COMM_MAX))
                DEBUG_PRINTF("Stats were %s\n", fSentSomething ? "sent" : "not sent");
            return fSentSomething;
        }

    // If we've got pending sensor readings, flush to the service
    return(send_update_to_service(UPDATE_NORMAL));

}

// Is the communications path to the service initialized?
bool comm_can_send_to_service() {

    // Exit if the physical hardware is disabled
    if (currently_deselected)
        return false;

    // If we don't have comms, then there's no harm in saying "yes" which helps us debug w/no comms
    if (comm_mode() == COMM_NONE)
        return true;

    // Exit if we're barely coming off of init.
    if (get_seconds_since_boot() < FAST_DEVICE_UPDATE_BEGIN)
        return false;

    // Let the individual transport decide
    switch (comm_mode()) {
#ifdef LORA
    case COMM_LORA:
        return(lora_can_send_to_service());
#endif
#ifdef FONA
    case COMM_FONA:
        return(fona_can_send_to_service());
#endif
    }

    return false;
}

// Is the communications path to the service busy, and thus transmitting is pointless?
bool comm_is_busy() {
    if (!comm_can_send_to_service())
        return true;
    switch (comm_mode()) {
#ifdef LORA
    case COMM_LORA:
        return(lora_is_busy());
#endif
#ifdef FONA
    case COMM_FONA:
        return(fona_is_busy());
#endif
    }
    return false;
}

// Force gps to be re-acquired next time we can.
// (This is currently only coded to work for Oneshot, where the GPS is
// re-acquired when fona initializes.  If we need it to work in conditions
// other than fona, this should be enhanced to do whatever is needed to
// trigger a GPS re-acquisition.
void comm_gps_update() {
#ifdef FONA
    fona_gps_update();
#endif
    stats_add(0, 0, 0, 0, 0, 1);
}

// Use last known good info if we can't get the real info
void comm_gps_abort() {
    if (!overrideLocationWithLastKnownGood) {
        STORAGE *f = storage();
        DEBUG_PRINTF("GPS using last known good: %f %f\n", f->lkg_gps_latitude, f->lkg_gps_longitude);
    }
    overrideLocationWithLastKnownGood = true;
}

// Get the gps value, knowing that there may be multiple ways to fetch them
uint16_t comm_gps_get_value(float *pLat, float *pLon, float *pAlt) {
    uint16_t result;
    float lat, lon, alt;

    // Initialize this sequence of tests as "not configured"
    result = GPS_NOT_CONFIGURED;

    // If they're statically configured, we've got them.
    STORAGE *s = storage();
    if (s->gps_latitude != 0.0 && s->gps_longitude != 0.0) {
        static bool fDisplayed = false;
        lat = s->gps_latitude;
        lon = s->gps_longitude;
        alt = s->gps_altitude;
        result = GPS_LOCATION_FULL;
        if (!fDisplayed) {
            fDisplayed = true;
            DEBUG_PRINTF("GPS: Using statically-configured location\n");
        }
    }

    // If we are configured to have the TWI GPS, give it second dibs
#ifdef TWIUBLOXM8
    if (result != GPS_LOCATION_FULL)
        result = s_gps_get_value(&lat, &lon, &alt);
#endif

    // If the Fona is picking up GPS location, give it a shot to improve it
#ifdef FONA
    if (result != GPS_LOCATION_FULL) {
        float lat_improved, lon_improved, alt_improved;
        uint16_t result_improved = fona_gps_get_value(&lat_improved, &lon_improved, &alt_improved);
        if (result == GPS_NOT_CONFIGURED || result_improved == GPS_LOCATION_FULL || result_improved == GPS_LOCATION_PARTIAL) {
            lat = lat_improved;
            lon = lon_improved;
            alt = alt_improved;
            result = result_improved;
        }
    }
#endif

    // If it's still not configured, see if we should use LKG values
    if (result != GPS_LOCATION_FULL && result != GPS_LOCATION_PARTIAL) {

        // If we've been at this for too long, just give up so that we
        // don't completely block the ability to boot the device
        if (get_seconds_since_boot() > (GPS_ABORT_MINUTES * 60L))
            comm_gps_abort();

        // Substitute the last known good info if appropriate
        if (overrideLocationWithLastKnownGood) {
            STORAGE *f = storage();
            lat = f->lkg_gps_latitude;
            lon = f->lkg_gps_longitude;
            alt = f->lkg_gps_altitude;
            result = GPS_LOCATION_FULL;
        }

    }

    // Return the values
    if (result == GPS_LOCATION_FULL || result == GPS_LOCATION_PARTIAL) {
        if (pLat != NULL)
            *pLat = lat;
        if (pLon != NULL)
            *pLon = lon;
        if (pAlt != NULL)
            *pAlt = alt;
    }

    // If we now have full location, tell BOTH that they can shut down
    if (result == GPS_LOCATION_FULL) {
#ifdef TWIUBLOXM8
        s_gps_shutdown();
#endif
#ifdef FONA
        fona_gps_shutdown();
#endif
        gpio_indicate(INDICATE_GPS_CONNECTED);
    }

    // Done
    return result;
}

// Decode a hex-encoded received message, then unmarshal and process what's inside
uint16_t comm_decode_received_message(char *msg, void *ttmessage, uint8_t *buffer, uint16_t buffer_length, uint16_t *bytesDecoded) {
    char *listen_tags;
    uint8_t bin[256];
    int length;
    char hiChar, loChar;
    uint8_t databyte;
    uint16_t status;
    char ch;
    int ktlen, mtlen;
    char *kptr, *kptr2, ktag[64];
    char *mptr, *mptr2, mtag[64];
    bool match;
    teletype_Telecast tmessage;
    teletype_Telecast *message = (teletype_Telecast *) ttmessage;

    // Skip leading whitespace and control characters, to get to the hex
    while (*msg != '\0' && *msg <= ' ')
        msg++;

    // Convert hex to binary
    for (length = 0; length < sizeof(bin); length++) {
        hiChar = *msg++;
        loChar = *msg++;
        if (!HexValue(hiChar, loChar, &databyte))
            break;
        bin[length] = databyte;
    }

    // Zero out the structure to receive the decoded data
    if (message == NULL)
        message = &tmessage;
    memset(message, 0, sizeof(teletype_Telecast));

    // Create a stream that will write to our buffer.
    pb_istream_t stream = pb_istream_from_buffer(bin, length);

    // Decode the message
    status = pb_decode(&stream, teletype_Telecast_fields, message);
    if (!status) {
        DEBUG_PRINTF("pb_decode: %s\n", PB_GET_ERROR(&stream));
        return MSG_NOT_DECODED;
    }

    // Return how many we've decoded
    if (bytesDecoded != NULL)
        *bytesDecoded = length;

    // Copy the message to the output buffer
    if (message->has_Message)
        strncpy((char *) buffer, message->Message, buffer_length);
    else
        buffer[0] = '\0';

    // We don't support or need listen tags in any mode except Lora
#ifdef LORA
    if (comm_mode() == COMM_LORA)
        listen_tags = lora_get_listen_tags();
    else
        listen_tags = "";
#else
    listen_tags = "";
#endif

    // Do various things based on device type
    switch (message->DeviceType) {
    case teletype_Telecast_deviceType_SIMPLECAST:
    case teletype_Telecast_deviceType_BGEIGIE_NANO:
        return MSG_SAFECAST;
    case teletype_Telecast_deviceType_TTGATE:
        // If it's from ttgate and directed at us, then it's a reply to our request
        if (message->has_DeviceIDNumber && message->DeviceIDNumber == io_get_device_address())
            return MSG_REPLY_TTGATE;
        return MSG_TELECAST;
    case teletype_Telecast_deviceType_TTSERVE:
        // If it's from ttserve and directed at us, then it's a reply to our request
        if (message->has_DeviceIDNumber && message->DeviceIDNumber == io_get_device_address())
            return MSG_REPLY_TTSERVE;
        return MSG_TELECAST;
    case teletype_Telecast_deviceType_TTAPP:
        // If we're in receive mode and we're filtering tags, break out and do that processing
        if (listen_tags[0] == '\0')
            return MSG_TELECAST;
        break;
    default:
        return MSG_TELECAST;
    }

    // This is a Telecast 'text message' from TTAPP, so
    // iterate over all tags to see if it's something we're listening for
    match = false;
    for (kptr = listen_tags; !match && *kptr != '\0'; kptr++) {

        // Scan past leading whitespace
        while (*kptr != '\0' && *kptr <= ' ')
            kptr++;

        // Exit if we've run out of things to check
        if (*kptr == '\0')
            break;

        // Do special processing if we're sitting on a hash tag
        if (*kptr == '#') {

            // Extract the tag, normalizing it to uppercase
            ktlen = 0;
            for (kptr2 = kptr; *kptr2 != '\0' && *kptr2 > ' '; kptr2++)
                if (ktlen < sizeof(ktag) - 1) {
                    ch = *kptr2;
                    if (ch >= 'a' && ch <= 'z')
                        ch -= 'a' - 'A';
                    ktag[ktlen++] = ch;
                }

            // Test the tag against all tags found in the message
            for (mptr = message->Message; !match && *mptr != '\0'; mptr++) {

                // Scan past leading whitespace
                while (*mptr != '\0' && *mptr <= ' ')
                    mptr++;

                // Exit if we've run out of things to check
                if (*mptr == '\0')
                    break;

                // Do special processing if we're sitting on a hash tag
                if (*mptr == '#') {

                    // Extract the tag, normalizing it to uppercase
                    mtlen = 0;
                    for (mptr2 = mptr; *mptr2 != '\0' && *mptr2 > ' '; mptr2++)
                        if (mtlen < sizeof(mtag) - 1) {
                            ch = *mptr2;
                            if (ch >= 'a' && ch <= 'z')
                                ch -= 'a' - 'A';
                            mtag[mtlen++] = ch;
                        }

                    // Test the tag for equality
                    if (ktlen == mtlen) {
                        int i;
                        for (i = 0; i < ktlen; i++)
                            if (ktag[i] != mtag[i])
                                break;
                        if (i == ktlen)
                            match = true;
                    }
                }

                // Find the end of this word or tag
                while (*mptr != '\0' && *mptr > ' ')
                    mptr++;

            }

        }

        // Find the end of this word or tag
        while (*kptr != '\0' && *kptr > ' ')
            kptr++;

    }

    // If there's a match, tell the caller to display it
    return (match ? MSG_TELECAST : MSG_NOT_DECODED);

}

// Turn off the power to all comms if any is selected
bool comm_deselect() {
    gpio_uart_select(UART_NONE);
    currently_deselected = true;
    oneshotCompleted = true;
    gpio_indicate(INDICATE_COMMS_STATE_UNKNOWN);
    return true;
}

// See if we are truly powered off
bool comm_is_deselected() {
    return(currently_deselected);
}

// Re-enable comms if it is disabled
void comm_reselect() {
    if (currently_deselected)
        comm_select(active_comm_mode);
    oneshotCompleted = false;
}

// Select a specific comms mode
void comm_select(uint16_t which) {
    if (which == COMM_NONE) {
        gpio_uart_select(UART_NONE);
    }
#ifdef LORA
    if (which == COMM_LORA) {
        gpio_uart_select(UART_LORA);
        lora_init();
    }
#endif
#ifdef FONA
    if (which == COMM_FONA) {
        gpio_uart_select(UART_FONA);
        fona_init();
    }
#endif

    // Now (and only now) that that we're initialized, allow things to proceed
    active_comm_mode = which;
    currently_deselected = false;

}

// Initialization of this module and the entire state machine
void comm_init() {

    // Init state machines
    phone_init();
#ifdef BGEIGIE
    bgeigie_init();
#endif
    comm_select(COMM_NONE);

    // Done
    commInitialized = true;
    commEverInitialized = true;
    commWaitingForFirstSelect = true;
}

// Is comms initialized?
bool comm_is_initialized() {
    return(commInitialized);
}

// Reinitialize if appropriate
void comm_reinit() {
    if (commEverInitialized)
        comm_init();
}

// What mode are we in?
uint16_t comm_mode() {
    if (!commEverInitialized)
        return COMM_NONE;
    return (active_comm_mode);
}

// Process a completion event
void completion_event_handler(void *p_event_data, uint16_t event_size) {
    uint16_t type = * (uint16_t *) p_event_data;

    switch (type) {
    case CMDBUF_TYPE_PHONE:
        phone_process();
        break;
#ifdef BGEIGIE
    case CMDBUF_TYPE_BGEIGIE:
        bgeigie_process();
        break;
#endif
#ifdef LORA
    case CMDBUF_TYPE_LORA:
        if (!currently_deselected)
            lora_process();
        break;
#endif
#ifdef FONA
    case CMDBUF_TYPE_FONA:
        if (!currently_deselected)
            fona_process();
        break;
    case CMDBUF_TYPE_FONA_DEFERRED:
        if (!currently_deselected)
            fona_process_deferred();
        break;
#endif
    }

}

// Enqueue, at an interrupt level, a completion event
void comm_enqueue_complete(uint16_t type) {
    uint32_t err_code;
    err_code = app_sched_event_put(&type, sizeof(type), completion_event_handler);
    if (err_code != NRF_SUCCESS)
        DEBUG_PRINTF("Can't put (%d): 0x%04x\n", err_code, err_code);
}
