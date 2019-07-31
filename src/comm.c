// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

// Communications abstraction for TTNode that, to a reasonable extent,
// largely "driverizes" access to specific communications mechanisms
// such as Lora and Fona.

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
#include "stats.h"
#include "timer.h"
#include "gpio.h"
#include "pms.h"
#include "opc.h"
#include "ugps.h"
#include "io.h"
#include "serial.h"
#include "sensor.h"
#include "twi.h"
#include "storage.h"
#include "tt.pb.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "app_scheduler.h"
#include "battery.h"

// Initialization-related
static bool commWaitingForFirstSelect = false;
static bool commInitialized = false;
static bool commEverInitialized = false;
static bool commForceCell = false;
static bool commCallNow = false;
static uint16_t active_comm_mode = COMM_NONE;
static uint16_t currently_deselected = false;
static bool fFlushBuffers = false;
static uint16_t mode_request = COMM_NONE;
static uint16_t connect_state = CONNECT_STATE_UNKNOWN;
static char last_select_reason[64] = "";

// Burn & stats stuff
static bool burn_toggle_mode_request = false;
#ifdef FASTBURN
static bool fSentFullStatsOnLora = true;
static bool fSentFullStatsOnFona = true;
#else
static bool fSentFullStatsOnLora = false;
static bool fSentFullStatsOnFona = false;
#endif
#if defined(BURN) && !defined(FASTBURN)
static bool fSentFullStats = false;
#else
static bool fSentFullStats = true;
#endif

// Last Known Good GPS info
static bool overrideLocationWithLastKnownGood = false;
static bool gpsEverAborted = false;

// Service suppression info
static uint32_t lastServicePingTime = 0L;
static bool oneshotCompleted = false;
static bool oneshotCompletedNextPoll = false;
static uint32_t lastOneshotTime = 0L;
static uint32_t comm_powered_up = 0L;
static uint32_t comm_last_powered_up = 0L;
static uint32_t comm_powered_down = 0L;
static bool oneshotDisabled = false;

// Suppression
static uint32_t lastServiceUpdateTime = 0L;

// App scheduler
static uint16_t pending_completions = 0;

// Timer for comm select, for stats purposes
#define COMM_SELECT_TRACK_TIMES 10
static uint16_t worstCommSelectTimes[COMM_SELECT_TRACK_TIMES];
static uint16_t absoluteWorst = 0;
static uint32_t lastCommSelectTimePurgeTime = 0L;
static uint32_t lastCommSelectTime = 0L;
static bool isCommSelectInProgress = false;
static uint32_t failedCommSelects = 0L;
static uint32_t totalCommSelects = 0L;

// Get name of battery status, for debugging
char *comm_mode_name(uint16_t mode) {
    switch (mode) {
    case COMM_NONE:
        return "NONE";
    case COMM_LORA:
        return "LORA";
    case COMM_FONA:
        return "FONA";
    }
    return "?";
}

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
            // Busy buffer overflow - just reset it and cause data loss
            // because there's really nothing else we can do.
            cmd->busy_length = 0;
            cmd->busy_nextput = 0;
            return false;
        }
    }

    // If it's a newline AND not a blank line AND within our line length limits, we're done
    if (databyte == '\n') {
        if (cmd->length != 0) {
            cmd->complete = true;
            return(true);
        }
        // Blank line
        return(false);
    }

    // Only append it if it's simple ASCII
    if (databyte >= 0x20 && databyte < 0x7f) {
        cmd->buffer[cmd->length++] = databyte;

        // If we've overflowed the buffer, it's an error, so let's take some intentional data loss
        // rather than enqueueing bad work to be done.  Very specifically, if we get flooded with
        // data for whatever reason that is both within the ASCII range and isn't terminated
        // by a newline, there's a possibility that we might either overflow the busy buffer
        // or enqueue scheduler put's far too fast to handle.  Just shunt this crap to ground.
        if (cmd->length >= CMD_MAX_LINELENGTH)
            cmd->length = 0;

        // Always leave the buffer null-terminated, so we can use string operations during parsing
        cmd->buffer[cmd->length] = '\0';

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

// Force a call now, for debugging
void comm_call_now() {
    commCallNow = true;
    lastOneshotTime = 0;
    comm_initiate_service_update(false);
}

// See if we can send stats with limited MTU available
bool comm_can_send_large_stats() {
    return (comm_get_mtu() > 256);
}

// Get MTU
uint16_t comm_get_mtu() {

    // Default MTU to be extremely high, because during init the comm_mode() is undefined
    // while we are trying to actually determine our comm mode!
    uint16_t MTU = 512;

    switch (comm_mode()) {
#ifdef LORA
    case COMM_LORA:
        MTU = lora_get_mtu();
        break;
#endif
#ifdef FONA
    case COMM_FONA:
        MTU = fona_get_mtu();
        break;
#endif
    }

    return MTU;
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

    // Mark as completed, giving it a bit of time to complete
    // the current transaction before actually powering down.
    oneshotCompletedNextPoll = true;

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
    if (!comm_gps_completed())
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

    // If we  don't yet have the GPS, stay in continuous mode so we can acquire it
    if (!comm_gps_completed())
        return false;

    // If we're in fona mode and DFU is pending, stay in continuous mode
    if (storage()->dfu_status == DFU_PENDING)
        return false;

    // Exit if MTU test is in progress
    if (send_mtu_test_in_progress())
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

// Get primary sampling interval, in seconds
uint32_t get_oneshot_interval() {
    uint32_t suppressionSeconds = storage()->oneshot_minutes * 60;

    // If we're debugging and want to force a call, do it now
    if (commCallNow)
        return 1;

    // Depending upon battery level, optionally slow down the one-shot uploader.
    // Note that if we fail to upload, this will naturally stop the sensors from
    // re-sampling - and thus it will slow down the entire oneshot process as a
    // desperate way of keeping the battery level in a reasonable state
    switch (battery_status()) {
        // If battery is dead, only one daily update
    case BAT_DEAD:
        suppressionSeconds = 24 * 60 * 60;
        break;
        // Only on a long interval if in danger
    case BAT_EMERGENCY:
        suppressionSeconds = 6 * 60 * 60;
        break;
        // Update 30m if just a warning
    case BAT_WARNING:
        suppressionSeconds = 30 * 60;
        break;
        // Full battery
    case BAT_FULL:
        if (suppressionSeconds > (ONESHOT_FAST_MINUTES * 60))
            suppressionSeconds = (ONESHOT_FAST_MINUTES * 60);
        break;
    case BAT_BURN:
    case BAT_TEST:
        suppressionSeconds = 5 * 60;
        break;
        // Wicked-fast mobile mode
    case BAT_MOBILE: {
        uint16_t period_secs = sensor_get_mobile_upload_period();
        if (period_secs == 0)
            suppressionSeconds = 1;
        else
            suppressionSeconds = period_secs;
        break;
    }
    }

    return(suppressionSeconds);

}

// Get the cellular call interval, in seconds
uint32_t get_oneshot_cell_interval() {

    // If we're debugging and want to force a call, do it now
    if (commCallNow)
        return 1;
    
    // If in test mode, force it every 10m
    if (sensor_op_mode() == OPMODE_TEST_FAST || sensor_op_mode() == OPMODE_TEST_BURN)
        return (10 * 60);

    // Return what's configured
    return(storage()->oneshot_cell_minutes * 60);

}

// Get service update interval, with debugging support
uint32_t get_service_update_interval_minutes() {

    // If we're trying to force a call, stop the buffering
    if (commCallNow)
        return 1;

    // If we're in burn mode, force upload of stats periodically
    if (sensor_op_mode() == OPMODE_TEST_BURN) {
#ifdef FASTBURN
        return (5);
#else
        return (15);
#endif
    }

    // Use the setting in NVRAM
    return (storage()->stats_minutes);

}

// Display current comm state
void comm_show_state() {
    uint32_t seconds_since_boot = get_seconds_since_boot();
    if (!comm_oneshot_currently_enabled())
        DEBUG_PRINTF("Oneshot disabled\n");
    else {
        if (!currently_deselected) {
            DEBUG_PRINTF("Oneshot(%s) selected (%s)\n", comm_mode_name(comm_mode()), last_select_reason);
        } else {

            // Display oneshot time
            bool fOverdue = false;
            int nextsecs = get_oneshot_interval() - (seconds_since_boot-lastOneshotTime);
            if (nextsecs < 0) {
                nextsecs = -nextsecs;
                fOverdue = true;
            }
            int nextmin = nextsecs/60;
            nextsecs -= nextmin*60;
            char buff1[128];
            if (fOverdue)
                sprintf(buff1, "(%ldm) is overdue by %dm%ds", get_oneshot_interval()/60, nextmin, nextsecs);
            else
                sprintf(buff1, "(%ldm) will begin in %dm%ds", get_oneshot_interval()/60, nextmin, nextsecs);

            // Display cell oneshot time
            fOverdue = false;
            nextsecs = get_oneshot_cell_interval() - (seconds_since_boot - lastOneshotTime);
            if (nextsecs < 0) {
                nextsecs = -nextsecs;
                fOverdue = true;
            }
            nextmin = nextsecs/60;
            nextsecs -= nextmin*60;
            char buff2[128];
            if (fOverdue)
                sprintf(buff2, "(%dm) is overdue by %dm%ds", (int)get_oneshot_cell_interval()/60, nextmin, nextsecs);
            else
                sprintf(buff2, "(%dm) will begin in %dm%ds", (int)get_oneshot_cell_interval()/60, nextmin, nextsecs);

            // Display state
            DEBUG_PRINTF("Oneshot(%s) currently deselected (%s)\n", comm_mode_name(comm_mode()), last_select_reason);
            DEBUG_PRINTF("  Next oneshot %s\n", buff1);
            if (comm_would_be_buffered(false))
                DEBUG_PRINTF("  Next cell upload: %s\n", buff2);
        }
        DEBUG_PRINTF("  uart %s, svc %s, svc %s, %s, %s uploads\n",
                     gpio_current_uart() == UART_NONE ? "avail" : "busy",
                     comm_can_send_to_service() ? "avail" : "unavail",
                     comm_is_busy() ? "busy" : "idle",
                     comm_would_be_buffered(false) ? "buff" : "nobuff",
                     sensor_any_upload_needed() ? "pending" : "no");
        // Display stats time
        bool fOverdue = false;
        int nextsecs = (get_service_update_interval_minutes()*60) - (seconds_since_boot-lastServiceUpdateTime);
        if (nextsecs < 0) {
            nextsecs = -nextsecs;
            fOverdue = true;
        }
        int nextmin = nextsecs/60;
        nextsecs -= nextmin*60;
        DEBUG_PRINTF("Next stats update (%ldm) %s by %dm%ds\n", get_service_update_interval_minutes(), fOverdue ? "is overdue" : "will begin", nextmin, nextsecs);

    }

}

void select_lora_if_available() {
#ifdef LORA
    comm_select(COMM_LORA, "lora desired");
#else
    comm_select(COMM_NONE, "lora desired but not configured");
#endif
}

// Debug function to request a switch on the next reselect
void comm_request_mode_on_reselect(uint16_t mode) {
    mode_request = mode;
    DEBUG_PRINTF("Comm mode %s requested\n", comm_mode_name(mode));
}

// Primary comms-related poller, called from our app timer
void comm_poll() {

    // Exit if the basic comms package has never yet been initialized.
    if (!commEverInitialized)
        return;

    // If a Oneshot completion was requested on the next poll, do it.
    if (oneshotCompletedNextPoll) {
        oneshotCompletedNextPoll = false;
        oneshotCompleted = true;
    }

    // Display failures periodically, as a debugging tool
    mtu_status_check(false);

    // If we're waiting for our first select, process it.
    if (commWaitingForFirstSelect) {
        uint16_t wan;

        // Exit if we're fetching GPS
        if (gpio_current_uart() != UART_NONE) {
            if (debug(DBG_COMM_MAX))
                DEBUG_PRINTF("Comms init waiting for UART\n");
            return;
        }

        if (debug(DBG_COMM_MAX))
            DEBUG_PRINTF("Initializing communications.\n");

        // Exit if we're still too early
        if (get_seconds_since_boot() < BOOT_DELAY_UNTIL_INIT)
            return;

        // Get WAN requested in nvram
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
            comm_select(COMM_NONE, "no comms found");
            break;

            // If we're explicitly doing any form of LORA,
            // we go to COMM_LORA unless we also have
            // a FONA configured to get the GPS.
        case WAN_LORA_THEN_LORAWAN:
        case WAN_LORAWAN_THEN_LORA:
        case WAN_LORA:
        case WAN_LORAWAN:
#if defined(FONAGPS)
            if (!comm_gps_completed())
                comm_select(COMM_FONA, "lora desired, no GPS yet");
            else
                comm_select(COMM_LORA, "lora desired");
#elif defined(UGPS)
            if (!comm_gps_completed())
                comm_select(COMM_NONE, "lora desired, no GPS yet");
            else
                select_lora_if_available();
#else
            select_lora_if_available();
#endif
            break;

            // Explicitly wanting FONA
#ifdef FONA
        case WAN_FONA:
        case WAN_FONA_PLUS_MOBILE:
#if defined(UGPS)
            if (!comm_gps_completed())
                comm_select(COMM_NONE, "fona desired, no GPS yet");
            else
                comm_select(COMM_FONA, "fona desired");
#else
            comm_select(COMM_FONA, "fona desired");
#endif
            break;
#endif

            // Auto starting with LORA, but we must get the GPS first
        case WAN_AUTO:
#if defined(FONAGPS)
            if (!comm_gps_completed())
                comm_select(COMM_FONA, "auto desired, no GPS yet");
            else
                select_lora_if_available();
#elif defined(UGPS)
            if (!comm_gps_completed())
                comm_select(COMM_NONE, "auto desired, no GPS yet");
            else
                select_lora_if_available();
#else
            select_lora_if_available();
#endif
            break;
        }

        // Done witih initial select
        if (active_comm_mode != COMM_NONE)
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
        comm_select(COMM_FONA, "failover");
        return;
    }

    // If we've performed the switch, periodically restart the device in case the network has returned
    if (restartAfterFailover && !ShouldSuppress(&failoverTime, FAILOVER_RESTART_MINUTES * 60L)) {
        io_request_restart();
        return;
    }

#endif

    // Perform MTU testing if appropriate
    if (send_mtu_test_in_progress()) {
        comm_reselect();
        send_update_to_service(UPDATE_STATS_MTU_TEST);
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

    // If we're in oneshot mode, see if it's time to turn off or wake up the hardware.
    if (comm_oneshot_currently_enabled()) {

        // If comms is active
        if (!currently_deselected) {

            // If we're hung in init, presumably waiting for service, abort after a while
            // because aborting is preferable to hanging here forever and draining the battery.
            if (!comm_can_send_to_service() && comm_powered_up != 0) {
                if (!ShouldSuppress(&comm_powered_up, ONESHOT_ABORT_SECONDS)) {
                    comm_deselect("oneshot aborted");
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
                if (!comm_update_service()) {
                    comm_deselect("no work");
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
                && !ShouldSuppress(&comm_powered_up, ONESHOT_UPDATE_SECONDS)) {
                if (!comm_update_service()) {
                    comm_deselect("oneshot idle");
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
            && (gpio_current_uart() == UART_NONE || comm_would_be_buffered(false))
            && (!comm_can_send_to_service() || comm_would_be_buffered(false))
            && !sensor_group_any_exclusive_powered_on()
            && !sensor_group_any_exclusive_busy()
            && (sensor_any_upload_needed() || commCallNow)) {

            // Check to see if it's time to reselect
            uint32_t suppressionSeconds = get_oneshot_interval();
            if (suppressionSeconds != 0 && !ShouldSuppressConsistently(&lastOneshotTime, suppressionSeconds)) {
                stats()->oneshots++;

                // If the comms would be buffered, just do the buffered service update now - else reselect
                if (comm_would_be_buffered(false)) {

                    uint16_t updates = 0;
                    while (comm_update_service())
                        updates++;

                    if (updates > 1)
                        DEBUG_PRINTF("%d oneshots buffered\n", updates);

                } else {
                    // Now that we know we aren't being buffered, clear the
                    // flag that may have been used to flush buffers, so that
                    // buffering may once again occur
                    fFlushBuffers = false;

                    // The reselect() will start the fona_init() et al, and
                    // the actual comm_update_service will
                    // occur on the NEXT poll interval.
                    if (debug(DBG_COMM_MAX))
                        DEBUG_PRINTF("Reselecting comms\n");
                    comm_reselect();

                }
            }
        }
    }

    // Do TTN pings if we're configured to do so, and if it's time
    if ((storage()->flags & FLAG_PING) != 0)
        if (!ShouldSuppress(&lastServicePingTime, PING_SERVICE_SECONDS)) {
            DEBUG_PRINTF("Ping.\n");
            send_ping_to_service(REPLY_NONE);
            return;
        }

    // Send our periodic updates to the service, except if we're buffering
    // in which case we want better control over the timing
    if (!comm_would_be_buffered(false))
        comm_update_service();

    // Update our uptime stats
    stats_update();

}

// Force an update with nonbuffered I/O to flush the buffers
void comm_flush_buffers() {
    fFlushBuffers = true;
}

// Force a stats update on the next opportunity to talk with service
void comm_initiate_service_update(bool fFull) {
    if (fFull) {
        fSentFullStats = false;
        fSentFullStatsOnLora = false;
        fSentFullStatsOnFona = false;
    }
    lastServiceUpdateTime = 0;
    comm_flush_buffers();
}

// If it's time, do a single transaction with the service to keep it up-to-date
bool comm_update_service() {

    // Exit if we're currently busy transmitting something
    if (comm_is_busy())
        return false;

    // Before doing anything else, flush measurements that are pending in nvram
    if (!comm_is_deselected() && db_get(NULL, NULL, NULL) != 0) {
        uint8_t entry[DB_ENTRY_BYTES];
        uint16_t entry_length, entry_request_type;
        uint16_t messages = db_get(entry, &entry_length, &entry_request_type);
        if (messages != 0) {
            // Only attempt to send it if there's some possibility that we CAN.
            // This happens frequently because we may have buffered data while in
            // mobile mode, but then later we're on Lora which can't send out
            // the buffered messages.  They'll just need to wait until a Fona
            // connection is active.
            if (entry_length <= comm_get_mtu()) {
                DEBUG_PRINTF("SEND %db/%dm from flash (%d entries)\n", entry_length, messages, entry[1]);
                if (comm_send_to_service(entry, entry_length, entry_request_type)) {
                    db_get_release();
                    return true;
                }
                return false;
            }
        }
    }

    // Because it happens so seldomoly, give priority to periodically sending our version # to the service,
    // and receiving service policy updates back (processed in receive processing)
    if (!comm_would_be_buffered(false) && !ShouldSuppress(&lastServiceUpdateTime, get_service_update_interval_minutes()*60)) {
        static bool fSentConfigDEV = true;
        static bool fSentConfigSVC = true;
        static bool fSentConfigTTN = true;
        static bool fSentConfigLAB = true;
        static bool fSentConfigBAT = true;
        static bool fSentConfigMOD = true;
        static bool fSentConfigERR = true;
        static bool fSentConfigGPS = true;
        static bool fSentConfigSEN = true;
        static bool fSentDFU = true;
        static bool fSentCell1 = true;
        static bool fSentCell2 = true;
        bool fMobile = sensor_op_mode() == OPMODE_MOBILE;
        bool fSentStats = false;
        bool fSentSomething = false;
        // On first iteration, initialize statics based on whether strings are non-null
        if (!fSentFullStats) {
            fSentConfigLAB = !storage_get_device_label_as_string(NULL, 0);
            fSentConfigBAT = stats()->battery[0] == '\0';
            fSentConfigMOD = false;
            fSentConfigERR = false;
            fSentConfigDEV = !storage_get_device_params_as_string(NULL, 0);
            fSentConfigSVC = !storage_get_service_params_as_string(NULL, 0);
            fSentConfigTTN = storage()->ttn_dev_eui[0] == '\0';
            fSentConfigGPS = !storage_get_gps_params_as_string(NULL, 0);
            fSentConfigSEN = !storage_get_sensor_params_as_string(NULL, 0);
            fSentDFU = !storage_get_dfu_state_as_string(NULL, 0);
            fSentCell1 = fSentCell2 = true;
#ifdef FONA
            if (comm_mode() == COMM_FONA)
                fSentCell1 = fSentCell2 = false;
#endif
        }
        // Keep track of which one was active when we sent full stats
        if (!fSentFullStats) {
            if (comm_mode() == COMM_LORA) {
                fSentFullStatsOnLora = true;
                DEBUG_PRINTF("Now sending full stats on LORA\n");
            }
            if (comm_mode() == COMM_FONA) {
                fSentFullStatsOnFona = true;
                DEBUG_PRINTF("Now sending full stats on FONA\n");
            }
        }
        // Send each one in sequence
        if (!fSentFullStats)
            fSentSomething = fSentFullStats = fMobile || send_update_to_service(UPDATE_STATS_VERSION);
        else if (!fSentConfigLAB)
            fSentSomething = fSentConfigLAB = fMobile || send_update_to_service(UPDATE_STATS_LABEL);
        else if (!fSentConfigDEV)
            fSentSomething = fSentConfigDEV = fMobile || send_update_to_service(UPDATE_STATS_CONFIG_DEV);
        else if (!fSentConfigGPS)
            fSentSomething = fSentConfigGPS = fMobile || send_update_to_service(UPDATE_STATS_CONFIG_GPS);
        else if (!fSentConfigSVC)
            fSentSomething = fSentConfigSVC = fMobile || send_update_to_service(UPDATE_STATS_CONFIG_SVC);
        else if (!fSentConfigTTN)
            fSentSomething = fSentConfigTTN = fMobile || send_update_to_service(UPDATE_STATS_CONFIG_TTN);
        else if (!fSentConfigSEN)
            fSentSomething = fSentConfigSEN = fMobile || send_update_to_service(UPDATE_STATS_CONFIG_SEN);
        else if (!fSentConfigBAT)
            fSentSomething = fSentConfigBAT = fMobile || send_update_to_service(UPDATE_STATS_BATTERY);
        else if (!fSentConfigMOD)
            fSentSomething = fSentConfigMOD = fMobile || send_update_to_service(UPDATE_STATS_MODULES);
        else if (!fSentConfigERR)
            fSentSomething = fSentConfigERR = fMobile || send_update_to_service(UPDATE_STATS_ERRORS);
        else if (!fSentDFU)
            fSentSomething = fSentDFU = fMobile || send_update_to_service(UPDATE_STATS_DFU);
        else if (!fSentCell1)
            fSentSomething = fSentCell1 = fMobile || send_update_to_service(UPDATE_STATS_CELL1);
        else if (!fSentCell2)
            fSentSomething = fSentCell2 = fMobile || send_update_to_service(UPDATE_STATS_CELL2);
        else {
            fSentSomething = fSentStats = send_update_to_service(UPDATE_STATS);
        }
        // Come back here immediately if the message couldn't make it out or we have stuff left to do
        if (!fSentFullStats
            || !fSentConfigDEV
            || !fSentConfigGPS
            || !fSentConfigSVC
            || !fSentConfigTTN
            || !fSentConfigLAB
            || !fSentConfigBAT
            || !fSentConfigMOD
            || !fSentConfigERR
            || !fSentConfigSEN
            || !fSentDFU
            || !fSentCell1
            || !fSentCell2
            || !fSentStats) {
            lastServiceUpdateTime = 0L;
            // When we come back, let's make sure that we are NOT using buffered I/O
            comm_flush_buffers();
        } else {
            // We've completed sending stats.
            // If we're in burn mode, set up for the next iteration
            if (sensor_op_mode() == OPMODE_TEST_BURN) {
                // Toggle to the other of Fona or Lora mode on the next iteration
                burn_toggle_mode_request = true;
                // Make sure we've sent full stats at least once in each mode
#ifndef BURNFONA
                if (!fSentFullStatsOnLora) {
                    DEBUG_PRINTF("Requesting full stats for LORA\n");
                    fSentFullStats = false;
                }
#endif
#ifndef BURNLORA
                if (!fSentFullStatsOnFona) {
                    DEBUG_PRINTF("Requesting full stats for FONA\n");
                    fSentFullStats = false;
                }
#endif
                // In burn mode, always include errors when doing stats
                fSentConfigERR = false;
            }

        }
        if (debug(DBG_COMM_MAX))
            DEBUG_PRINTF("Stats were %s, updtime=%ld\n", fSentSomething ? "sent" : "not sent", lastServiceUpdateTime);
        return fSentSomething;
    }

    // If we've got pending sensor readings, flush to the service
    return(send_update_to_service(UPDATE_NORMAL));

}

// Would comms be buffered if we tried to send?
bool comm_would_be_buffered(bool fVerbose) {

    // If we're trying to force a call, stop the buffering
    if (commCallNow)
        return false;
    
    // If we're forcing nonbuffered, do it here.
#ifdef COMMS_FORCE_NONBUFFERED
    if (fVerbose)
        DEBUG_PRINTF("No: compiled\n");
    return false;
#endif

    // If forcing all transactions to the service, don't buffer
    if ((storage()->flags & FLAG_CONFIRM_ALL) != 0) {
        if (fVerbose)
            DEBUG_PRINTF("No: confirming all transactions\n");
        return false;
    }

    // If not doing oneshots, don't buffer
    if (get_oneshot_cell_interval() == 0) {
        if (fVerbose)
            DEBUG_PRINTF("No: not oneshot mode\n");
        return false;
    }

    // We will only buffer when we are deselected
    if (!currently_deselected) {
        if (fVerbose)
            DEBUG_PRINTF("No: not deselected\n");
        return false;
    }

    // If comms isn't cellular, it won't be buffered
    if (comm_mode() != COMM_FONA) {
        if (fVerbose)
            DEBUG_PRINTF("No: not fona\n");
        return false;
    }

    // During testing, turn off buffering
    if (sensor_op_mode() == OPMODE_TEST_FAST || sensor_op_mode() == OPMODE_TEST_BURN) {
        if (fVerbose)
            DEBUG_PRINTF("No: wrong operating mode\n");
        return false;
    }

    // If there's no room left, it won't be buffered.  Note that in making this calculation
    // we proactively assume that there will be another message, and we don't want to
    // buffer if we're this close to the end.
#define NEXT_MESSAGE_ALLOWANCE 150
    if (!db_enabled() && send_buff_is_full(NEXT_MESSAGE_ALLOWANCE)) {
        if (fVerbose)
            DEBUG_PRINTF("No: full buff\n");
        return false;
    }

    // If we don't have fine-granularity time, we can't do any buffering
    // because all the uploads will look like they're at the same date/time
    if (!get_current_timestamp(NULL, NULL, NULL)) {
        if (fVerbose)
            DEBUG_PRINTF("No: no timestamp\n");
        return false;
    }

    // If we're forcing a flush, do it
    if (fFlushBuffers && !send_buff_is_empty()) {
        if (fVerbose)
            DEBUG_PRINTF("No: need to flush buffers\n");
        return false;
    }

    // If it's time to do a transmit to the service, don't buffer it
    if (!WouldSuppress(&comm_last_powered_up, get_oneshot_cell_interval())) {
        if (fVerbose)
            DEBUG_PRINTF("No: time for a oneshot\n");
        return false;
    }

    // If it's time to do a stats request, don't buffer it
    if (!WouldSuppress(&lastServiceUpdateTime, get_service_update_interval_minutes()*60)) {
        if (fVerbose)
            DEBUG_PRINTF("No: time for stats\n");
        return false;
    }

    // Done
    if (fVerbose)
        DEBUG_PRINTF("Would be buffered!\n");

    return true;

}

// Send data to the service
bool comm_send_to_service(uint8_t *buffer, uint16_t length, uint16_t RequestType) {
    bool fTransmitted = false;
    switch (comm_mode()) {
#ifdef LORA
    case COMM_LORA:
        fTransmitted = lora_send_to_service(buffer, length, RequestType);
        break;
#endif
#ifdef FONA
    case COMM_FONA:
        fTransmitted = fona_send_to_service(buffer, length, RequestType);
        break;
#endif
    default:
        fTransmitted = false;
        break;
    }
    return fTransmitted;
}

// Is the communications path to the service initialized?
bool comm_can_send_to_service() {

    // Exit if the physical hardware is disabled
    if (currently_deselected)
        return comm_would_be_buffered(false);

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
    // Removed on 2017-05-24 because this defeats the purpose of the difference between
    // truly busy (comms in progress) and not able to send to service.  Was causing comms
    // hang on fona no service.  If this is still here by 2017-07, removing this worked well
    // and you should delete the "#if 0"'d block
#if 0
    if (!comm_can_send_to_service())
        return true;
#endif
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

// See if the GPS is currently active and updating
bool comm_gps_active() {
#ifdef UGPS
    return s_ugps_active();
#else
    return false;
#endif
}

// Force gps to be re-acquired next time we can.
// (This is currently only coded to work for Oneshot, where the GPS is
// re-acquired when fona initializes.  If we need it to work in conditions
// other than fona, this should be enhanced to do whatever is needed to
// trigger a GPS re-acquisition.
void comm_gps_update() {

    // If we are on an extremely-low MTU device ie LoRaWAN, don't
    // even support GPS update because we cannot risk seven the possibility
    // of sending unstamped messages for risk of MTU overflow.
    if (comm_get_mtu() < 64)
        return;
    
    // Exit if statically-configured GPS
    if (storage()->gps_latitude != 0.0 && storage()->gps_longitude != 0.0) {
        DEBUG_PRINTF("GPS will not be refreshed because it is statically defined\n");
        return;
    }

    DEBUG_PRINTF("GPS will be refreshed\n");

    // Update the GPS location
#ifdef FONAGPS
    fona_gps_update();
#endif
#ifdef UGPS
    s_ugps_update();
#endif

}

// Use last known good info if we can't get the real info
void comm_gps_abort() {

    // Remember that we have indeed aborted
    gpsEverAborted = true;

    // Request the override with LNG
    if (!overrideLocationWithLastKnownGood) {
        STORAGE *f = storage();
        DEBUG_PRINTF("GPS using last known good: %f %f\n", f->lkg_gps_latitude, f->lkg_gps_longitude);
        overrideLocationWithLastKnownGood = true;
    }

    // Shut down the GPS
#ifdef TWIUBLOXM8
    s_gps_shutdown();
#endif
#ifdef UGPS
    s_ugps_shutdown();
#endif
#ifdef FONAGPS
    fona_gps_shutdown();
#endif
    gpio_indicate(INDICATE_GPS_CONNECTED);

}

// Get the gps value, knowing that there may be multiple ways to fetch them
// On 2017-03-17 I changed this to allow PARTIAL because we shouldn't block updates/comms because
// of something so trivial as altitude.
bool comm_gps_completed() {
    uint16_t status = comm_gps_get_value(NULL, NULL, NULL);
    bool fCompleted = (status == GPS_LOCATION_FULL || status == GPS_LOCATION_PARTIAL || status == GPS_NOT_CONFIGURED || status == GPS_LOCATION_ABORTED);
    return fCompleted;
}

// Get the gps value, knowing that there may be multiple ways to fetch them
uint16_t comm_gps_get_value(float *pLat, float *pLon, float *pAlt) {
    uint16_t result;
    float lat, lon, alt;

    // Initialize this sequence of tests as "not configured"
    result = GPS_NOT_CONFIGURED;
    lat = lon = alt = 0.0;

    // If they're statically configured, we've got them.
    STORAGE *s = storage();
    if (s->gps_latitude != 0.0 && s->gps_longitude != 0.0) {
        static bool fDisplayed = false;
        lat = s->gps_latitude;
        lon = s->gps_longitude;
        alt = s->gps_altitude;
        result = GPS_LOCATION_FULL;
        overrideLocationWithLastKnownGood = false;
        if (!fDisplayed) {
            fDisplayed = true;
            DEBUG_PRINTF("GPS: Using statically-configured location\n");
        }
    }

    // If we are configured to have the TWI GPS, give it second dibs
#ifdef TWIUBLOXM8
    if (result != GPS_LOCATION_FULL) {
        result = s_gps_get_value(&lat, &lon, &alt);
        if (result == GPS_LOCATION_FULL || result == GPS_LOCATION_PARTIAL)
            overrideLocationWithLastKnownGood = false;
    }
#endif

    // If the Fona is picking up GPS location, give it a shot to improve it
#ifdef FONAGPS
    if (result != GPS_LOCATION_FULL) {
        float lat_improved, lon_improved, alt_improved;
        uint16_t result_improved = fona_gps_get_value(&lat_improved, &lon_improved, &alt_improved);
        if (result == GPS_NOT_CONFIGURED || result_improved == GPS_LOCATION_FULL || result_improved == GPS_LOCATION_PARTIAL) {
            lat = lat_improved;
            lon = lon_improved;
            alt = alt_improved;
            result = result_improved;
        }
        if (result == GPS_LOCATION_FULL || result == GPS_LOCATION_PARTIAL)
            overrideLocationWithLastKnownGood = false;
    }
#endif

    // See if we can improve the values yet again
#ifdef UGPS
    if (result != GPS_LOCATION_FULL) {
        float lat_improved, lon_improved, alt_improved;
        uint16_t result_improved = s_ugps_get_value(&lat_improved, &lon_improved, &alt_improved);
        if (result == GPS_NOT_CONFIGURED || result_improved == GPS_LOCATION_FULL || result_improved == GPS_LOCATION_PARTIAL) {
            lat = lat_improved;
            lon = lon_improved;
            alt = alt_improved;
            result = result_improved;
        }
        if (result == GPS_LOCATION_FULL || result == GPS_LOCATION_PARTIAL)
            overrideLocationWithLastKnownGood = false;
    }
#endif

    // If it's still not configured, see if we should use LKG values
    if (result != GPS_LOCATION_FULL && result != GPS_LOCATION_PARTIAL && result != GPS_NOT_CONFIGURED) {

        // If we've been at this for too long, just give up so that we
        // don't completely block the ability to boot the device.  Note that
        // we only do this for old devices, because starting with the UGPS
        // the GPS driver itself is responsible for aborting itself on timeout..
#if defined(FONAGPS) || defined(TWIUBLOXM8)
        if (get_seconds_since_boot() > (GPS_ABORT_FIRST_MINUTES * 60L))
            comm_gps_abort();
#endif

        // Substitute the last known good info if we had aborted
        if (overrideLocationWithLastKnownGood) {
            STORAGE *f = storage();
            if (f->lkg_gps_latitude != 0.0 && f->lkg_gps_longitude != 0.0) {
                lat = f->lkg_gps_latitude;
                lon = f->lkg_gps_longitude;
                alt = f->lkg_gps_altitude;
                result = GPS_LOCATION_FULL;
            } else {
                lat = 0;
                lon = 0;
                alt = 0;
            }

        }

        // If our location is 0 and we've aborted, or if we are actually trying to use 0/0 as
        // a true result, return that it has been aborted.  Otherwise, just go with the
        // result that we've got already (i.e. unconfigured, no data, no location, etc)
        if (lat == 0.0 && lon == 0.0) {
            if (gpsEverAborted || (result == GPS_LOCATION_FULL || result == GPS_LOCATION_PARTIAL))
                result = GPS_LOCATION_ABORTED;
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
    if (result == GPS_LOCATION_FULL || result == GPS_NOT_CONFIGURED) {
#ifdef TWIUBLOXM8
        s_gps_shutdown();
#endif
#ifdef UGPS
        s_ugps_shutdown();
#endif
#ifdef FONAGPS
        fona_gps_shutdown();
#endif
        gpio_indicate(INDICATE_GPS_CONNECTED);
    }

    // Done
    return result;
}

// Decode a hex-encoded received message, then unmarshal and process what's inside
uint16_t comm_decode_received_message(char *msg, void *ttmessage, uint8_t *buffer, uint16_t buffer_length, uint16_t *bytesDecoded) {
    uint8_t bin[256], *pbin;
    int length;
    char hiChar, loChar;
    uint8_t databyte;
    uint16_t status;
    ttproto_Telecast tmessage;
    ttproto_Telecast *message = (ttproto_Telecast *) ttmessage;

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

    // Return how many we've hex-decoded
    if (bytesDecoded != NULL)
        *bytesDecoded = length;

    // Zero out the structure to receive the decoded data
    if (message == NULL)
        message = &tmessage;
    memset(message, 0, sizeof(ttproto_Telecast));

    // Look at the first byte of what's been received, and see if it's in the new "array" format.
    // It will be this way if we're relaying a message.
    // If not, just process it as-is under the assumption that it's a single protocol buffer
    pbin = bin;
    if (bin[0] == BUFF_FORMAT_PB_ARRAY && bin[1] == 1) {

        // Process the message
        length = bin[2];
        pbin = &bin[3];

        DEBUG_PRINTF("Received %d-byte message\n", length);

    } else {

        DEBUG_PRINTF("Received message of unknown format 0x%02x 0x%02x 0x%02x\n", bin[0], bin[1], bin[2]);
        return MSG_NOT_DECODED;

    }

    // Create a stream that will write to our buffer.
    pb_istream_t stream = pb_istream_from_buffer(pbin, length);

    // Decode the message
    status = pb_decode(&stream, ttproto_Telecast_fields, message);
    if (!status) {
        DEBUG_PRINTF("pb_decode: %s\n", PB_GET_ERROR(&stream));
        return MSG_NOT_DECODED;
    }

    // Copy the message to the output buffer
    if (message->has_message)
        strlcpy((char *) buffer, message->message, buffer_length);
    else
        buffer[0] = '\0';

    // Do various things based on device type
    if (!message->has_device_type) {

        DEBUG_PRINTF("(ignoring message from %lu)\n", message->device_id);
        return MSG_SAFECAST;

    } else {

        switch (message->device_type) {
        case ttproto_Telecast_deviceType_UNKNOWN_DEVICE_TYPE:
        case ttproto_Telecast_deviceType_SOLARCAST:
        case ttproto_Telecast_deviceType_BGEIGIE_NANO:
        case ttproto_Telecast_deviceType_TTGATEPING:
            DEBUG_PRINTF("(ignoring message from %d)\n", message->device_id);
            return MSG_SAFECAST;

        case ttproto_Telecast_deviceType_TTGATE:
            // If it's from ttgate and directed at us, then it's a reply to our request
            if (message->has_device_id) {
                if (message->device_id == io_get_device_address())
                    DEBUG_PRINTF("Received TTGATE message\n");
                else
                    DEBUG_PRINTF("Received TTGATE message for device %lu\n", message->device_id);
            } else {
                DEBUG_PRINTF("Received an invalidly-formatted TTGATE message\n");
            }
            return MSG_REPLY_TTGATE;

        case ttproto_Telecast_deviceType_TTSERVE:
            // If it's from ttserve and directed at us, then it's a reply to our request
            if (message->has_device_id && message->device_id == io_get_device_address()) {
                DEBUG_PRINTF("Received TTSERVE message\n");
                return MSG_REPLY_TTSERVE;
            }
            DEBUG_PRINTF("Received TTServe message not intended for this device\n");
            return MSG_TELECAST;

        case ttproto_Telecast_deviceType_TTAPP:
            DEBUG_PRINTF("Received TTAPP message\n");
            return MSG_TELECAST;

        case ttproto_Telecast_deviceType_TTNODE:
            DEBUG_PRINTF("Received peer message\n");
            return MSG_TELECAST;
        }
    }

    DEBUG_PRINTF("Received unknown message\n");
    return MSG_NOT_DECODED;

}

// Set the state so that we can understand why connects may have failed
void comm_set_connect_state(uint16_t state) {
    connect_state = state;
}

// Get the connect state
char *comm_connect_state() {
    switch (connect_state) {
    case CONNECT_STATE_UNKNOWN:
        return "(not yet connected)";
    case CONNECT_STATE_LORA_MODULE:
        return "Starting Lora";
    case CONNECT_STATE_FONA_MODULE:
        return "Starting Cell";
    case CONNECT_STATE_WIRELESS_SERVICE:
        return "Waiting for cell service";
    case CONNECT_STATE_DATA_SERVICE:
        return "Waiting for cell data";
    case CONNECT_STATE_APP_SERVICE:
        return "Waiting for Safecast service";
    case CONNECT_STATE_LORA_GATEWAY:
        return "Looking for gateway";
    case CONNECT_STATE_LORAWAN_GATEWAY:
        return "Looking for TTN gateway";
    case CONNECT_STATE_LORA_DESELECTED:
        return "Lora idle";
    case CONNECT_STATE_FONA_DESELECTED:
        return "Fona idle";
    case CONNECT_STATE_LORA_ACTIVE:
        return "Lora active";
    case CONNECT_STATE_LORAWAN_ACTIVE:
        return "Lora TTN active";
    case CONNECT_STATE_FONA_ACTIVE:
        return "Cell active";
    }
    return "?";
}

// Temporarily deselect the active comms
void comm_deselect(char *reason) {
    uint16_t comm_mode = active_comm_mode;
    comm_select(COMM_NONE, reason);
    active_comm_mode = comm_mode;
}

// See if we are truly powered off
bool comm_is_deselected() {
    return(currently_deselected);
}

// See if we're in a state such that we can buffer to flash storage
bool comm_db_is_active() {
    return (comm_is_deselected() && db_enabled());
}

// See if comms should be overridden
uint16_t comm_mode_override(uint16_t new_mode) {
    uint16_t wan_mode = storage()->wan;
    uint16_t old_mode = new_mode;

    // Don't substitute if selecting NONE
    if (new_mode == COMM_NONE)
        return new_mode;

    // When in burn-in mode, toggle comms when told to do so
#if defined(BURNFONA)
    DEBUG_PRINTF("BURN using FONA\n");
    new_mode = COMM_FONA;
    burn_toggle_mode_request = false;
#elif defined(BURNLORA)
    DEBUG_PRINTF("BURN using LORA\n");
    new_mode = COMM_LORA;
    burn_toggle_mode_request = false;
#else
    if (burn_toggle_mode_request && wan_mode == WAN_AUTO) {
        if (new_mode == COMM_LORA) {
            DEBUG_PRINTF("Toggling to FONA\n");
            new_mode = COMM_FONA;
            burn_toggle_mode_request = false;
        } else if (new_mode == COMM_FONA) {
            DEBUG_PRINTF("Toggling to LORA\n");
            new_mode = COMM_LORA;
            burn_toggle_mode_request = false;
        }
    }
#endif

    // When in mobile mode and requesting Lora, toggle comms to Fona
    if (new_mode == COMM_LORA && sensor_op_mode() == OPMODE_MOBILE) {
        if (wan_mode == WAN_AUTO || wan_mode == WAN_FONA || wan_mode == WAN_FONA_PLUS_MOBILE) {
            DEBUG_PRINTF("Switching to FONA for mobile\n");
            new_mode = COMM_FONA;
        }
    }

    // If there is a specific request, process it
    if (mode_request != COMM_NONE) {
        new_mode = mode_request;
        mode_request = COMM_NONE;
    }

    // When we WERE in mobile mode, and we're leaving stuff in the buffers, we can't
    // go back to Lora mode until the buffers are flushed, because we can't flush them on Lora.
    if (new_mode == COMM_LORA && (!send_buff_is_empty() || db_get(NULL, NULL, NULL) != 0)) {
        DEBUG_PRINTF("Can't switch to LORA because huge buffers yet to be transmitted.\n");
        new_mode = COMM_FONA;
    }

    // Display if changed
    if (old_mode != new_mode)
        DEBUG_PRINTF("Comm mode switched from %s to %s\n", comm_mode_name(old_mode), comm_mode_name(new_mode));

    return new_mode;
}

// Re-enable comms if it is disabled
void comm_reselect() {

    if (currently_deselected) {

        // Select the new comms
        comm_select(active_comm_mode, "reselect");

    }

    // Remember whether or not the work to be performed by this
    // select has ever been completed successfully.
    oneshotCompleted = false;

}

// Find best of the worst comm_select time in the table
uint16_t best_comm_select_time_index() {
    int i, best_index;
    uint16_t best_value = 65535;
    for (i=best_index=0; i<COMM_SELECT_TRACK_TIMES; i++)
        if (worstCommSelectTimes[i] < best_value) {
            best_index = i;
            best_value = worstCommSelectTimes[best_index];
        }
    return best_index;
}

// Find worst comm_select time in the table
uint16_t worst_comm_select_time_index() {
    int i, worst_index;
    uint16_t worst_value = 0;
    for (i=worst_index=0; i<COMM_SELECT_TRACK_TIMES; i++)
        if (worstCommSelectTimes[i] > worst_value) {
            worst_index = i;
            worst_value = worstCommSelectTimes[worst_index];
        }
    return worst_index;
}

// Remember the longest times we've spent on the air
void log_longest_comm_select(uint32_t seconds) {
    int i, count;
    uint32_t sum;

    // Remember the absolute worst
    if (seconds > absoluteWorst)
        absoluteWorst = seconds;

    // Every day, throw away the worst half of the entries
    if (!ShouldSuppress(&lastCommSelectTimePurgeTime, 24L * 60L * 60L)) {
        for (i=0; i<COMM_SELECT_TRACK_TIMES/2; i++)
            worstCommSelectTimes[worst_comm_select_time_index()] = 0;
    }

    // If the current value is worst than the best, replace it
    i = best_comm_select_time_index();
    if (seconds > worstCommSelectTimes[i])
        worstCommSelectTimes[i] = seconds;

    // Compute the average from the non-null entries
    for (i=sum=count=0; i<COMM_SELECT_TRACK_TIMES; i++)
        if (worstCommSelectTimes[i] != 0) {
            count++;
            sum += worstCommSelectTimes[i];
        }

    // Remember the average
    if (count != 0) {
        stats()->oneshot_seconds = sum/count;
        if (failedCommSelects)
            DEBUG_PRINTF("%ds connect (%davg/%dmax %ldfail/%ldtotal)\n", seconds, sum/count, absoluteWorst, failedCommSelects, totalCommSelects);
        else
            DEBUG_PRINTF("%ds connect (%davg/%dmax)\n", seconds, sum/count, absoluteWorst);
    }

    // Log it

}

// Mark a select as having been completed
void comm_select_completed() {
    isCommSelectInProgress = false;
    if (lastCommSelectTime != 0) {
        if (get_seconds_since_boot() > lastCommSelectTime)
            log_longest_comm_select(get_seconds_since_boot() - lastCommSelectTime);
        lastCommSelectTime = 0;
    }
}

// Select a specific comms mode
void comm_select(uint16_t which, char *reason) {
    uint16_t original_which = which;

    strlcpy(last_select_reason, reason, sizeof(last_select_reason)-1);

    if (debug(DBG_COMM_MAX))
        DEBUG_PRINTF("SELECT: %s\n", reason);

    // Override the mode if desired
    which = comm_mode_override(which);
    if (original_which != which && debug(DBG_COMM_MAX))
        DEBUG_PRINTF("SELECT: OVERRIDE to %s\n", reason);

    // Exit if superfluous or inappropriate
    if (sensor_test_mode() && which != COMM_NONE)
        return;

    // Detect if we've failed a previous select
    if (isCommSelectInProgress) {
        isCommSelectInProgress = false;
        failedCommSelects++;
        switch (connect_state) {
        case CONNECT_STATE_LORA_MODULE:
            DEBUG_PRINTF("Failed to connect: lora module\n");
            stats()->errors_connect_lora++;
            break;
        case CONNECT_STATE_LORA_GATEWAY:
            DEBUG_PRINTF("Failed to connect: lora gateway\n");
            stats()->errors_connect_gateway++;
            break;
        case CONNECT_STATE_LORAWAN_GATEWAY:
            DEBUG_PRINTF("Failed to connect: lorawan gateway\n");
            stats()->errors_connect_gateway++;
            break;
        case CONNECT_STATE_FONA_MODULE:
            DEBUG_PRINTF("Failed to connect: fona module\n");
            stats()->errors_connect_fona++;
#ifdef FONA
            fona_request_full_reset();
#endif
            break;
        case CONNECT_STATE_WIRELESS_SERVICE:
            DEBUG_PRINTF("Failed to connect: carrier\n");
            stats()->errors_connect_wireless++;
            break;
        case CONNECT_STATE_DATA_SERVICE:
            DEBUG_PRINTF("Failed to connect: apn/data\n");
            stats()->errors_connect_data++;
            break;
        case CONNECT_STATE_APP_SERVICE:
            DEBUG_PRINTF("Failed to connect: ttserve\n");
            stats()->errors_connect_service++;
            break;
        }
    }

    // Oneshot is done if we're selecting NONE
    if (which == COMM_NONE)
        oneshotCompleted = true;

    // Handle deselection of existing mode
    lastCommSelectTime = 0;
    comm_powered_up = 0;
    comm_powered_down = get_seconds_since_boot();
    currently_deselected = true;
    comm_set_connect_state(CONNECT_STATE_UNKNOWN);

    // Terminate subsystem and power-off module
    switch (active_comm_mode) {
#ifdef LORA
    case COMM_LORA:
        lora_term(true);
        comm_set_connect_state(CONNECT_STATE_LORA_DESELECTED);
        break;
#endif
#ifdef FONA
    case COMM_FONA:
        fona_term(true);
        comm_set_connect_state(CONNECT_STATE_FONA_DESELECTED);
        break;
#endif
    }

    if (which != COMM_NONE) {

        // We're attempting to select something other than NONE
        lastCommSelectTime = get_seconds_since_boot();
        isCommSelectInProgress = true;
        totalCommSelects++;
        commCallNow = false;
        
    }

    // Now, presumptively under the assumption that we are about to
    // complete initialization, mark ourselves as currently
    // selected and active.  (This ordering is essential because,
    // for one, fona_init checks to see if we're selected
    // during reset processing.)
    active_comm_mode = which;
    currently_deselected = (which == COMM_NONE);

    // Initialize the subsystem as appropriate
#ifdef LORA
    if (which == COMM_LORA) {
        gpio_uart_select(UART_LORA);
        comm_last_powered_up = comm_powered_up = get_seconds_since_boot();
        comm_powered_down = 0;
        comm_set_connect_state(CONNECT_STATE_LORA_MODULE);
        lora_init();
    }
#endif
#ifdef FONA
    if (which == COMM_FONA) {
        gpio_uart_select(UART_FONA);
        comm_last_powered_up = comm_powered_up = get_seconds_since_boot();
        comm_powered_down = 0;
        comm_set_connect_state(CONNECT_STATE_FONA_MODULE);
        fona_init();
    }
#endif

}

// Initialization of this module and the entire state machine
void comm_init() {

    // Init statistics
    memset(stats(), 0, sizeof(stats_t));

    // Init state machines
    phone_init();
#ifdef BGEIGIE
    bgeigie_init();
#endif
    comm_select(COMM_NONE, "init");

    // Init the notion of when we last powered comms down
    comm_powered_down = get_seconds_since_boot();
    comm_powered_up = comm_last_powered_up = 0;

    // Init the first oneshot time to be halfway through its interval,
    // so as to stagger it away from the sensor measurement tempo
    lastOneshotTime = get_seconds_since_boot() + (2*get_oneshot_interval()/3);

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

    // One less completion pending to be processed, defensively coded
    if (pending_completions)
        pending_completions--;

    // Dispatch to the type of completion
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
        // Note that we must process lora completions even if deselected
        // so that we can deal with LoRaWAN "save state", which happens
        // after the deselect.
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

    // If for some reason we are being flooded with input and aren't
    // processing it quickly enough, it's better to drop it on the floor
    // and cause high-level software to time-out on requests/responses
    // than to overflow the app sched queue.
    if (pending_completions > 10)
        return;
    pending_completions++;

    // Schedule it
    app_sched_event_put(&type, sizeof(type), completion_event_handler);

}
