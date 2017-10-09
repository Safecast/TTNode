// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

// Lora state machine processing
// Note that the "elegant" LoraWAN power save/restore method is described here:
// https://www.microchip.com/forums/m945840.aspx#951895

#ifdef LORA

#include <stdio.h>
#include <stdlib.h>
#include "debug.h"
#include "config.h"
#include "comm.h"
#include "gpio.h"
#include "lora.h"
#include "lorafp.h"
#include "send.h"
#include "stats.h"
#include "recv.h"
#include "phone.h"
#include "misc.h"
#include "timer.h"
#include "gpio.h"
#include "io.h"
#include "serial.h"
#include "twi.h"
#include "storage.h"
#include "nrf_delay.h"
#include "tt.pb.h"
#include "pb_encode.h"
#include "pb_decode.h"

// Device states
#define COMM_LORA_SYSRESETRPL           COMM_STATE_DEVICE_START+0
#define COMM_LORA_INITCOMPLETED         COMM_STATE_DEVICE_START+1
#define COMM_LORA_SETADRRPL             COMM_STATE_DEVICE_START+2
#define COMM_LORA_SENDFPRPL             COMM_STATE_DEVICE_START+3
#define COMM_LORA_UNSOLICITED           COMM_STATE_DEVICE_START+4
#define COMM_LORA_RESETREQ              COMM_STATE_DEVICE_START+5
#define COMM_LORA_MACPAUSERPL           COMM_STATE_DEVICE_START+6
#define COMM_LORA_SETWDTRPL             COMM_STATE_DEVICE_START+7
#define COMM_LORA_TXRPL1                COMM_STATE_DEVICE_START+8
#define COMM_LORA_TXRPL2                COMM_STATE_DEVICE_START+9
#define COMM_LORA_GETVERRPL             COMM_STATE_DEVICE_START+10
#define COMM_LORA_RXRPL                 COMM_STATE_DEVICE_START+11
#define COMM_LORA_SLEEPRPL              COMM_STATE_DEVICE_START+12
#define COMM_LORA_MACRESUMERPL          COMM_STATE_DEVICE_START+13
#define COMM_LORA_HWEUIRPL              COMM_STATE_DEVICE_START+14
#define COMM_LORA_SETDEVEUIRPL          COMM_STATE_DEVICE_START+15
#define COMM_LORA_SETAPPEUIRPL          COMM_STATE_DEVICE_START+16
#define COMM_LORA_SETAPPKEYRPL          COMM_STATE_DEVICE_START+17
#define COMM_LORA_JOINRPL               COMM_STATE_DEVICE_START+18
#define COMM_LORA_LORAWANREQ            COMM_STATE_DEVICE_START+19
#define COMM_LORA_LORAREQ               COMM_STATE_DEVICE_START+20
#define COMM_LORA_RETRYJOIN             COMM_STATE_DEVICE_START+21
#define COMM_LORA_GETSNRRPL             COMM_STATE_DEVICE_START+22
#define COMM_LORA_SENDFQRPL             COMM_STATE_DEVICE_START+23
#define COMM_LORA_REJOIN1               COMM_STATE_DEVICE_START+24
#define COMM_LORA_REJOIN2               COMM_STATE_DEVICE_START+25
#define COMM_LORA_REJOIN3               COMM_STATE_DEVICE_START+26
#define COMM_LORA_SAVESTATERPL          COMM_STATE_DEVICE_START+27
#define COMM_LORA_RESTORESTATERPL       COMM_STATE_DEVICE_START+28
#define COMM_LORA_HWEUIDONE             COMM_STATE_DEVICE_START+29

// Delay that Microchip appears to need in many reset-related circumstances
#define MICROCHIP_LONG_DELAY_MS 1500

// Command buffer
static cmdbuf_t fromLora;

// LoRa vs LoRaWAN state and primary modes
static bool LoRaWAN_mode = false;
static bool LoRaWAN_mode_desired_after_reset = false;
static bool LoRaWAN_mode_try_the_other_on_failure = true;
// LPWAN chip or frequency detection
static bool isRN2483 = false;
static bool isRN2903 = false;

// Finer-grained state management
#define XMIT_REPLY_RETRIES_LORAWAN 1
#define XMIT_REPLY_RETRIES_LORA 3
static int xmitReplyRetriesLeft;
static bool awaitingTTGateReply = false;
static uint32_t beganAwaitingTTGateReply = 0;
static bool awaitingTTServeReply = false;
static uint16_t accept_retries;
static bool deferred_transmit = false;
static char deferred_transmit_buffer[CMD_MAX_LINELENGTH + 1];
static bool fTermAfterSleep = false;

// For gateway connectivity checking
static uint32_t lastGatewayConnectivityCheck = 0L;

// Things to deal with the atypical (power-hungry) case of "listen enabled" mode
static bool sleepDisabled = false;
static bool receive_from_lpwan_mode = false;

// Initialization and fault-related
static bool loraFirstResetAfterInit = false;
static uint32_t watchdog_set_time;
static bool loraInitCompleted = false;
static bool loraInitEverCompleted = false;
static bool loraInitInProgress = false;
static uint32_t loraInitLastInitiated = 0L;
static uint16_t lorafpRegionCommandNumber;

// Relay state
static uint8_t toRelayBuffer[CMD_MAX_LINELENGTH];
static uint16_t toRelayBufferLength;
static uint32_t toRelayDevice;
static int toRelaySNR;

// Get MTU
uint16_t lora_get_mtu() {

    // https://thethingsnetwork.slack.com/files/johan/F41UBKAKA/lorawan_regional_parameters_v1_0-20161012_1397_1.pdf
    if (LoRaWAN_mode)
        return(51);

    // Measured with mtu test on 2017-02-08; lora module hangs above 126
    return(125);
}

// Transmit the command to the LPWAN
void lora_send(char *msg) {

    if (!comm_is_initialized())
        return;

    // Defensive programming, to cover spurious app_sched events occurring after module power-down
    if (gpio_current_uart() != UART_LORA)
        return;

    if (!serial_transmit_enabled())
        DEBUG_PRINTF("? %s\n", msg);
    else if (debug(DBG_TX))
        DEBUG_PRINTF("> %s\n", msg);

    // Send it
    while (*msg != '\0')
        serial_send_byte(*msg++);

    // Send terminating newline
    serial_send_byte('\r');
    serial_send_byte('\n');

}

// Are we in a mode where we need a receive outstanding?
bool receive_mode_active() {
    if (receive_from_lpwan_mode)
        return true;
    if (awaitingTTGateReply)
        return true;
    if (awaitingTTServeReply)
        return true;
    return false;
}

// Set LPWAN to idle or receive mode, based on modes including LoRaWAN mode
void restart_receive() {
    if (LoRaWAN_mode || !receive_mode_active()) {
#ifdef DISABLE_LPWAN_SLEEP
        // Only if debugging and want to keep the chip awake so we can try out commands
        comm_cmdbuf_set_state(&fromLora, COMM_STATE_IDLE);
#else
        if (sleepDisabled || !loraInitCompleted) {
            comm_cmdbuf_set_state(&fromLora, COMM_STATE_IDLE);
        } else {
            char command[32];
            if (fromLora.state == COMM_LORA_SLEEPRPL) {
                DEBUG_PRINTF("Already sleeping!\n");
            } else {
                // Send the sleep command
                sprintf(command, "sys sleep %d", LPWAN_SLEEP_MILLISECONDS);
                lora_send(command);
                comm_cmdbuf_set_state(&fromLora, COMM_LORA_SLEEPRPL);
                // Totally prohibit ANY serial output while we are sleeping
                serial_transmit_enable(false);
            }
        }
#endif
    } else {
        lora_send("radio rx 0");
        comm_cmdbuf_set_state(&fromLora, COMM_LORA_RXRPL);
        // Totally prohibit ANY serial output while we are sleeping
        serial_transmit_enable(false);
    }
}

// Simple wrapper to simplify thisargisL
bool thisargisL(char *what) {
    return (comm_cmdbuf_this_arg_is(&fromLora, what));
}

// Set lorawan to the specified state
void setstateL(uint16_t newstate) {
    comm_cmdbuf_set_state(&fromLora, newstate);
}

// Set LPWAN to idle or receive mode, based on modes
void setidlestateL() {
    if (receive_mode_active())
        restart_receive();
    else {
#ifdef DISABLE_LPWAN_SLEEP
        setstateL(COMM_STATE_IDLE);
#else
        restart_receive();
#endif
    }
}

// Synchronously process a new state (doing so "nested" if calling from cmdbuf_process)
// This is tremendously convenient, as opposed to introducing new intermediate states.
void processstateL(uint16_t newstate) {
    setstateL(newstate);
    fromLora.complete = true;
    lora_process();
}

// Process a transmit that was deferred because of a receive-in-progress
bool sent_pending_outbound() {
    if (!deferred_transmit)
        return false;
    if (fromLora.state == COMM_LORA_SLEEPRPL) {
        DEBUG_PRINTF("Sleeping!\n");
        return(false);
    }
#ifdef FLOWTRACE
    DEBUG_PRINTF("(I'm sending now.)\n");
#endif
    deferred_transmit = false;
    lora_send(deferred_transmit_buffer);
    setstateL(COMM_LORA_TXRPL1);
    return true;
}

// Process the hex-encoded portion of a message received from the LPWAN
void process_rx(char *in) {
    uint8_t buffer[CMD_MAX_LINELENGTH];
    uint16_t msgtype, decodedBytes;
    ttproto_Telecast message;

    // Regardless of what we received, indicate that we're no longer waiting for
    // a TTServe reply, because we only want to listen for a single receive
    // window, for power reasons  If we don't pick it up now, we'll pick it up
    // eventually on the next message to the service.
    awaitingTTServeReply = false;

    // Decode the message
    msgtype = comm_decode_received_message(in, &message, buffer, sizeof(buffer) - 1, &decodedBytes);
    if (msgtype == MSG_NOT_DECODED) {
        DEBUG_PRINTF("Unrecognized message received.\n");
        // Bad received message
        setidlestateL();
        return;
    }

    // Bump stats about what we've received on the wire
    stats_io(0, decodedBytes);

    // A reply from an "are you there?" ping we sent to TTGATE?
    if (msgtype == MSG_REPLY_TTGATE) {
        awaitingTTGateReply = false;
        DEBUG_PRINTF("Lora gateway responded\n");
        processstateL(COMM_LORA_INITCOMPLETED);
        return;
    }

    // The reply is NOT from TTGATE.  If we're awaiting one from TTGATE, though,
    // discard it and just send a ping to TTGATE to attempt to get that reply.
    if (awaitingTTGateReply) {
        setstateL(COMM_STATE_IDLE);
        DEBUG_PRINTF("Unrecognized message received  while awaiting Lora gateway response\n");
        // Only retry for a reasonable amount of time, else a bunch of devices searching
        // for a gateway would keep each other occupied forever.
        if (!ShouldSuppress(&beganAwaitingTTGateReply, 30))
            setidlestateL();
        else {
            uint32_t saveBeganAwaitingTTGateReply = beganAwaitingTTGateReply;
            send_ping_to_service(REPLY_TTGATE);
            beganAwaitingTTGateReply = saveBeganAwaitingTTGateReply;
        }
        return;
    }

    // If this is a reply from a text message that we sent to TTSERVE, process it.
    if (msgtype == MSG_REPLY_TTSERVE) {
        DEBUG_PRINTF("Reply received from TTServe\n");
        awaitingTTGateReply = false;
        // Set idle state before we do this so that we can process recv_message without being "busy"
        comm_cmdbuf_reset(&fromLora);
        setidlestateL();
        recv_message_from_service((char *)buffer);
        return;
    }

    // If this is a text message, just display it.
    if (msgtype == MSG_TELECAST) {
        awaitingTTGateReply = false;
        DEBUG_PRINTF("%s\n", (char *)buffer);
        comm_cmdbuf_reset(&fromLora);
        setidlestateL();
        return;
    }

    // This is a Safecast message.  If we're not configured as a relay, we're done.
    STORAGE *s = storage();
    if (LoRaWAN_mode || (s->flags & FLAG_RELAY) == 0) {
        if (debug(DBG_COMM_MAX))
            DEBUG_PRINTF("(safecast message ignored)\n");
        comm_cmdbuf_reset(&fromLora);
        setidlestateL();
        return;
    }

    // We know that it's a relay message
    if (debug(DBG_COMM_MAX))
        DEBUG_PRINTF("Relaying Safecast message.\n");

    // This is a Safecast or bGeigie message that we're relaying.
    // Check to make sure that it has a device ID, and that we haven't already relayed it
    if (!message.has_device_id) {
        comm_cmdbuf_reset(&fromLora);
        setidlestateL();
        return;
    }

    // Determine whether or not we've already relayed it
    uint32_t thisDeviceID = io_get_device_address();
    if ((message.has_relay_device1 && message.relay_device1 == thisDeviceID)
        || (message.has_relay_device2 && message.relay_device2 == thisDeviceID)
        || (message.has_relay_device3 && message.relay_device3 == thisDeviceID)
        || (message.has_relay_device4 && message.relay_device4 == thisDeviceID)
        || (message.has_relay_device5 && message.relay_device5 == thisDeviceID)) {
        DEBUG_PRINTF("RELAY: already relayed\n");
        comm_cmdbuf_reset(&fromLora);
        setidlestateL();
        return;
    }

    // Assign it a routing slot
    if (!message.has_relay_device1) {
        message.has_relay_device1 = true;
        message.relay_device1 = thisDeviceID;
    } else if (!message.has_relay_device2) {
        message.has_relay_device2 = true;
        message.relay_device2 = thisDeviceID;
    } else if (!message.has_relay_device3) {
        message.has_relay_device3 = true;
        message.relay_device3 = thisDeviceID;
    } else if (!message.has_relay_device4) {
        message.has_relay_device4 = true;
        message.relay_device4 = thisDeviceID;
    } else if (!message.has_relay_device5) {
        message.has_relay_device5 = true;
        message.relay_device5 = thisDeviceID;
    } else {
        // If no slots left, relay no further.
        DEBUG_PRINTF("RELAY: too many hops\n");
        comm_cmdbuf_reset(&fromLora);
        setidlestateL();
        return;
    }

    // Encode the PB to be transmitted
    pb_ostream_t stream = pb_ostream_from_buffer(toRelayBuffer, sizeof(toRelayBuffer));
    uint16_t status = pb_encode(&stream, ttproto_Telecast_fields, &message);
    if (!status) {
        DEBUG_PRINTF("Relay pb_encode: %s\n", PB_GET_ERROR(&stream));
        comm_cmdbuf_reset(&fromLora);
        setidlestateL();
        return;
    }
    toRelayBufferLength = stream.bytes_written;
    toRelayDevice = message.device_id;

    // Get the SNR of the received message, which will then relay it.
    lora_send("radio get snr");
    setstateL(COMM_LORA_GETSNRRPL);

}

// Reset our watchdog timer
void lora_watchdog_reset() {
    watchdog_set_time = get_seconds_since_boot();
}

// Process a completed lora command
void lora_complete() {

    // If we're not in an idle state, let's not process commands for it
    if (fromLora.state != COMM_STATE_IDLE)
        return;

    fromLora.state = COMM_LORA_UNSOLICITED;

}

// Set up so that we can send commands to the chip
void lora_enter_command_mode() {
    sleepDisabled = true;
    receive_from_lpwan_mode = false;
}

// Return true if we're initialized.  Note that we also say that it's ok
// to send to the service if init is in progress, because we need to send
// the Ping to TTGATE.
bool lora_can_send_to_service() {
    return(loraInitCompleted);
}

// Return true if transmitting would be pointless
bool lora_is_busy() {

    // Take not of whether we're temporarily "not listening" and thus not able to transmit
    bool fSleeping = (fromLora.state == COMM_LORA_RXRPL || fromLora.state == COMM_LORA_SLEEPRPL);

    // Exit if we're in the middle of sending a different message,
    // This can happen because transmits take a finite amount of time,
    // and we might be being asked to transmit both for the local
    // geiger counter as well as the URSC-connected bGeigie.
    if (fromLora.state != COMM_STATE_IDLE && !fSleeping) {

        // Causes communications issues doing this debug output in the interrupt handler,
        // , so only use this when doing super debugging.
#ifdef FLOWTRACE
        if (fromLora.state == COMM_LORA_TXRPL2)
            DEBUG_PRINTF("(Can't send - busy xmitting!)\n");
        else
            DEBUG_PRINTF("(Can't send - busy %d!)\n", fromLora.state);
#endif

        return true;
    }

    if (fSleeping && deferred_transmit) {
#ifdef FLOWTRACE
        if (debug(DBG_COMM_MAX))
            DEBUG_PRINTF("(Can't send - something else waiting.)\n");
#endif
        return true;
    }

    if (fromLora.state == COMM_LORA_RXRPL) {
        if (debug(DBG_COMM_MAX))
            DEBUG_PRINTF("(Busy waiting for reply.)\n");
        return true;
    }

    return false;
}

// Transmit a well-formed protocol buffer to the LPWAN as a message
bool lora_send_to_service(uint8_t *buffer, uint16_t length, uint16_t RequestType) {
    char *command;

    // Exit if we're not yet initialized
    if (!comm_is_initialized()) {
        if (debug(DBG_TX))
            DEBUG_PRINTF("Lora not yet initialized.\n");
        return false;
    }

    // Once every [N] minutes, even if this wasn't a request asking for a reply, force a RequestType so
    // that we give TTGATE an opportunity to send us a "down" message in case it can't reach the service.
    if (RequestType == REPLY_NONE)
        if (!ShouldSuppress(&lastGatewayConnectivityCheck, FAILOVER_CHECK_MINUTES * 60L))
            RequestType = REPLY_TTSERVE;

    // If this is a request that requires a reply, remember that we're doing so.
    awaitingTTGateReply = false;
    awaitingTTServeReply = false;
    if (RequestType == REPLY_TTGATE) {
        comm_set_connect_state(CONNECT_STATE_LORA_GATEWAY);
        DEBUG_PRINTF("Requesting Lora gateway reply\n");
        awaitingTTGateReply = true;
        beganAwaitingTTGateReply = get_seconds_since_boot();
    } if (RequestType == REPLY_TTSERVE)
          awaitingTTServeReply = true;

    // If we're busy doing something else, drop this
    if (lora_is_busy())
        return false;

    // Do different types of transmit, based on mode.  Start by assuming no retries.
    xmitReplyRetriesLeft = 0;
    if (LoRaWAN_mode) {
        if (RequestType == REPLY_NONE) {
            command = "mac tx uncnf 1 ";
        } else {
            command = "mac tx cnf 1 ";
            // Indicate that we should do a LoraWAN ping for a reply just for good measure,
            // simply because we only have a small window to receive.
            xmitReplyRetriesLeft = XMIT_REPLY_RETRIES_LORAWAN;
        }
    } else {
        command = "radio tx ";
        if (RequestType == REPLY_TTGATE) {
            // Indicate that we should do a Lora ping for a reply just for good measure,
            // simply because we only have a small window to receive.
            xmitReplyRetriesLeft = XMIT_REPLY_RETRIES_LORA;
        }
    }

    // Convert the binary buffer to a command with hexified data
    HexCommand(deferred_transmit_buffer, sizeof(deferred_transmit_buffer),
               command,
               buffer, length);

    // Bump stats about what we've transmitted
    stats_io(length, 0);

    // If we are sleeping/receiving, defer the xmit until it completes
    if (fromLora.state == COMM_LORA_RXRPL || fromLora.state == COMM_LORA_SLEEPRPL) {
        deferred_transmit = true;
#ifdef FLOWTRACE
        DEBUG_PRINTF("(I'll send in a moment.)\n");
#endif
    } else {
        deferred_transmit = false;
        lora_send(deferred_transmit_buffer);
        setstateL(COMM_LORA_TXRPL1);
    }

    // Notify caller that we transmitted it (or successfully deferred it)
    return true;
}

// Do LPWAN reset processing
bool lora_needed_to_be_reset() {
    uint32_t secondsSinceBoot = get_seconds_since_boot();

    // Initialize the entire cmd subsystem only after the chip has
    // had a chance to stabilize after boot.
    if (!loraInitCompleted && !loraInitInProgress && secondsSinceBoot > BOOT_DELAY_UNTIL_INIT) {
        lora_reset(false);
        return true;
    }

    if (watchdog_set_time >= secondsSinceBoot)
        lora_watchdog_reset();

    // This is a special case that just causes us to go idle if we
    // happen to miss the OK reply to a sleep request.
    // This is just intended to jostle it into response
    if (secondsSinceBoot >= SLEEP_WATCHDOG_SECONDS) {
        if ((secondsSinceBoot - watchdog_set_time) > SLEEP_WATCHDOG_SECONDS)
            if (fromLora.state == COMM_LORA_SLEEPRPL) {
                serial_transmit_enable(true);
                lora_send("sys get ver");
                setstateL(COMM_STATE_IDLE);
                return false;
            }
    }

    // Reset all state and the LPWAN chip if appropriate,
    // handling clock wrap.  Only do this, though, if
    // the lpWAN is in a non-idle state, because we
    // don't want to be performing resets of completely
    // idle devices.
    if (secondsSinceBoot >= LORA_WATCHDOG_SECONDS) {
        if ((secondsSinceBoot - watchdog_set_time) > LORA_WATCHDOG_SECONDS) {
            // Auto-switch if the module is missing or nonresponsive
            if (fromLora.state == COMM_LORA_GETVERRPL && storage()->wan == WAN_AUTO) {
                stats()->errors_lora++;
                comm_select(COMM_FONA, "module failure - lora handoff");
                return true;
            }
            if (fromLora.state != COMM_STATE_IDLE) {
                DEBUG_PRINTF("WATCHDOG: stuck in fromLora(%d)\n", fromLora.state);
                lora_reset(true);
                stats()->errors_lora++;
                return true;
            }
        }
    }

    // Not reset
    return false;

}

// Initialize or reinitialize the LPWAN chip and state machine
void lora_reset(bool force) {

    // Prevent recursion
    if (!force && loraInitInProgress)
        return;

    // If we're forcing a reset, init as though it had never been completed,
    // which means for TTN a full re-join
    if (force)
        loraInitEverCompleted = false;

    // Kick off a device reset
    processstateL(COMM_LORA_RESETREQ);

}

// Request state for debugging
void lora_request_state() {
    DEBUG_PRINTF("Lora %s: st=%d cc=%d b=%d,%d,%d '%s'\n", comm_is_deselected() ? "disconnected" : "connected", fromLora.state, fromLora.complete, fromLora.busy_length, fromLora.busy_nextput, fromLora.busy_nextget, fromLora.buffer);
}

// One-time or per-oneshot init
void lora_init() {

    // Do the state machine initialization
    comm_cmdbuf_init(&fromLora, CMDBUF_TYPE_LORA);
    comm_cmdbuf_set_state(&fromLora, COMM_LORA_RESETREQ);
    awaitingTTGateReply = false;
    awaitingTTServeReply = false;
    loraInitInProgress = false;
    loraInitCompleted = false;
    loraFirstResetAfterInit = true;
    fTermAfterSleep = false;

    // Kick the module into doing something, else it will just be idle
    lora_send("sys get ver");

}

// Terminate, power down, and set the state of things such that we will look "not busy"
// when we're in a deselected mode.
void lora_do_term() {
    gpio_uart_select(UART_NONE);
    deferred_transmit = false;
    serial_transmit_enable(true);
    setstateL(COMM_STATE_IDLE);
}

// Termination AND power down
void lora_term(bool fPowerdown) {
    if (fPowerdown) {
        if (loraInitCompleted && LoRaWAN_mode) {
            fTermAfterSleep = true;
        } else {
            lora_do_term();
        }
    }
}

// Process byte received from LPWAN
void lora_received_byte(uint8_t databyte) {
    comm_cmdbuf_received_byte(&fromLora, databyte);
}

// Primary state processing of the command buffer
void lora_process() {
    char buffer[CMD_MAX_LINELENGTH];

    // If there was a spurious app sched event remaining after powering off the module, ignore it
    if (gpio_current_uart() != UART_LORA)
        return;

    // If it's not complete, just exit.
    if (!fromLora.complete)
        return;

    // If we're idle, set the state based on buffer contents
    if (fromLora.state == COMM_STATE_IDLE)
        lora_complete();

    // If extreme debugging
    if (debug(DBG_RX) && fromLora.buffer[0] != '\0')
        if (fromLora.state != COMM_LORA_RESETREQ)
            DEBUG_PRINTF("<%d %s\n", fromLora.state, fromLora.buffer);

    //////////////
    ///////
    /////// This state machine is divided into the following blocks
    /////// 1. States associated with resetting the LPWAN
    /////// 2. States associated with attempting to init in LoRa mode
    /////// 3. States associated with attempting to init in LoRaWAN mode
    /////// 4. States associated with completing the initialization
    /////// 5. Steady state states
    ///////
    //////////////

    switch (fromLora.state) {

        ///////
        /////// This section of state management is related to doing LPWAN resets
        ///////

    case COMM_LORA_RESETREQ: {
        DEBUG_PRINTF("LPWAN initializing\n");
        if (loraFirstResetAfterInit)
            loraFirstResetAfterInit = false;
        else
            stats()->resets++;
        gpio_indicate(INDICATE_LORA_INITIALIZING);
        lora_watchdog_reset();
        loraInitCompleted = false;
        loraInitInProgress = true;
        loraInitLastInitiated = get_seconds_since_boot();
        serial_transmit_enable(true);
        setstateL(COMM_STATE_IDLE);
        lora_send("sys get ver");
        setstateL(COMM_LORA_GETVERRPL);
        break;
    }

    case COMM_LORA_GETVERRPL: {
        STORAGE *s = storage();
        // Delay after any get ver.  This matters for Microchip timing reasons.
        nrf_delay_ms(MICROCHIP_LONG_DELAY_MS);
        // There may be garbage, so retry until we get in sync
        if (!loraInitEverCompleted)
            DEBUG_PRINTF("%s\n", &fromLora.buffer[fromLora.args]);
        if (thisargisL("rn2483")) {
            isRN2483 = true;
            strlcpy(stats()->module_lora, "RN2493", sizeof(stats()->module_lora)-1);
            if (s->lpwan_region[0] == '\0')
                strlcpy(s->lpwan_region, "eu", sizeof(s->lpwan_region)-1);
        } else if (thisargisL("rn2903")) {
            isRN2903 = true;
            strlcpy(stats()->module_lora, "RN2903", sizeof(stats()->module_lora)-1);
            if (s->lpwan_region[0] == '\0')
                strlcpy(s->lpwan_region, "us", sizeof(s->lpwan_region)-1);
        } else if (thisargisL("invalid_param")) {
            // This is totally expected, as we are trying to re-sync
            lora_send("sys get ver");
            setstateL(COMM_LORA_GETVERRPL);
            break;
        } else {
            DEBUG_PRINTF("Unexpected chip ID/version\n");
            lora_send("sys get ver");
            setstateL(COMM_LORA_GETVERRPL);
            break;
        }
        // Init into the appropriate receive mode
        if ((s->flags & (FLAG_LISTEN|FLAG_RELAY)) != 0)
            receive_from_lpwan_mode = true;
        else
            receive_from_lpwan_mode = false;
        // If we're restarting, then jump into the appropriate mode
        if (loraInitEverCompleted) {
            processstateL(LoRaWAN_mode ? COMM_LORA_LORAWANREQ : COMM_LORA_LORAREQ);
            break;
        }
        // This is a first time init
        if (s->wan == WAN_LORA || s->wan == WAN_LORAWAN)
            LoRaWAN_mode_try_the_other_on_failure = false;
        if (s->wan == WAN_LORA_THEN_LORAWAN || s->wan == WAN_AUTO || s->wan == WAN_LORA)
            processstateL(COMM_LORA_LORAREQ);
        else
            processstateL(COMM_LORA_LORAWANREQ);
        break;
    }

    case COMM_LORA_SYSRESETRPL: {
        // Let things settle down after the sys reset
        nrf_delay_ms(MICROCHIP_LONG_DELAY_MS);
        if (loraInitEverCompleted) {
            processstateL(COMM_LORA_HWEUIDONE);
            break;
        }
        lora_send("sys get hweui");
        setstateL(COMM_LORA_HWEUIRPL);
        break;
    }

    case COMM_LORA_HWEUIRPL: {
        char *devEui = (char *) &fromLora.buffer[fromLora.args];
        // Save it in NVRAM if it has changed
        if (strcmp(storage()->ttn_dev_eui, devEui) != 0) {
            strlcpy(storage()->ttn_dev_eui, devEui, sizeof(storage()->ttn_dev_eui));
            DEBUG_PRINTF("Saving DevEUI: %s\n", devEui);
            storage_save(false);
            nrf_delay_ms(MICROCHIP_LONG_DELAY_MS);
        } else if (!loraInitEverCompleted) {
            DEBUG_PRINTF("DevEui: %s\n", devEui);
        }
        processstateL(COMM_LORA_HWEUIDONE);
        break;
    }

    case COMM_LORA_HWEUIDONE: {
        // We're always in this mode after a sys reset
        LoRaWAN_mode = true;
        // Dispatch based on what mode we want to be in
        if (LoRaWAN_mode_desired_after_reset)
            processstateL(COMM_LORA_MACRESUMERPL);
        else {
            lora_send("mac pause");
            setstateL(COMM_LORA_MACPAUSERPL);
            LoRaWAN_mode = false;
        }
        break;
    }

        ///////
        /////// This section of state management is common to both init paths
        ///////

    case COMM_LORA_INITCOMPLETED: {
        // Stay in whatever mode we were first in after boot, across resets
        LoRaWAN_mode_try_the_other_on_failure = false;
        // Done with initialization
        loraInitInProgress = false;
        loraInitCompleted = true;
        loraInitEverCompleted = true;
        comm_select_completed();
        if (LoRaWAN_mode) {
            comm_set_connect_state(CONNECT_STATE_LORAWAN_ACTIVE);
            gpio_indicate(INDICATE_LORAWAN_CONNECTED);
        } else {
            comm_set_connect_state(CONNECT_STATE_LORA_ACTIVE);
            gpio_indicate(INDICATE_LORA_CONNECTED);
        }
        DEBUG_PRINTF("LPWAN online: %s\n", LoRaWAN_mode ? "LoRaWAN" : "LoRa");
        // Use brute force to ensure that we don't just start jamming commands
        // down the device's throat after successful initialization.  We've
        // seen cases where just after a reset the lpwan chip goes into a
        // state in which it just does nothing but say "busy"
        nrf_delay_ms(250);
        setidlestateL();
        // Initiate a service upload if one is pending
        comm_update_service();
        break;
    }

        ///////
        /////// This section of state management is related to joining in LoRa mode
        ///////

    case COMM_LORA_LORAREQ: {
        DEBUG_PRINTF("Entering LoRa mode\n");
        lorafpRegionCommandNumber = 0;
        gpio_indicate(INDICATE_LORA_INITIALIZING);
        LoRaWAN_mode_desired_after_reset = false;
        setstateL(COMM_STATE_IDLE);
        lora_send("sys reset");
        setstateL(COMM_LORA_SYSRESETRPL);
        break;
    }

    case COMM_LORA_MACPAUSERPL: {
        // Indicate that we are now NOT in LoRaWAN mode
        STORAGE *s = storage();
        if ((s->flags & FLAG_RELAY) == 0)
            sprintf(buffer, "radio set wdt %lu", RECEIVE_TIMEOUT_LISTEN_SECONDS * 1000L);
        else
            sprintf(buffer, "radio set wdt %lu", RECEIVE_TIMEOUT_RELAY_SECONDS * 1000L);
        lora_send(buffer);
        setstateL(COMM_LORA_SETWDTRPL);
        break;
    }

    case COMM_LORA_SETWDTRPL: {
        STORAGE *s = storage();
        // Stay in the same state, sending frequency plan commands until there are none left for the region
        if (lorafp_get_command(s->lpwan_region, false, lorafpRegionCommandNumber, buffer, sizeof(buffer))) {
            lorafpRegionCommandNumber++;
            lora_send(buffer);
            setstateL(COMM_LORA_SETWDTRPL);
            break;
        }
    }
        // fallthrough when no more commands to send
    case COMM_LORA_SENDFQRPL: {
        setstateL(COMM_STATE_IDLE);
        // Send a ping in order to see if a gateway is actually there, because
        // if it isn't there we may give up and switch to LoRaWAN mode
        if (LoRaWAN_mode_try_the_other_on_failure) {
            DEBUG_PRINTF("Is Lora gateway present?\n");
            send_ping_to_service(REPLY_TTGATE);
        } else
            processstateL(COMM_LORA_INITCOMPLETED);
        break;
    }

        ///////
        /////// This section of state management is related to joining in LoRaWAN mode
        ///////

    case COMM_LORA_LORAWANREQ: {
        DEBUG_PRINTF("Entering LoRaWAN mode\n");
        lorafpRegionCommandNumber = 0;
        gpio_indicate(INDICATE_LORAWAN_INITIALIZING);
        LoRaWAN_mode_desired_after_reset = true;
        setstateL(COMM_STATE_IDLE);
        if (loraInitEverCompleted) {
            lora_send("mac join abp");
            setstateL(COMM_LORA_RESTORESTATERPL);
        } else {
            lora_send("sys reset");
            setstateL(COMM_LORA_SYSRESETRPL);
        }
        break;
    }

    case COMM_LORA_MACRESUMERPL: {
        accept_retries = 0;
        sprintf(buffer, "mac set deveui %s", storage()->ttn_dev_eui);
        lora_send(buffer);
        setstateL(COMM_LORA_SETDEVEUIRPL);
        break;
    }

    case COMM_LORA_SETDEVEUIRPL: {
        char *appEui = storage()->ttn_app_eui;
        sprintf(buffer, "mac set appeui %s", appEui);
        lora_send(buffer);
        setstateL(COMM_LORA_SETAPPEUIRPL);
        break;
    }

    case COMM_LORA_SETAPPEUIRPL: {
        char *appKey = storage()->ttn_app_key;
        sprintf(buffer, "mac set appkey %s", appKey);
        lora_send(buffer);
        setstateL(COMM_LORA_SETAPPKEYRPL);
        break;
    }

    case COMM_LORA_SETAPPKEYRPL: {
#if 1   // Disabled 2017-05-03 because the commands don't seem to be compatible with EU gateways
        // Re-enabled 2017-05-05 after upgrading lorafp
        STORAGE *s = storage();
        // Stay in the same state, sending frequency plan commands until there are none left for the region
        if (lorafp_get_command(s->lpwan_region, true, lorafpRegionCommandNumber, buffer, sizeof(buffer))) {
            lorafpRegionCommandNumber++;
            lora_send(buffer);
            setstateL(COMM_LORA_SETAPPKEYRPL);
            break;
        }
#endif
    }
        // fallthrough when no more commands to send
        // See https://www.microchip.com/forums/m945840.aspx#951895
    case COMM_LORA_SENDFPRPL: {
        lora_send("mac set devaddr 00000000");
        setstateL(COMM_LORA_REJOIN1);
        break;
    }

        // See https://www.microchip.com/forums/m945840.aspx#951895
    case COMM_LORA_REJOIN1: {
        lora_send("mac set nwkskey 00000000000000000000000000000000");
        setstateL(COMM_LORA_REJOIN2);
        break;
    }

        // See https://www.microchip.com/forums/m945840.aspx#951895
    case COMM_LORA_REJOIN2: {
        lora_send("mac set appskey 00000000000000000000000000000000");
        setstateL(COMM_LORA_REJOIN3);
        break;
    }

    case COMM_LORA_REJOIN3: {
        lora_send("mac set adr off");
        setstateL(COMM_LORA_SETADRRPL);
        break;
    }

    case COMM_LORA_SAVESTATERPL: {
        lora_do_term();
        setstateL(COMM_STATE_IDLE);
        break;
    }

    case COMM_LORA_RESTORESTATERPL: {
        if (thisargisL("ok")) {
            setstateL(COMM_LORA_RESTORESTATERPL);
        } else if (thisargisL("accepted")) {
            processstateL(COMM_LORA_INITCOMPLETED);
        } else {
            processstateL(COMM_LORA_RESETREQ);
        }
        break;
    }

    case COMM_LORA_SETADRRPL:
        // fallthrough
    case COMM_LORA_RETRYJOIN: {
        comm_set_connect_state(CONNECT_STATE_LORAWAN_GATEWAY);
        lora_send("mac join otaa");
        setstateL(COMM_LORA_JOINRPL);
        break;
    }

    case COMM_LORA_JOINRPL: {
        bool fRetry = false;
        if (thisargisL("ok")) {
            // this is expected response from initiating the rcv,
            // so just reset the buffer and keep waiting for a message to come in
            setstateL(COMM_LORA_JOINRPL);
        } else if (thisargisL("accepted")) {
            stats()->joins++;
            stats()->joins_today++;
            processstateL(COMM_LORA_INITCOMPLETED);
        } else if (thisargisL("busy")) {
            DEBUG_PRINTF("Join busy, retrying.\n");
            // This is not at all expected, but it means that we're
            // moving too quickly and we should try again.
            fRetry = true;
        } else if (thisargisL("no_free_ch")) {
            DEBUG_PRINTF("No free channel on join, retrying.\n");
            fRetry = true;
        } else if (thisargisL("denied")) {
            DEBUG_PRINTF("Join denied, retrying.\n");
            stats()->denies++;
            stats()->denies_today++;
            fRetry = true;
        } else {
            DEBUG_PRINTF("Unknown join response.\n");
            nrf_delay_ms(MICROCHIP_LONG_DELAY_MS);
            setstateL(COMM_LORA_JOINRPL);
        }

        if (fRetry) {
            if (++accept_retries >= LORAWAN_JOIN_RETRIES) {
                // If we can't rejoin "optimally" after having joined successfully,
                // try a full join just in case something happened on the service side
                // to cause state to have fundamentally changed
                loraInitEverCompleted = false;
                // If we aren't supposed to try the other, we need to just retry this one
                if (!LoRaWAN_mode_try_the_other_on_failure) {
                    processstateL(COMM_LORA_RESETREQ);
                } else {

#ifndef FONA
                    // If there's no FONA, switch into LORA mode
                    processstateL(COMM_LORA_LORAREQ);
#else
                    // If we're supposed to try LORA then LORAWAN and we found none of them, go to Cell.
                    if (storage()->wan == WAN_AUTO) {
                        setidlestateL();
                        comm_select(COMM_FONA, "handoff from lora");
                    } else {
                        processstateL(COMM_LORA_LORAREQ);
                    }
#endif

                }
            } else {
                DEBUG_PRINTF("Attempt #%d of %d\n", accept_retries + 1, LORAWAN_JOIN_RETRIES);
                processstateL(COMM_LORA_RETRYJOIN);
            }
        }
        break;
    }

        ///////
        /////// This section of state management is related to steady-state operations
        ///////

    case COMM_STATE_IDLE:
        setstateL(COMM_STATE_IDLE);
        return;

    case COMM_STATE_COMPLETE: {
        DEBUG_PRINTF("** wtf?? (%d) %s\n", fromLora.type, &fromLora.buffer[fromLora.args]);
        setstateL(COMM_STATE_IDLE);
        break;
    }

    case COMM_LORA_SLEEPRPL: {
        setstateL(COMM_STATE_IDLE);
        // Re-enable transmits now that we're safe to do so
        serial_transmit_enable(true);
        if (fTermAfterSleep) {
            fTermAfterSleep = false;
            lora_send("mac save");
            setstateL(COMM_LORA_SAVESTATERPL);
            break;
        }
        if (!sent_pending_outbound())
            restart_receive();
        break;
    }

    case COMM_LORA_GETSNRRPL: {
        uint16_t ms, i;
        toRelaySNR = atoi((char *)&fromLora.buffer[fromLora.args]);
        // Do a random delay so that we don't step on others who have similar WWAN visibility to the message
        ms = 5000 + io_get_random(5000);
#define delay_interval_ms 100
        for (i=0; i<ms; i+=delay_interval_ms)
            nrf_delay_ms(delay_interval_ms);
        // Set idle (else the send_to_service will be blocked) and transmit it
        comm_cmdbuf_set_state(&fromLora, COMM_STATE_IDLE);
        if (send_to_service(toRelayBuffer, toRelayBufferLength, REPLY_NONE, SEND_1))
            DEBUG_PRINTF("RELAY %lu snr=%d\n", toRelayDevice, toRelaySNR);
        else {
            if (!sent_pending_outbound())
                restart_receive();
        }
        break;
    }

    case  COMM_LORA_TXRPL1: {
        if (thisargisL("ok"))
            setstateL(COMM_LORA_TXRPL2);
        else
            setidlestateL();
        break;
    }

    case  COMM_LORA_TXRPL2: {
        if (thisargisL("radio_tx_ok") || thisargisL("mac_tx_ok")) {
            setidlestateL();
        } else if (thisargisL("mac_rx")) {
            comm_cmdbuf_next_arg(&fromLora);
            // skip mac_rx
            thisargisL("*");
            comm_cmdbuf_next_arg(&fromLora);
            // skip port#
            thisargisL("*");
            char *rxdata = comm_cmdbuf_next_arg(&fromLora);
            if (rxdata[0] != '\0') {
                process_rx(rxdata);
            } else {
                // If we get an empty reply, try up to one more time
                // to see if it happens to come back to us on that xmit
                if (xmitReplyRetriesLeft > 0) {
                    int saveRetriesLeft = xmitReplyRetriesLeft;
                    setstateL(COMM_STATE_IDLE);
                    send_ping_to_service(REPLY_NONE);
                    // send_ping_to_service resets xmitReplyRetriesLeft, so
                    // here we restore and decrement it.
                    xmitReplyRetriesLeft = saveRetriesLeft - 1;
                } else {
                    setidlestateL();
                }
            }
        } else {
            DEBUG_PRINTF("tx2 reply ?? %s\n", &fromLora.buffer[fromLora.args]);
            // Record this as an error because it means that something the caller
            // thought was transmitted silently got dropped.
            stats()->errors_lora++;
            setidlestateL();
        }
        if (loraInitEverCompleted && !awaitingTTServeReply)
            comm_oneshot_completed();
        break;
    }

    case COMM_LORA_RXRPL: {
        if (thisargisL("ok")) {
            // this is expected response from initiating the rcv,
            // so just reset the buffer and keep waiting for a message to come in
            setstateL(COMM_LORA_RXRPL);
        } else if (thisargisL("radio_err")) {
            // Re-enable serial output now that it's safe to do so
            serial_transmit_enable(true);
            // We're done waiting for reply
            if (awaitingTTServeReply)
                comm_oneshot_completed();
            // Cancel any pending ttserve reply; it didn't come.
            awaitingTTServeReply = false;
            // Expected from receive timeout of WDT seconds.
            // Note that if we were awaiting a ping reply, we need to stop now
            // because we didn't receive it.
            if (awaitingTTGateReply) {
                awaitingTTGateReply = false;
                // Try one more time, because waiting for a single reply can be fragile
                if (xmitReplyRetriesLeft > 0) {
                    int saveRetriesLeft = xmitReplyRetriesLeft;
                    setstateL(COMM_STATE_IDLE);
                    DEBUG_PRINTF("Retrying ping to Lora gateway\n");
                    send_ping_to_service(REPLY_TTGATE);
                    // send_ping_to_service resets xmitReplyRetriesLeft, so
                    // here we restore and decrement it.
                    xmitReplyRetriesLeft = saveRetriesLeft - 1;
                } else {
                    // Either try the other mode, or try this one again.
                    if (LoRaWAN_mode_try_the_other_on_failure)
                        processstateL(COMM_LORA_LORAWANREQ);
                    else
                        processstateL(COMM_LORA_LORAREQ);
                }
            } else {
                // If there's a pending outbound, transmit it (which will change state)
                // else restart the receive.
                if (!sent_pending_outbound())
                    restart_receive();
            }
        } else if (thisargisL("busy")) {
            // Re-enable serial output now that it's safe to do so
            serial_transmit_enable(true);
            // This is not at all expected, but it means that we're
            // moving too quickly and we should try again.
            nrf_delay_ms(MICROCHIP_LONG_DELAY_MS);
            restart_receive();
        } else if (thisargisL("radio_rx")) {
            // Re-enable serial output now that it's safe to do so
            serial_transmit_enable(true);
            comm_cmdbuf_next_arg(&fromLora);
            process_rx((char *) &fromLora.buffer[fromLora.args]);
            // We're done waiting for reply
            comm_oneshot_completed();
        } else {
            // Re-enable serial output now that it's safe to do so
            serial_transmit_enable(true);
            // Totally unknown error, but since we cannot just
            // leave things in a state without a pending receive,
            // we need to just restart it
            DEBUG_PRINTF("Receive error!\n");
            restart_receive();
        }
        // We may be leaving this state in the same state as we came in,
        // so make sure that we reset the watchdog timer so that we don/t
        // inadvertently confuse this with zero activity.
        lora_watchdog_reset();
        break;
    }

    case  COMM_LORA_UNSOLICITED: {
        // Ignore spurious OK that may come in after sleep reply;
        // see cmd_clear_lpwan_sleep_state for more info.
        // Also ignore the reply from a sys get ver
        if (!thisargisL("ok"))
            if (fromLora.buffer[0] != 'R' && fromLora.buffer[0] != 'N')
                DEBUG_PRINTF("lpwan ?? %s\n", &fromLora.buffer[fromLora.args]);
        setidlestateL();
        break;
    }

    } // switch

} // lora_process()

#endif // LORA
