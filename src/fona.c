// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

// CELL state machine processing

#ifdef FONA

// 2017-05-24 during debugging of MCU crashes during fona
#define TRYNODELAY

#include <stdio.h>
#include <stdlib.h>
#include "debug.h"
#include "config.h"
#include "comm.h"
#include "fona.h"
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
#include "custom_board.h"
#include "pb_encode.h"
#include "pb_decode.h"

// Use TCP instead of HTTP for confirmed transactions
#define USETCP true

// Hard-wired file names
#define DFU_INFO_PACKET "dfu.dat"
#define DFU_FIRMWARE    "dfu.bin"

// Device states
#define COMM_FONA_ECHORPL               COMM_STATE_DEVICE_START+0
#define COMM_FONA_INITCOMPLETED         COMM_STATE_DEVICE_START+1
#define COMM_FONA_RESETREQ              COMM_STATE_DEVICE_START+2
#define COMM_FONA_CRESETRPL             COMM_STATE_DEVICE_START+3
#define COMM_FONA_CPSIRPL               COMM_STATE_DEVICE_START+4
#define COMM_FONA_CGSOCKCONTRPL         COMM_STATE_DEVICE_START+5
#define COMM_FONA_CSOCKSETPNRPL         COMM_STATE_DEVICE_START+6
#define COMM_FONA_CIPMODERPL            COMM_STATE_DEVICE_START+7
#define COMM_FONA_NETOPENRPL            COMM_STATE_DEVICE_START+8
#define COMM_FONA_CIPOPENRPL            COMM_STATE_DEVICE_START+9
#define COMM_FONA_CHTTPSSTARTRPL        COMM_STATE_DEVICE_START+10
#define COMM_FONA_CHTTPSOPSERPL         COMM_STATE_DEVICE_START+11
#define COMM_FONA_CHTTPSSENDRPL         COMM_STATE_DEVICE_START+12
#define COMM_FONA_CHTTPSSEND2RPL        COMM_STATE_DEVICE_START+13
#define COMM_FONA_CHTTPSRECVRPL         COMM_STATE_DEVICE_START+14
#define COMM_FONA_CHTTPSCLSERPL         COMM_STATE_DEVICE_START+15
#define COMM_FONA_CGFUNCRPL1            COMM_STATE_DEVICE_START+16
#define COMM_FONA_CGFUNCRPL2            COMM_STATE_DEVICE_START+17
#define COMM_FONA_CGPSINFO3RPL          COMM_STATE_DEVICE_START+18
#define COMM_FONA_IFCRPL2               COMM_STATE_DEVICE_START+19
#define COMM_FONA_CGPSRPL               COMM_STATE_DEVICE_START+20
#define COMM_FONA_CGPSINFORPL           COMM_STATE_DEVICE_START+21
#define COMM_FONA_CPINRPL               COMM_STATE_DEVICE_START+22
#define COMM_FONA_CGPSINFO2RPL          COMM_STATE_DEVICE_START+23
#define COMM_FONA_MISCRPL               COMM_STATE_DEVICE_START+24
#define COMM_FONA_STARTRPL              COMM_STATE_DEVICE_START+25
#define COMM_FONA_ECHORPL2              COMM_STATE_DEVICE_START+26
#define COMM_FONA_CICCIDRPL             COMM_STATE_DEVICE_START+27
#define COMM_FONA_CPSI0RPL              COMM_STATE_DEVICE_START+28
#define COMM_FONA_CDNSGIPRPL            COMM_STATE_DEVICE_START+29
#define COMM_FONA_DFUBEGIN              COMM_STATE_DEVICE_START+30
#define COMM_FONA_DFURPL0               COMM_STATE_DEVICE_START+31
#define COMM_FONA_DFURPL1               COMM_STATE_DEVICE_START+32
#define COMM_FONA_DFURPL2               COMM_STATE_DEVICE_START+33
#define COMM_FONA_DFURPL3               COMM_STATE_DEVICE_START+34
#define COMM_FONA_DFURPL4               COMM_STATE_DEVICE_START+35
#define COMM_FONA_DFURPL4A              COMM_STATE_DEVICE_START+36
#define COMM_FONA_DFURPL5               COMM_STATE_DEVICE_START+37
#define COMM_FONA_DFURPL5A              COMM_STATE_DEVICE_START+38
#define COMM_FONA_DFURPL6               COMM_STATE_DEVICE_START+39
#define COMM_FONA_DFURPL7               COMM_STATE_DEVICE_START+40
#define COMM_FONA_DFURPL8               COMM_STATE_DEVICE_START+41
#define COMM_FONA_DFURPL9               COMM_STATE_DEVICE_START+42
#define COMM_FONA_DFUVALIDATE           COMM_STATE_DEVICE_START+43
#define COMM_FONA_DFUPREPARE            COMM_STATE_DEVICE_START+44
#define COMM_FONA_NOLED1                COMM_STATE_DEVICE_START+45
#define COMM_FONA_NOLED2                COMM_STATE_DEVICE_START+46
#define COMM_FONA_ATIRPL                COMM_STATE_DEVICE_START+47
#define COMM_FONA_CDNSGIPRPL2           COMM_STATE_DEVICE_START+48
#define COMM_FONA_CIPOPENRPL2           COMM_STATE_DEVICE_START+49
#define COMM_FONA_CIPHEADRPL            COMM_STATE_DEVICE_START+50
#define COMM_FONA_CIPSRIPRPL            COMM_STATE_DEVICE_START+51
#define COMM_FONA_CIPRXGETRPL           COMM_STATE_DEVICE_START+52
#define COMM_FONA_CIPRXGETRPL2          COMM_STATE_DEVICE_START+53
#define COMM_FONA_CIPTIMEOUTRPL         COMM_STATE_DEVICE_START+54
#define COMM_FONA_CIPSENDRPL            COMM_STATE_DEVICE_START+55
#define COMM_FONA_CIPCLOSERPL           COMM_STATE_DEVICE_START+56

// Command buffer
static cmdbuf_t fromFona;

// The Fona module cannot, by design, send more than 1500 bytes.  We choose this
// number to be close but comfortably below that number.
#define FONA_MTU 1480

// Buffers for sending/receiving
static uint8_t deferred_iobuf[FONA_MTU+256];
static uint16_t deferred_iobuf_length;
static uint16_t deferred_request_type;
static uint32_t deferred_active = 0;
static bool deferred_done_after_callback = false;
static bool deferred_callback_requested = false;

// APN
static char apn[64] = "";

// DNS-resolved service address
static uint32_t service_last_dns_lookup = 0L;
static char service_udp_ipv4[32] = "";
static char service_tcp_ipv4[32] = "";

// IP
static uint16_t ip_open_retries;

// GPS context
#ifdef FONAGPS
static bool gpsShutdown = false;
static bool gpsSendShutdownCommandWhenIdle = false;
static bool gpsHaveLocation = false;
static bool gpsUpdateLocation = false;
static bool gpsDataParsed = false;
static float gpsLatitude;
static float gpsLongitude;
static float gpsAltitude;
#endif

// Initialization and fault-related
static bool fonaForceFullHardwareReset = true;
static bool fonaFirstResetAfterInit = false;
static bool fonaLock = false;
static uint32_t watchdog_set_time;
static bool watchdog_extend = false;
static bool fonaInitCompleted = false;
static bool fonaInitInProgress = false;
static bool fonaDFUInProgress = false;
static uint32_t fonaInitLastInitiated = 0L;
static bool fonaNoNetwork = false;
static uint32_t fona_received_since_powerup = 0;
static bool atiMode = false;
static bool fRecordingStats = false;

// DFU state management
static uint32_t dfu_total_packets = 0;
static uint32_t dfu_total_length = 0;
static uint32_t dfu_last_message_length = 0;
static uint16_t getfile_retries;

// Request/reply state management
static bool awaitingTTServeReply = false;

// Get MTU
uint16_t fona_get_mtu() {
    return FONA_MTU;
}

// Transmit the command to the cellular modem
void fona_send(char *msg) {

    if (!comm_is_initialized())
        return;

    // Defensive programming, to cover spurious app_sched events occurring after module power-down
    if (gpio_current_uart() != UART_FONA)
        return;

    if (debug(DBG_TX))
        DEBUG_PRINTF("> %s\n", msg);

    // Send it
    while (*msg != '\0')
        serial_send_byte(*msg++);

    // Send terminating CR
    serial_send_byte('\r');

}

// Check what's in the command buffer
bool thisargisF(char *what) {
    return (comm_cmdbuf_this_arg_is(&fromFona, what));
}

// Quick wrapper to move to next arg
char *nextargF() {
    return(comm_cmdbuf_next_arg(&fromFona));
}

// Set a reply type as having been identified
void seenF(uint32_t mask) {
    fromFona.recognized |= mask;
}

// Set a reply type as having been identified
bool allwereseenF(uint32_t mask) {
    if ((fromFona.recognized & mask) == mask)
        return true;
    return false;
}

// Set modem to the specified state
void setstateF(uint16_t newstate) {
    comm_cmdbuf_set_state(&fromFona, newstate);
}

// Set state to idle
void setidlestateF() {
    setstateF(COMM_STATE_IDLE);
}

// Synchronously process a new state (doing so "nested" if calling from cmdbuf_process)
// This is tremendously convenient, as opposed to introducing new intermediate states.
void processstateF(uint16_t newstate) {
    setstateF(newstate);
    fromFona.complete = true;
    fona_process();
}

// Check to see if we received what we regard as a bad reply universally
bool commonreplyF() {

    // Handle an error on any command by resetting the device.  If
    // you need to do special per-state handling of "error", just do so
    // before calling commonreplyF.
    if (thisargisF("error")) {
        DEBUG_PRINTF("ERROR(%d)\n", fromFona.state);
        // We need to do a hardware reset to close currently open sessions
        fonaForceFullHardwareReset = true;
        processstateF(COMM_FONA_RESETREQ);
        return true;
    }

    // Handle an unexpected reset that may have occurred because of
    // power supply issues, etc.
    if (thisargisF("start") && fromFona.state != COMM_STATE_IDLE) {
        DEBUG_PRINTF("** SPONTANEOUS RESET in state %d **\n", fromFona.state);
        if (fRecordingStats) {
            if (fromFona.state == COMM_FONA_CPSIRPL)
                stats()->ant_fails++;
            else
                stats()->power_fails++;
            stats()->errors_fona++;
        }
        processstateF(COMM_FONA_STARTRPL);
        return(true);
    }

    // Process incoming TCP/IP data
    if (thisargisF("+ipd*")) {
        char command[64];
        nextargF();
        thisargisF("*");
        int len = atoi(nextargF());
        if (len > CMD_MAX_LINELENGTH)
            len = CMD_MAX_LINELENGTH;;
        deferred_iobuf_length = 0;
        sprintf(command, "at+ciprxget=3,1,%d", len);
        fona_send(command);
        setstateF(COMM_FONA_CIPRXGETRPL2);
        return(true);
    }

    // Handle stateless error conditions
    if (thisargisF("+ciperror:")) {
        nextargF();
        DEBUG_PRINTF("CIPERROR(%d) %s\n", fromFona.state, &fromFona.buffer[fromFona.args]);
        // We need to do a hardware reset to close currently open sessions
        fonaForceFullHardwareReset = true;
        processstateF(COMM_FONA_RESETREQ);
        return true;
    }

    // Handle unrecoverable SIM errors
    if (thisargisF("+cme")) {
        nextargF();
        if (thisargisF("error:")) {
            nextargF();
            if (memcmp(&fromFona.buffer[fromFona.args], "SIM failure", strlen("SIM failure")) == 0) {
                fonaNoNetwork = true;
            } else if (memcmp(&fromFona.buffer[fromFona.args], "SIM busy", strlen("SIM busy")) == 0) {
                // Ignore
            } else {
                DEBUG_PRINTF("CME ERROR(%d) '%s'\n", fromFona.state, &fromFona.buffer[fromFona.args]);
            }
        }
        return true;
    }

    // Map SIM ICCID to APN
    // https://en.wikipedia.org/wiki/Subscriber_identity_module
    if (thisargisF("+iccid:")) {
        nextargF();
        // Save it for stats purposes
        char *iccid = (char *)&fromFona.buffer[fromFona.args];
        char *carrier = "";
        strlcpy(stats()->cell_iccid, iccid, sizeof(stats()->cell_iccid)-1);
        // Twilio US
        if (memcmp(iccid, "890126", 6) == 0) {
            carrier = "Twilio Beta";
            strcpy(apn, "wireless.twilio.com");
        }
        // Soracom Global Beta
        if (memcmp(iccid, "891030", 6) == 0) {
            carrier = "Soracom Beta";
            strcpy(apn, "openroamer.com");
        }
        // Soracom Global Production
        if (memcmp(iccid, "894230", 6) == 0) {
            carrier = "Soracom";
            strcpy(apn, "soracom.io");
        }
        // AT&T IoT Global via Contract, assigned to
        // Safecast by Ann Tomasello 2017-02-27
        // with Ray being POC
        if (memcmp(iccid, "890117", 6) == 0) {
            carrier = "AT&T";
            strcpy(apn, "m2m005267.attz");
        }
        // Unrecognized
        if (apn[0] == '\0')
            DEBUG_PRINTF("Can't set APN; unrecognized SIM ICCID: '%s'\n", iccid);
        else {
            DEBUG_PRINTF("SIM ICCID: %s %s\n", carrier, iccid);
        }
        return true;
    }

    // Process incoming gps info reports
#ifdef FONAGPS
    if (thisargisF("+cgpsinfo:*")) {

        // Indicate that we've got some data
        gpio_indicate(INDICATE_GPS_CONNECTING);

        // Parse the arguments
        gpsDataParsed = true;
        nextargF();
        thisargisF("*");
        char *lat = nextargF();
        thisargisF("*");
        char *latNS = nextargF();
        thisargisF("*");
        char *lon = nextargF();
        thisargisF("*");
        char *lonEW = nextargF();
        thisargisF("*");
        char *utcDate = nextargF();
        thisargisF("*");
        char *utcTime = nextargF();
        thisargisF("*");
        char *alt = nextargF();
        if (utcDate[0] != '\0' && utcTime[0] != '\0')
            set_timestamp(atol(utcDate), atol(utcTime));
        if (lat[0] != '\0' && lon[0] != '\0' && alt[0] != '\0') {
            gpsLatitude = GpsEncodingToDegrees(lat, latNS);
            gpsLongitude = GpsEncodingToDegrees(lon, lonEW);
            gpsAltitude = atof(alt);
            // Altitude seems to be reported as negative, even when well above sea level
            if (gpsAltitude < 0)
                gpsAltitude = -gpsAltitude;
            // Save a last known good location, but only on the first good acquisition.
            if (!gpsHaveLocation || gpsUpdateLocation) {
                STORAGE *f = storage();
                f->lkg_gps_latitude = gpsLatitude;
                f->lkg_gps_longitude = gpsLongitude;
                f->lkg_gps_altitude = gpsAltitude;
                storage_save(false);
            }
            // We've now got it
            gpsHaveLocation = true;
            gpsUpdateLocation = false;
        } else {
            // This is for debugging while on bluetooth, enabling us
            // to see all the sentences being generated.  Note that you
            // should NOT run this when not debugging, because it occasionally
            // causes a "busy buffer overflow" because of the unexpected
            // incoming flood of data at a time when we are expecting small
            // responses from things we transmit to the service.
            if (debug(DBG_GPS_MAX)) {
                static bool fSentOnce = false;
                if (!fSentOnce && fromFona.state == COMM_STATE_IDLE) {
                    fSentOnce = true;
                    fona_send("at+cgpsinfocfg=10,31");
                }
            }
        }

        // If GPS data is now available,
        // shut down all the GPS units as a side-effect
        comm_gps_get_value(NULL, NULL, NULL);

        return true;

    }
#endif  // FONAGPS

    return false;
}

// Reset our watchdog timer
void fona_watchdog_reset() {
    watchdog_set_time = get_seconds_since_boot();
}

// Return true if we're initialized
bool fona_can_send_to_service() {

    // If we have no network, we can't send to the service.  This is quite important
    // because without it our "auto hangup" timer in comm.c won't function.
    if (fonaNoNetwork)
        return false;

    return(fonaInitCompleted);
}

// Return true if transmitting would be pointless
bool fona_is_busy() {

    // Exit if we're in the middle of sending a different message,
    // This can happen because transmits take a finite amount of time,
    // and we might be being asked to transmit both for the local
    // geiger counter as well as the URSC-connected bGeigie.
    if (fromFona.state != COMM_STATE_IDLE) {

#ifdef FLOWTRACE
        DEBUG_PRINTF("(Can't send - busy %d!)\n", fromFona.state);
#endif

        return true;
    }

    // If a transmit is in-progress, the state might be idle when
    // we are awaiting the next phase of the transmit process.
    // Indicate that we're busy so that we don't try to come back
    // and transmit something else.
    if (deferred_active != 0) {

        // If it's been active for too long, this is an indication that
        // either the fona module is misbehaving or there was some uart issue.
        // So as defensively not to get "stuck", this will abort if it is
        // active for far, far too long.
        if (!ShouldSuppress(&deferred_active, 60)) {
            deferred_active = 0;
            DEBUG_PRINTF("Warning: Fona transmit timeout!\n");
            return false;
        }

        // We are still waiting for transmit to complete
        return true;

    }

#ifdef FONAGPS
    // Exit if we're waiting to shut off the GPS
    if (gpsSendShutdownCommandWhenIdle)
        return true;

    // Exit if we're waiting to update the location.  If we've been
    // waiting too long, though, just give up.
    if (gpsUpdateLocation) {
        if (ShouldSuppress(&fonaInitLastInitiated, gpsHaveLocation ? GPS_ABORT_IMPROVE_MINUTES*60L : GPS_ABORT_FIRST_MINUTES*60L))
            return true;
        gpsUpdateLocation = false;
    }
#endif

    // Not busy
    return false;
}

// Transmit a well-formed protocol buffer to the LPWAN as a message
bool fona_send_to_service(uint8_t *buffer, uint16_t length, uint16_t RequestType) {
    char command[64];

    // Exit if we're not yet initialized
    if (!comm_is_initialized())
        return false;

    // If we're busy doing something else, drop this
    if (fona_is_busy())
        return false;

    // If there's already a deferred command, drop this
    if (deferred_active != 0)
        return false;

    // If this is too long for buffers, exit
    if (length > sizeof(deferred_iobuf))
        return false;

    // Now that we're committed, if this is a request that requires a reply, remember that we're doing so.
    awaitingTTServeReply = (RequestType != REPLY_NONE);

    // Set up the deferred data
    deferred_active = get_seconds_since_boot();
    deferred_iobuf_length = length;
    memcpy(deferred_iobuf, buffer, length);
    deferred_request_type = RequestType;

    // If this is a transmit-only request, do it via UDP, else TCP/HTTP
    if (deferred_request_type == REPLY_NONE) {

#ifdef FLOWTRACE
        DEBUG_PRINTF("UDP send: %d\n", length);
#endif

        // Bump stats about what we've transmitted
        stats_io(length, 0);

        // Transmit it, expecting the deferred handler to finish this.
        deferred_callback_requested = true;
        deferred_done_after_callback = true;
        sprintf(command, "at+cipsend=0,%u,\"%s\",%u", deferred_iobuf_length, service_udp_ipv4, SERVICE_UDP_PORT);
        fona_send(command);
        setstateF(COMM_FONA_MISCRPL);

    } else {

#if USETCP

#ifdef FLOWTRACE
        DEBUG_PRINTF("TCP send: %d\n", length);
#endif

        // Bump stats about what we've transmitted
        stats_io(length, 0);

        // Transmit it, after which the "cipsend" will process the deferred iobuf
        ip_open_retries = 8;
        comm_set_connect_state(CONNECT_STATE_APP_SERVICE);
        sprintf(command, "at+cipopen=1,\"TCP\",\"%s\",%u", service_tcp_ipv4, SERVICE_TCP_PORT);
        fona_send(command);
        setstateF(COMM_FONA_CIPOPENRPL2);

#else

#ifdef FLOWTRACE
        DEBUG_PRINTF("HTTP send: %d\n", length);
#endif

        // Transmit it, expecting to receive a callback at fona_http_start_send() after
        // the session is open.
        sprintf(command, "at+chttpsopse=\"%s\",%u,1", SERVICE_HTTP_ADDRESS, SERVICE_HTTP_PORT);
        fona_send(command);
        setstateF(COMM_FONA_CHTTPSOPSERPL);

#endif // USETCP

    }

    // Done
    return true;
}

// Initiate the HTTP send now that the session is open.
// Note that this REPLACES the contents of deferred_iobuf with a hexified version of same.
#if !USETCP
void fona_http_start_send() {
    char command[64];
    char hiChar, loChar, body[sizeof(deferred_iobuf)*2+50+1];
    uint16_t i, header_length, hexified_length, total_length;

    // The hexified data length will be the original data * 2 (because of hexification)
    hexified_length = deferred_iobuf_length * 2;

    // Put together a minimalist HTTP header and command to transmit it
    sprintf(body, "POST %s HTTP/1.1\r\nHost: %s:%d\r\nUser-Agent: TTNODE\r\nContent-Length: %d\r\n\r\n",
            SERVICE_HTTP_TOPIC, SERVICE_HTTP_ADDRESS, SERVICE_HTTP_PORT, hexified_length);

    // Compute the remaining lengths
    header_length = strlen(body);

    // Hexify into the body, and add trailing \r\n, and a null term which is useful for %s printing when debugging
    total_length = header_length;
    for (i=0; i<deferred_iobuf_length; i++) {
        if ( (i + header_length) >= (sizeof(deferred_iobuf) - sizeof("00\r\n")) )
            break;
        HexChars(deferred_iobuf[i], &hiChar, &loChar);
        body[total_length++] = hiChar;
        body[total_length++] = loChar;
    }
    body[total_length] = '\0';

    // Bump stats about what we've transmitted
    stats_io(total_length, 0);

    // Move it back into the iobuf
    deferred_iobuf_length = total_length;
    memcpy(deferred_iobuf, body, deferred_iobuf_length);

    // Generate a command
    deferred_callback_requested = true;
    sprintf(command, "at+chttpssend=%u", deferred_iobuf_length);
    fona_send(command);

}
#endif // !USETCP

// Initiate the HTTP receive into the deferred iobuf
#if !USETCP
void fona_http_start_receive() {
    char command[64];
    deferred_iobuf_length = 0;
    sprintf(command, "at+chttpsrecv=%u", sizeof(deferred_iobuf));
    fona_send(command);
}
#endif // !USETCP

// Append the stuff received IF AND ONLY IF it looks like hexadecimal data
void fona_append_received_hex_data(char *buffer, uint16_t buffer_length) {
    char hiChar, loChar;
    uint8_t databyte;
    int i;

    // First do a quick pass to see if all the chars are hex decodable, ignoring whitespace
    for (i=0; i<buffer_length;) {
        hiChar = buffer[i++];
        if (hiChar > ' ') {
            if (i>=buffer_length)
                break;
            loChar = buffer[i++];
            if (!HexValue(hiChar, loChar, NULL)) {
                return;
            }
        }
    }

    // Append all the non-whitespace
    for (i=0; i<buffer_length; i++) {
        databyte = buffer[i];
        if (databyte > ' ') {
            if (deferred_iobuf_length >= sizeof(deferred_iobuf))
                return;
            deferred_iobuf[deferred_iobuf_length++] = databyte;
        }
    }

}

// Process the stuff in the deferred iobuf
void fona_process_received() {
    uint8_t buffer[CMD_MAX_LINELENGTH];
    uint16_t msgtype;

    // Regardless of what we received, indicate that we're no longer waiting for
    // a TTServe reply, because we only want to listen for a single receive
    // window, for power reasons  If we don't pick it up now, we'll pick it up
    // eventually on the next message to the service.
    awaitingTTServeReply = false;

    // Null-terminate the io buffer
    if (deferred_iobuf_length == sizeof(deferred_iobuf))
        deferred_iobuf_length--;
    deferred_iobuf[deferred_iobuf_length] = '\0';

    // Only do this if we got something back
    if (deferred_iobuf_length != 0) {

        // Bump stats about what we've received on the wire
        stats_io(0, deferred_iobuf_length);

        // Decode the message
        msgtype = comm_decode_received_message((char *)deferred_iobuf, NULL, buffer, sizeof(buffer) - 1, NULL);
        if (msgtype != MSG_REPLY_TTSERVE) {
            // This can happen if we get an HTTP error in the body
            deferred_iobuf[deferred_iobuf_length] = '\0';
            DEBUG_PRINTF("?: %s\n", deferred_iobuf);
        } else {

            // Process it
            recv_message_from_service((char *)buffer);

        }

    }

    // We're now done.
    deferred_active = 0;
    comm_oneshot_completed();

}

// Process the remainder of the deferred send
void fona_process_deferred() {
    int i;

    // Defensive programming, to cover spurious app_sched events occurring after module power-down
    if (gpio_current_uart() != UART_FONA)
        return;

    // Transmit deferred stuff
    for (i=0; i<deferred_iobuf_length; i++)
        serial_send_byte(deferred_iobuf[i]);

    // Now inactive, and we're done with the callback
    deferred_callback_requested = false;
    if (deferred_done_after_callback) {
        deferred_active = 0;
        comm_oneshot_completed();
    }
}

// Request that Fona be shut down, with control being transferred as appropriate
void fona_shutdown() {

    // If we've been given no alternative, just shut down all comms
    if (storage()->wan == WAN_FONA || storage()->wan == WAN_FONA_PLUS_MOBILE) {
        comm_deselect("fona shutdown");
        return;
    }

    // If we have LORA around, transfer control to it
#if defined(LORA) && defined(FONAGPS)
    if (storage()->wan == WAN_AUTO)
        comm_select(COMM_LORA, "fona shutdown");
#endif

    // Deselect without switching the active comms mode, in case cell comes back
    comm_deselect("fona shutdown");

}

// Do modem reset processing
bool fona_needed_to_be_reset() {
    uint32_t secondsSinceBoot = get_seconds_since_boot();

#ifdef FONAGPS
    // Check to see if Fona GPS needed to be shut down, and do it if appropriate.
    if (gpsSendShutdownCommandWhenIdle && fromFona.state == COMM_STATE_IDLE) {
        gpsSendShutdownCommandWhenIdle = false;
        fona_send("at+cgpsinfo=0");
        setstateF(COMM_FONA_CGPSINFO2RPL);
        return true;
    }
#endif

    // Check to see if the Fona card is simply missing or powered off
    if (fona_received_since_powerup == 0 && secondsSinceBoot > BOOT_DELAY_UNTIL_INIT) {
        if (!fonaLock && !fonaInitCompleted && fonaInitInProgress ) {
            DEBUG_PRINTF("CELL is non-responsive.\n");
            fonaNoNetwork = true;
#ifdef FONAGPS
            fona_gps_shutdown();
#endif
            fona_shutdown();
            comm_oneshot_completed();
            return true;
        }
    }

    // Initialize the entire cmd subsystem only after the chip has
    // had a chance to stabilize after boot.
    if (!fonaInitCompleted && !fonaInitInProgress && secondsSinceBoot > BOOT_DELAY_UNTIL_INIT) {
        DEBUG_PRINTF("reset\n");
        fona_reset(false);
        return true;
    }

    // See if we should reset the watchdog because of clock wraparound
    if (watchdog_set_time >= secondsSinceBoot)
        fona_watchdog_reset();

    // Reset all state and the modem if appropriate,
    // handling clock wrap.  Only do this, though, if
    // the modem is in a non-idle state, because we
    // don't want to be performing resets of completely
    // idle devices.
    if (secondsSinceBoot >= CELL_WATCHDOG_SECONDS) {
        uint32_t watchdog_seconds = CELL_WATCHDOG_SECONDS;
        // Extend to 5m for super-long operations such as DFU file transfer
        if (watchdog_extend)
            watchdog_seconds = 300;
        // If we've gone over the watchdog time, reset the world
        if ((secondsSinceBoot - watchdog_set_time) > watchdog_seconds)
            if (fromFona.state != COMM_STATE_IDLE) {
                DEBUG_PRINTF("WATCHDOG: Fona stuck st=%d cc=%d b=%d,%d,%d '%s'\n", fromFona.state, fromFona.complete, fromFona.busy_length, fromFona.busy_nextput, fromFona.busy_nextget, fromFona.buffer);
                // If we're in oneshot mode, use a much bigger stick to reset it, just for good measure
                // This ensures that the uart switch is set appropriately.
                // We need to do a hardware reset to close currently open sessions
                fonaForceFullHardwareReset = true;
                if (!comm_oneshot_currently_enabled())
                    fona_reset(true);
                else {
                    comm_deselect("fona reset");
                    comm_reselect();
                }
                if (fRecordingStats)
                    stats()->errors_fona++;
                return true;
            }
    }

    // Not reset
    return false;

}

// End DFU mode cleanly
void dfu_terminate(uint16_t error) {
    if (fonaDFUInProgress) {
        STORAGE *f = storage();
        f->dfu_status = DFU_IDLE;
        f->dfu_error = error;
        if (error == DFU_ERR_NONE) {
            f->dfu_count++;
            DEBUG_PRINTF("DFU (%lu/%lu) completed successfully\n", dfu_total_packets, dfu_total_length);
        } else {
            DEBUG_PRINTF("DFU (%lu/%lu) error: %d\n", error, dfu_total_packets, dfu_total_length);
        }
        storage_save(true);
        io_request_restart();
    }
    setidlestateF();
}

// Initialize or reinitialize the state machine
void fona_reset(bool force) {

    // If this was a DFU request and we're being killed by the watchdog, terminate DFU
    if (force)
        dfu_terminate(DFU_ERR_RESET);

    // If power is off, force ourselves into an idle state
    if (comm_is_deselected()) {
        fonaInitCompleted = true;
        fonaInitInProgress = false;
        gpio_indicate(INDICATE_COMMS_STATE_UNKNOWN);
        setidlestateF();
        return;
    }

    // Prevent recursion
    if (!force && fonaInitInProgress)
        return;

    // Kick off a device reset
    processstateF(COMM_FONA_RESETREQ);

}

// Request state for debugging
void fona_request_state() {
    DEBUG_PRINTF("Fona %s: st=%d cc=%d b=%d,%d,%d '%s'\n", comm_is_deselected() ? "disconnected" : "connected", fromFona.state, fromFona.complete, fromFona.busy_length, fromFona.busy_nextput, fromFona.busy_nextget, fromFona.buffer);
}

// Request a full hardware reset if there are init issues
void fona_request_full_reset() {
    fonaForceFullHardwareReset = true;
}

// One-time init
void fona_init() {
    comm_cmdbuf_init(&fromFona, CMDBUF_TYPE_FONA);
    comm_cmdbuf_set_state(&fromFona, COMM_FONA_RESETREQ);
    fonaNoNetwork = false;
    deferred_active = 0;
    awaitingTTServeReply = false;
    fonaInitInProgress = false;
    fonaInitCompleted = false;
    fonaFirstResetAfterInit = true;
    fona_received_since_powerup = 0;
    fonaDFUInProgress = (bool) (storage()->dfu_status == DFU_PENDING);
#ifdef FONAGPS
    gpsSendShutdownCommandWhenIdle = false;
    gpsUpdateLocation = false;
#endif

    // Just some defensive coding because there is an interdependency here
    // on two constants defined in different places.  We store one outgoing
    // buffer per DB entry, and so it is optimal if they are the same size.
#if DB_ENABLED
    if (DB_ENTRY_BYTES < FONA_MTU)
        DEBUG_PRINTF("Increase DB_ENTRY_SIZE so it is at least as large as Fona MTU\n");
#endif

}

// Terminate, power down, and set the state of things such that we will look "not busy"
// when we're in a deselected mode.
void fona_term(bool fPowerdown) {
    if (fPowerdown)
        gpio_uart_select(UART_NONE);
    serial_transmit_enable(true);
    deferred_active = 0;
    deferred_callback_requested = false;
    deferred_done_after_callback = false;
    awaitingTTServeReply = false;
    fona_watchdog_reset();
#ifdef FONAGPS
    gpsSendShutdownCommandWhenIdle = false;
#endif
    setidlestateF();
}

// Force GPS to re-aquire itself upon next initialization (ie oneshot)
#ifdef FONAGPS
void fona_gps_update() {
    gpsUpdateLocation = true;
    gpsShutdown = false;
}
#endif

// Process byte received from modem
void fona_received_byte(uint8_t databyte) {
    fona_received_since_powerup++;
    if (deferred_callback_requested && databyte == '>')
        comm_enqueue_complete(CMDBUF_TYPE_FONA_DEFERRED);
    else
        comm_cmdbuf_received_byte(&fromFona, databyte);
}

// Request that the GPS be shut down
#ifdef FONAGPS
void fona_gps_shutdown() {
    if (!gpsShutdown) {
        gpsShutdown = true;
        gpsHaveLocation = true;
        gpsUpdateLocation = false;
        gpio_indicator_no_longer_needed(GPS);
        DEBUG_PRINTF("GPS acquired.\n");
        // This can come even when we're not active, so don't do things that would cause commands to be sent on uart
        if (comm_mode() == COMM_FONA && !comm_is_deselected()) {
            gpsSendShutdownCommandWhenIdle = true;
            // If we've been waiting for the GPS to lock before transferring control, do it now.
            if (!fonaLock && fonaNoNetwork)
                fona_shutdown();
        }
    }
}
#endif

// Get the GPS value if we have it
#ifdef FONAGPS
uint16_t fona_gps_get_value(float *lat, float *lon, float *alt) {
    if (!gpsHaveLocation)
        return (gpsDataParsed ? GPS_NO_LOCATION : GPS_NO_DATA);
    if (lat != NULL)
        *lat = gpsLatitude;
    if (lon != NULL)
        *lon = gpsLongitude;
    if (alt != NULL)
        *alt = gpsAltitude;
    return (GPS_LOCATION_FULL);
}
#endif

// Primary state processing of the command buffer
void fona_process() {

    // If there was a spurious app sched event remaining after powering off the module, ignore it
    if (gpio_current_uart() != UART_FONA)
        return;
            
    // If it's not complete, just exit.
    if (!fromFona.complete)
        return;

    // If extreme debugging
    if (debug(DBG_RX))
        DEBUG_PRINTF("<%d %s\n", fromFona.state, fromFona.buffer);

    // If we've never been initialized and yet we're being called here,
    // just ignore it because we're in an odd state.
    if (fromFona.state != COMM_FONA_RESETREQ && !fonaInitInProgress && !fonaInitCompleted) {
        setidlestateF();
        return;
    }

    // If displaying version info
    if (atiMode && fromFona.buffer[0] != '+' && fromFona.buffer[0] != 'O')
        DEBUG_PRINTF("%s\n", fromFona.buffer);

    //////////////
    ///////
    /////// This state machine is divided into the following blocks
    /////// 1. States associated with initialization
    /////// 2. Steady state
    ///////
    //////////////

    switch (fromFona.state) {

        ///////
        /////// This section of state management is related to initialization
        ///////

    case COMM_FONA_RESETREQ: {
        // If we get here when powered off, force ourselves idle
        if (comm_is_deselected()) {
            fona_reset(true);
            break;
        }
        // Initialize comms
        if (fonaFirstResetAfterInit) {
            fonaFirstResetAfterInit = false;
            DEBUG_PRINTF("CELL initializing\n");
        } else {
            stats()->resets++;
            DEBUG_PRINTF("Cell reset\n");
        }
        fona_watchdog_reset();
        fonaNoNetwork = false;
        fonaInitCompleted = false;
        fonaInitInProgress = true;
        fonaInitLastInitiated = get_seconds_since_boot();
        deferred_active = 0;
        deferred_callback_requested = false;
        deferred_done_after_callback = false;
        awaitingTTServeReply = false;
        serial_transmit_enable(true);
        if (apn[0] == '\0')
            strcpy(apn, storage()->carrier_apn);
        gpio_indicate(INDICATE_CELL_INITIALIZING);
        // Regardless of the state previously, we MUST
        // blast the chip immediately with the desire to
        // turn off flow control.  These settings
        // are persistent across chip resets, and it
        // is critical that they both be done before the
        // reset so that it will come out of the reset
        // in a mode where we can communicate with it
        // regardless of the hardware flow control config.
        fona_send("at+cgfunc=11,0");
        setstateF(COMM_FONA_CGFUNCRPL1);
        break;
    }

    case COMM_FONA_CGFUNCRPL1: {
        // Save time and energy if we just powered on
        // the module by skipping a high-overhead reset.
        // This is necessary, however, if triggered
        // by a subsequent watchdog operation.
        // We only do this, however, if we already have
        // the location, because we've observed that we
        // need to do a creset to set the GPS function
        // to stabilize, as evidenced by an ERROR(120)
        // resulting from the very first at+cgps=1
        bool fDoOptimizedReset = false;
#ifdef FONAGPS
        gpsSendShutdownCommandWhenIdle = false;
        if (gpsHaveLocation && !gpsUpdateLocation)
            if (fonaDFUInProgress || !fonaForceFullHardwareReset)
                fDoOptimizedReset = true;
#else
        if (fonaDFUInProgress || !fonaForceFullHardwareReset)
            fDoOptimizedReset = true;
#endif
        fonaForceFullHardwareReset = false;
        if (fDoOptimizedReset) {
            fona_send("ate0");
            setstateF(COMM_FONA_ECHORPL);
        } else {
            DEBUG_PRINTF("Fona: full reset\n");
            fona_send("at+creset");
            setstateF(COMM_FONA_CRESETRPL);
        }
        break;
    }

    case COMM_FONA_CRESETRPL: {
        // Process start up-front, else we'd go recursive because of commonreplyF()
        if (thisargisF("start")) {
            seenF(0x01);
            fRecordingStats = true;
        } else if (thisargisF("+cpin: ready"))
            seenF(0x02);
        else if (thisargisF("pb done")) {
            // Wait until 1 second after when we think we're done
            // This seems to be necessary else we get a +CME ERROR: SIM busy
            nrf_delay_ms(1000);
            seenF(0x04);
        } else if (commonreplyF())
            break;
        if (allwereseenF(0x07)) {
            fona_send("ate0");
            setstateF(COMM_FONA_ECHORPL);
        }
        break;
    }

    case COMM_FONA_STARTRPL: {
        fona_send("ate0");
        setstateF(COMM_FONA_ECHORPL);
        break;
    }

    case COMM_FONA_ECHORPL: {
        if (commonreplyF())
            break;
        if (thisargisF("ok")) {
            if (serial_hwfc_enabled()){
                // Enable hardware flow control & reconfigure GPIO
                fona_send("at+cgfunc=11,1");
            } else {
                // Again ensure that flow control is disabled
                fona_send("at+cgfunc=11,0");
            }
            setstateF(COMM_FONA_CGFUNCRPL2);
        }
        break;
    }

    case COMM_FONA_CGFUNCRPL2: {
        if (serial_hwfc_enabled()){
            if (commonreplyF())
                break;
            if (thisargisF("ok")) {
                // Enable hardware flow control & reconfigure GPIO
                fona_send("at+ifc=2,2");
                setstateF(COMM_FONA_IFCRPL2);
            }
        } else {
            // If no hwfc, skip the at+ifc because the ifc=0,0 cmd returns
            // an error if we've not first done an at+cgfunc=11,1
            processstateF(COMM_FONA_IFCRPL2);
        }
        break;
    }

    case COMM_FONA_IFCRPL2: {
        if (commonreplyF())
            break;
        // Disable status LED
        fona_send("at+cgfunc=1,0");
        setstateF(COMM_FONA_NOLED1);
        break;
    }

    case COMM_FONA_NOLED1: {
        if (commonreplyF())
            break;
        // Disable status LED driver
        fona_send("at+cleditst=0,0");
        setstateF(COMM_FONA_NOLED2);
        break;
    }

    case COMM_FONA_NOLED2: {
        if (commonreplyF())
            break;
#ifdef FONAGPS
        // If we don't yet have location, let's get it.
        // Otherwise, let's just jump to verifying carrier connectivity.
        if (!gpsHaveLocation || gpsUpdateLocation) {
            fona_send("at+cgps=1");
            setstateF(COMM_FONA_CGPSRPL);
            break;
        }
#endif
        comm_set_connect_state(CONNECT_STATE_WIRELESS_SERVICE);
        fona_send("at+cpsi=5");
        setstateF(COMM_FONA_CPSIRPL);
        break;
    }

#ifdef FONAGPS
    case COMM_FONA_CGPSRPL: {
        // ignore error reply because it may have been user-enabled via at+cgpsauto=1
        if (thisargisF("error") || thisargisF("ok"))
            seenF(0x01);
        if (commonreplyF())
            break;
        if (allwereseenF(0x01)) {
            fona_send("at+cgpsinfo=10");
            setstateF(COMM_FONA_CGPSINFORPL);
        }
        break;
    }
#endif

#ifdef FONAGPS
    case COMM_FONA_CGPSINFORPL: {
        if (commonreplyF())
            break;
        if (thisargisF("ok"))
            seenF(0x01);
        if (allwereseenF(0x01)) {
            // Settle down from reset.  This appears to be necessary, else
            // we see ourselves getting stuck in this state.
            nrf_delay_ms(750);

            // If we're debugging, force there to be no cell network connectivity
#ifdef FONANOSIM
            fonaNoNetwork = true;
#endif

            // After getting GPS, proceed to connect to the network - or avoid doing so.
            // Note that this path is ONLY taken during the first power-on init, because
            // if we're here during oneshot (or failover) we would have taken the
            // gpsHaveLocation fast path to getting onto the network.
            switch (storage()->wan) {

                // When explicitly requesting Fona, we must connect to network
            case WAN_FONA:
            case WAN_FONA_PLUS_MOBILE:
                break;

                // For Auto mode, we switch away from Fona (to another transport iff it is present)
                // after we get GPS lock. But then, if we ever are instructed to failover
                // because of TTGATE backhaul failure, we will return back to the
                // "cellular oneshot" path - and ultimately will restart to check again
                // to see if the TTGATE backhaul has returned.
            case WAN_AUTO:
#ifdef LORA
                fonaNoNetwork = true;
#endif
                break;

                // For non-Fona modes, we drop Fona after getting a GPS lock
            default:
                fonaNoNetwork = true;
                break;
            }

            // If we want to avoid connecting to the network, we're done.
            if (fonaNoNetwork) {
                processstateF(COMM_FONA_INITCOMPLETED);
                break;
            }

            // Since we will be connecting to the net, Check for presence of a valid SIM card
            fona_send("at+cpin?");
            setstateF(COMM_FONA_CPINRPL);

        }
        break;
    }
#endif // FONAGPS

#ifdef FONAGPS
    case COMM_FONA_CGPSINFO2RPL: {
        if (commonreplyF())
            break;
        if (thisargisF("ok"))
            seenF(0x01);
        if (allwereseenF(0x01)) {
            fona_send("at+cgpsinfocfg=0");
            setstateF(COMM_FONA_CGPSINFO3RPL);
        }
        break;
    }
#endif

#ifdef FONAGPS
    case COMM_FONA_CGPSINFO3RPL: {
        if (commonreplyF())
            break;
        if (thisargisF("ok"))
            seenF(0x01);
        if (allwereseenF(0x01)) {
            fona_send("at+cgps=0");
            setstateF(COMM_FONA_MISCRPL);
        }
        break;
    }
#endif

    case COMM_FONA_CPINRPL: {
        // The commonreplyF handler will get a +CME ERROR if no sim card
        if (commonreplyF()) {
            if (fonaNoNetwork)
                processstateF(COMM_FONA_INITCOMPLETED);
            break;
        }
        if (thisargisF("ok")) {
            comm_set_connect_state(CONNECT_STATE_WIRELESS_SERVICE);
            fona_send("at+cpsi=5");
            setstateF(COMM_FONA_CPSIRPL);
        }
        break;
    }

        // This is the "fast-path" entry point when doing one-shots
    case COMM_FONA_ECHORPL2: {
        comm_set_connect_state(CONNECT_STATE_WIRELESS_SERVICE);
        fona_send("at+cpsi=5");
        setstateF(COMM_FONA_CPSIRPL);
        break;
    }

        // This is the loop that we won't get past until
        // we connect to the cell carrier.  We do, however,
        // abort everything from here on if we don't have
        // a SIM card inserted.
    case COMM_FONA_CPSIRPL: {
        bool retry = false;
        if (commonreplyF())
            break;
        if (fonaNoNetwork) {
            processstateF(COMM_FONA_INITCOMPLETED);
            break;
        }
        if (thisargisF("ok")) {
            seenF(0x01);
        } else if (thisargisF("+cpsi:")) {
            nextargF();
            // See if it's something we recognize
            if (thisargisF("no service")) {
                uint32_t seconds_waiting = get_seconds_since_boot()-fonaInitLastInitiated;
                if (seconds_waiting < CELL_SERVICE_SECONDS) {
                    DEBUG_PRINTF("CELL wait for service (%lds)\n", seconds_waiting);
                    retry = true;
                } else {
                    gpio_indicate(INDICATE_CELL_NO_SERVICE);
                    DEBUG_PRINTF("Cannot acquire service (%lds)\n", seconds_waiting);
                    fonaNoNetwork = true;
                    processstateF(COMM_FONA_INITCOMPLETED);
                }
            } else {
                // Skip over WCDMA or GSM
                thisargisF("*");
                char *sysmode = nextargF();
                if (thisargisF("online")) {
                    seenF(0x02);
                    thisargisF("*");
                    nextargF();
                    thisargisF("*");
                    char *mcc = nextargF();
                    thisargisF("*");
                    char *mnc = nextargF();
                    thisargisF("*");
                    char *lac = nextargF();
                    thisargisF("*");
                    char *cellid = nextargF();
                    char buff[128];
                    sprintf(buff, "%s,%s,%s,%s,%s", sysmode, mcc, mnc, lac, cellid);
                    strlcpy(stats()->cell_cpsi, buff, sizeof(stats()->cell_cpsi)-1);
                } else
                    retry = true;
            }
        }
        if (allwereseenF(0x03)) {
            gpio_indicate(INDICATE_CELL_INITIALIZING);
            fona_send("at+cpsi=0");
            setstateF(COMM_FONA_CPSI0RPL);
        } else if (retry) {
            if (comm_is_deselected()) {
                fona_reset(true);
            } else {
                fona_watchdog_reset();
            }
        }
        break;
    }

    case COMM_FONA_CPSI0RPL: {
        if (commonreplyF())
            break;
        if (thisargisF("ok")) {
            if (apn[0] != '\0')
                processstateF(COMM_FONA_CICCIDRPL);
            else {
                atiMode = true;
                fona_send("ati");
                setstateF(COMM_FONA_ATIRPL);
            }
        }
        break;
    }

    case COMM_FONA_ATIRPL: {
        if (thisargisF("ok")) {
            atiMode = false;
            fona_send("at+ciccid");
            setstateF(COMM_FONA_CICCIDRPL);
        }
#define modstr "Model: "
        if (memcmp(&fromFona.buffer[fromFona.args], modstr, strlen(modstr)) == 0)
            strlcpy(stats()->module_fona, (char *) &fromFona.buffer[fromFona.args + strlen(modstr)], sizeof(stats()->module_fona)-1);
        break;
    }

    case COMM_FONA_CICCIDRPL: {
        if (commonreplyF())
            break;
        if (apn[0] == '\0')
            break;
        char buffer[128];
        sprintf(buffer, "at+cgsockcont=1,\"IP\",\"%s\"", apn);
        fona_send(buffer);
        setstateF(COMM_FONA_CGSOCKCONTRPL);
        break;
    }

    case COMM_FONA_CGSOCKCONTRPL: {
        if (commonreplyF())
            break;
        if (thisargisF("ok"))
            seenF(0x01);
        if (allwereseenF(0x01)) {
            fona_send("at+csocksetpn=1");
            setstateF(COMM_FONA_CSOCKSETPNRPL);
        }
        break;
    }

    case COMM_FONA_CSOCKSETPNRPL: {
        if (commonreplyF())
            break;
        if (thisargisF("ok"))
            seenF(0x01);
        if (allwereseenF(0x01)) {
            fona_send("at+cipmode=0");
            setstateF(COMM_FONA_CIPMODERPL);
        }
        break;
    }

    case COMM_FONA_CIPMODERPL: {
        if (commonreplyF())
            break;
        if (thisargisF("ok"))
            seenF(0x01);
        if (allwereseenF(0x01)) {
            fona_send("at+ciptimeout=120000,30000,120000");
            setstateF(COMM_FONA_CIPTIMEOUTRPL);
        }
        break;
    }

    case COMM_FONA_CIPTIMEOUTRPL: {
        if (commonreplyF())
            break;
        if (thisargisF("ok"))
            seenF(0x01);
        if (allwereseenF(0x01)) {
            comm_set_connect_state(CONNECT_STATE_DATA_SERVICE);
            watchdog_extend = true;
            fona_send("at+netopen");
            setstateF(COMM_FONA_NETOPENRPL);
        }
        break;
    }

    case COMM_FONA_NETOPENRPL: {
        if (commonreplyF())
            break;
        if (thisargisF("ok"))
            seenF(0x01);
        // Valid traversal of APN?
        if (thisargisF("+netopen: 0"))
            seenF(0x02);
        // Invalid traversal of APN?
        if (thisargisF("+netopen: 1")) {
            watchdog_extend = false;
            gpio_indicate(INDICATE_CELL_NO_SERVICE);
            DEBUG_PRINTF("Waiting for network...\n");
            // This delay is necessary, because when data service
            // is unavailable we come through this loop EXTREMELY
            // quickly over and over, and we need to give the
            // modem a chance to get us online.
#ifdef TRYNODELAY
            nrf_delay_ms(1500);
#endif
            comm_set_connect_state(CONNECT_STATE_WIRELESS_SERVICE);
            fona_send("at+cpsi=5");
            setstateF(COMM_FONA_CPSIRPL);
            break;
        }
        if (allwereseenF(0x03)) {
            char command[64], *p, ch;
            bool fnonnumeric = false;
            watchdog_extend = false;
            // Refresh DNS just in case load balancer wants us to have a different IP,
            // but don't do it too often because it takes network bandwidth
            if (!ShouldSuppress(&service_last_dns_lookup, CELL_DNS_LOOKUP_INTERVAL_MINUTES*60L)) {
                service_udp_ipv4[0] = '\0';
                service_tcp_ipv4[0] = '\0';
            }
            // If initial case or if refreshing
            if (service_udp_ipv4[0] == '\0')
                strcpy(service_udp_ipv4, SERVICE_UDP_ADDRESS);
            if (service_tcp_ipv4[0] == '\0')
                strcpy(service_tcp_ipv4, SERVICE_TCP_ADDRESS);
            // Don't do DNS lookup if it's already nnn.nnn.nnn.nnn
            for (p=service_udp_ipv4; *p != '\0'; p++) {
                ch = *p;
                if ((ch >= '0' && ch <= '9') || (ch == '.'))
                    continue;
                fnonnumeric = true;
                break;
            }
            for (p=service_tcp_ipv4; *p != '\0'; p++) {
                ch = *p;
                if ((ch >= '0' && ch <= '9') || (ch == '.'))
                    continue;
                fnonnumeric = true;
                break;
            }
            if (fnonnumeric) {
                sprintf(command, "at+cdnsgip=\"%s\"", service_udp_ipv4);
                fona_send(command);
                setstateF(COMM_FONA_CDNSGIPRPL);
            } else {
                fona_send("at+ciphead=1");
                setstateF(COMM_FONA_CIPHEADRPL);
            }
        }
        break;
    }

    case COMM_FONA_CDNSGIPRPL: {
        if (commonreplyF())
            break;
        if (thisargisF("ok"))
            seenF(0x01);
        else if (thisargisF("+cdnsgip: *")) {
            nextargF();
            thisargisF("*");
            char *err = nextargF();
            thisargisF("*");
            char *from = nextargF();
            thisargisF("*");
            char *to = nextargF();
            UNUSED_VARIABLE(from);
            if (atoi(err) == 1) {
                int i = 0;
                char *p = service_udp_ipv4;
                while (i<sizeof(service_udp_ipv4)-1) {
                    if (*to == '\0')
                        break;
                    if (*to != '"') {
                        *p++ = *to;
                        i++;
                    }
                    to++;
                }
                *p = '\0';
            }
        }
        if (allwereseenF(0x01)) {
            char command[64];
            sprintf(command, "at+cdnsgip=\"%s\"", service_tcp_ipv4);
            fona_send(command);
            setstateF(COMM_FONA_CDNSGIPRPL2);
        }
        break;
    }

    case COMM_FONA_CDNSGIPRPL2: {
        if (commonreplyF())
            break;
        if (thisargisF("ok"))
            seenF(0x01);
        else if (thisargisF("+cdnsgip: *")) {
            nextargF();
            thisargisF("*");
            char *err = nextargF();
            thisargisF("*");
            char *from = nextargF();
            thisargisF("*");
            char *to = nextargF();
            UNUSED_VARIABLE(from);
            if (atoi(err) == 1) {
                int i = 0;
                char *p = service_tcp_ipv4;
                while (i<sizeof(service_tcp_ipv4)-1) {
                    if (*to == '\0')
                        break;
                    if (*to != '"') {
                        *p++ = *to;
                        i++;
                    }
                    to++;
                }
                *p = '\0';
            }
        }
        if (allwereseenF(0x01)) {
            fona_send("at+ciphead=1");
            setstateF(COMM_FONA_CIPHEADRPL);
        }
        break;
    }

    case COMM_FONA_CIPHEADRPL: {
        if (commonreplyF())
            break;
        if (thisargisF("ok"))
            seenF(0x01);
        if (allwereseenF(0x01)) {
            fona_send("at+cipsrip=0");
            setstateF(COMM_FONA_CIPSRIPRPL);
        }
        break;
    }

    case COMM_FONA_CIPSRIPRPL: {
        if (commonreplyF())
            break;
        if (thisargisF("ok"))
            seenF(0x01);
        if (allwereseenF(0x01)) {
            fona_send("at+ciprxget=1");
            setstateF(COMM_FONA_CIPRXGETRPL);
        }
    }

    case COMM_FONA_CIPRXGETRPL: {
        if (commonreplyF())
            break;
        if (thisargisF("ok"))
            seenF(0x01);
        if (allwereseenF(0x01)) {
            fona_send("at+cipopen=0,\"UDP\",,,9000");
            setstateF(COMM_FONA_CIPOPENRPL);
        }
    }

    case COMM_FONA_CIPOPENRPL: {
        if (commonreplyF())
            break;
        if (thisargisF("ok"))
            seenF(0x01);
        if (allwereseenF(0x01)) {
#if USETCP
            processstateF(COMM_FONA_INITCOMPLETED);
#else
            fona_send("at+chttpsstart");
            setstateF(COMM_FONA_CHTTPSSTARTRPL);
#endif
        }
        break;
    }

#if !USETCP
    case COMM_FONA_CHTTPSSTARTRPL: {
        if (commonreplyF())
            break;
        if (thisargisF("ok"))
            seenF(0x01);
        if (allwereseenF(0x01)) {
            processstateF(COMM_FONA_INITCOMPLETED);
        }
        break;
    }
#endif

    case COMM_FONA_INITCOMPLETED: {
        // Done with initialization
        if (!fonaNoNetwork) {
            // Success; we're active
            comm_set_connect_state(CONNECT_STATE_FONA_ACTIVE);
            comm_select_completed();
        } else {
            // Failure; the oneshot failed
            fona_shutdown();
            comm_oneshot_completed();
        }
        fonaInitInProgress = false;
        fonaInitCompleted = true;
        setidlestateF();
        // If DFU requested, start to process it
        if (fonaDFUInProgress) {
            if (fonaNoNetwork)
                dfu_terminate(DFU_ERR_NO_NETWORK);
            else
                processstateF(COMM_FONA_DFUBEGIN);
            break;
        }
        // Done
        if (!fonaNoNetwork) {
            gpio_indicate(INDICATE_CELL_CONNECTED);
            DEBUG_PRINTF("CELL online\n");
            // If we DO succeed getting online, lock to
            // the Fona device so that we *never* fall back
            // to Lora in the case of a transient failure.
            fonaLock = true;
            // Initiate a service upload if one is pending
            comm_update_service();
        } else {
#ifdef FONAGPS
            gpio_indicator_no_longer_needed(COMM);
            DEBUG_PRINTF("CELL waiting for GPS\n");
#else
            DEBUG_PRINTF("CELL no network\n");
#endif
        }
        break;
    }

        ///////
        /////// This section of state management is related to a TCP request
        ///////

#if USETCP

    case COMM_FONA_CIPOPENRPL2: {
        if (commonreplyF())
            break;
        // Initial open
        if (thisargisF("ok")) {
            fona_watchdog_reset();
        } else if (thisargisF("+cipopen: 1,0")) {
            seenF(0x01);
        } else if (thisargisF("+cipopen:")) {
            // Retry
            if (ip_open_retries != 0) {
                ip_open_retries--;
                char command[64];
                DEBUG_PRINTF("%s open failure, retrying...\n", service_tcp_ipv4);
                // Fail over to a different server on failure
                strcpy(service_tcp_ipv4, SERVICE_TCP_ADDRESS);
                service_last_dns_lookup = 0;
                // Try again
                sprintf(command, "at+cipopen=1,\"TCP\",\"%s\",%u", service_tcp_ipv4, SERVICE_TCP_PORT);
                comm_set_connect_state(CONNECT_STATE_APP_SERVICE);
                fona_send(command);
                fona_watchdog_reset();
                setstateF(COMM_FONA_CIPOPENRPL2);
            } else {
                DEBUG_PRINTF("%s is unreachable\n", service_tcp_ipv4);
                setidlestateF();
            }
        }
        if (allwereseenF(0x01)) {
            char command[64];
            comm_set_connect_state(CONNECT_STATE_UNKNOWN);
            // Our deferred handler will finish this command
            deferred_callback_requested = true;
            deferred_done_after_callback = true;
            watchdog_extend = true;
            sprintf(command, "at+cipsend=1,%u", deferred_iobuf_length);
            fona_send(command);
            setstateF(COMM_FONA_CIPSENDRPL);
        }
        break;
    }

    case COMM_FONA_CIPSENDRPL: {
        if (thisargisF("error"))
            seenF(0x01);
        else if (thisargisF("ok"))
            seenF(0x01);
        else if (commonreplyF())
            break;
        else if (thisargisF("+ipclose:"))
            seenF(0x02);
        if (allwereseenF(0x03)) {
            watchdog_extend = false;
            fona_send("at+cipclose=1");
            setstateF(COMM_FONA_CIPCLOSERPL);
        }
        break;
    }

    case COMM_FONA_CIPCLOSERPL: {
        if (thisargisF("error"))
            seenF(0x01);
        else if (thisargisF("ok"))
            seenF(0x01);
        else if (commonreplyF())
            break;
        if (allwereseenF(0x01)) {
            setidlestateF();
        }
        break;
    }

    case COMM_FONA_CIPRXGETRPL2: {
        if (commonreplyF())
            break;
        if (thisargisF("ok"))
            break;
        else if (thisargisF("+ciprxget:"))
            break;
        else if (thisargisF("+ipclose:"))
            break;
        else {
            fona_append_received_hex_data((char *)fromFona.buffer, fromFona.length);
            fona_process_received();
            setidlestateF();
        }
        break;
    }

#endif // USETCP

        ///////
        /////// This section of state management is related to an HTTP request
        ///////

#if !USETCP
    case COMM_FONA_CHTTPSOPSERPL: {
        if (commonreplyF())
            break;
        if (thisargisF("ok"))
            seenF(0x01);
        if (allwereseenF(0x01)) {
            fona_http_start_send();
            setstateF(COMM_FONA_CHTTPSSENDRPL);
        }
        break;
    }

    case COMM_FONA_CHTTPSSENDRPL: {
        if (commonreplyF())
            break;
        if (thisargisF("ok"))
            seenF(0x01);
        if (allwereseenF(0x01)) {
            fona_send("at+chttpssend");
            setstateF(COMM_FONA_CHTTPSSEND2RPL);
        }
        break;
    }

    case COMM_FONA_CHTTPSSEND2RPL: {
        if (commonreplyF())
            break;
        if (thisargisF("ok"))
            seenF(0x01);
        else if (thisargisF("+chttps: recv event"))
            seenF(0x02);
        if (allwereseenF(0x03)) {
            fona_http_start_receive();
            setstateF(COMM_FONA_CHTTPSRECVRPL);
        }
        break;
    }

    case COMM_FONA_CHTTPSRECVRPL: {
        if (commonreplyF())
            break;
        if (thisargisF("ok"))
            break;
        if (thisargisF("+chttpsrecv: 0")) {
            fona_process_received();
            fona_send("at+chttpsclse");
            setstateF(COMM_FONA_CHTTPSCLSERPL);
        } else if (thisargisF("+chttpsrecv: data")) {
            break;
        } else {
            fona_append_received_hex_data((char *)fromFona.buffer, fromFona.length);
        }
        break;
    }

    case COMM_FONA_CHTTPSCLSERPL: {
        if (commonreplyF())
            break;
        if (thisargisF("ok"))
            seenF(0x01);
        if (allwereseenF(0x01)) {
            setidlestateF();
        }
        break;
    }
#endif // !USETCP

        ///////
        /////// This section of state management is related to DFU processing
        ///////

    case COMM_FONA_DFUBEGIN: {
        // Remove the "flag" that indicates buttonless DFU
        char buffer[50];
        sprintf(buffer, "at+fsdel=\"%s\"", DFU_INFO_PACKET);
        fona_send(buffer);
        setstateF(COMM_FONA_DFURPL0);
        break;
    }

    case COMM_FONA_DFURPL0: {
        // ERROR is most generally expected because it should NOT exist,
        // so don't do commonreplyF() processing
        char buffer[64];
        sprintf(buffer, "at+cftpserv=\"%s\"", SERVICE_FTP_ADDRESS);
        fona_send(buffer);
        setstateF(COMM_FONA_DFURPL1);
        break;
    }

    case COMM_FONA_DFURPL1: {
        if (commonreplyF()) {
            dfu_terminate(DFU_ERR_BASIC);
            break;
        }
        if (thisargisF("ok"))
            seenF(0x01);
        if (allwereseenF(0x01)) {
            char buffer[64];
            sprintf(buffer, "at+cftpport=%d", SERVICE_FTP_PORT);
            fona_send(buffer);
            setstateF(COMM_FONA_DFURPL2);
        }
        break;
    }

    case COMM_FONA_DFURPL2: {
        if (commonreplyF()) {
            dfu_terminate(DFU_ERR_BASIC);
            break;
        }
        if (thisargisF("ok"))
            seenF(0x01);
        if (allwereseenF(0x01)) {
            char command[64];
            sprintf(command, "at+cftpun=\"%lu\"", io_get_device_address());
            fona_send(command);
            setstateF(COMM_FONA_DFURPL3);
        }
        break;
    }

    case COMM_FONA_DFURPL3: {
        if (commonreplyF()) {
            dfu_terminate(DFU_ERR_BASIC);
            break;
        }
        if (thisargisF("ok"))
            seenF(0x01);
        if (allwereseenF(0x01)) {
            fona_send("at+cftppw=\"safecast-password\"");
            setstateF(COMM_FONA_DFURPL4);
        }
        break;
    }

    case COMM_FONA_DFURPL4: {
        if (commonreplyF()) {
            dfu_terminate(DFU_ERR_BASIC);
            break;
        }
        if (thisargisF("ok"))
            seenF(0x01);
        if (allwereseenF(0x01)) {
#ifdef DFU_TEST_WITHOUT_DOWNLOAD
            DEBUG_PRINTF("DFU Debugging - by passing OTA download\n");
            processstateF(COMM_FONA_DFUVALIDATE);
#else
            char command[64];
            sprintf(command, "at+fsdel=\"%s\"", DFU_INFO_PACKET);
            fona_send(command);
            setstateF(COMM_FONA_DFURPL4A);
#endif
        }
        break;
    }

    case COMM_FONA_DFURPL4A: {
        // Ignore errors on delete
        char command[64];
        sprintf(command, "at+fsdel=\"%s\"", DFU_FIRMWARE);
        fona_send(command);
        setstateF(COMM_FONA_DFURPL5);
        break;
    }

    case COMM_FONA_DFURPL5: {
        // Ignore errors on delete
        DEBUG_PRINTF("DFU downloading %s/%s\n", storage()->dfu_filename, DFU_FIRMWARE);
        char command[64];
        sprintf(command, "at+cftpgetfile=\"/%s/%s\",0", storage()->dfu_filename, DFU_FIRMWARE);
        fona_send(command);
        getfile_retries = 0;
        setstateF(COMM_FONA_DFURPL5A);
        // Disable watchdog because fetching the file takes a LONG time
        watchdog_extend = true;
        break;
    }

    case COMM_FONA_DFURPL5A: {
        if (commonreplyF()) {
            dfu_terminate(DFU_ERR_BASIC);
            watchdog_extend = false;
            break;
        }
        if (thisargisF("+cftpgetfile:")) {
            if (thisargisF("+cftpgetfile: 0")) {
                seenF(0x02);
                DEBUG_PRINTF("DFU downloaded successfully.\n");
            } else {
                // Sometimes the first download fails because there is no FTP session yet established
                if (++getfile_retries <= 10) {
                    char command[64];
                    nrf_delay_ms(500);
                    DEBUG_PRINTF("Retrying DFU download of %s (%s)\n", DFU_FIRMWARE, &fromFona.buffer[fromFona.args]);
                    sprintf(command, "at+cftpgetfile=\"/%s/%s\",0", storage()->dfu_filename, DFU_FIRMWARE);
                    fona_send(command);
                    setstateF(COMM_FONA_DFURPL5A);
                } else {
                    watchdog_extend = false;
                    dfu_terminate(DFU_ERR_GETFILE);
                }
                break;
            }
        }
        if (thisargisF("ok"))
            seenF(0x01);
        if (allwereseenF(0x03)) {
            DEBUG_PRINTF("DFU downloading %s/%s\n", storage()->dfu_filename, DFU_INFO_PACKET);
            char command[64];
            sprintf(command, "at+cftpgetfile=\"/%s/%s\",0", storage()->dfu_filename, DFU_INFO_PACKET);
            fona_send(command);
            setstateF(COMM_FONA_DFURPL6);
            // Disable watchdog because fetching the file takes a LONG time
            watchdog_extend = true;
            break;
        }
        break;

        break;
    }

    case COMM_FONA_DFURPL6: {
        if (commonreplyF()) {
            dfu_terminate(DFU_ERR_BASIC);
            watchdog_extend = false;
            break;
        }
        if (thisargisF("+cftpgetfile:")) {
            if (thisargisF("+cftpgetfile: 0")) {
                seenF(0x02);
                DEBUG_PRINTF("DFU downloaded successfully.\n");
            } else {
                watchdog_extend = false;
                dfu_terminate(DFU_ERR_GETFILE);
                break;
            }
        }
        if (thisargisF("ok"))
            seenF(0x01);
        if (allwereseenF(0x03)) {
            // Done with the download
            watchdog_extend = false;
            // Initiate the validation
            processstateF(COMM_FONA_DFUVALIDATE);
            break;
        }
        break;
    }

    case COMM_FONA_DFUVALIDATE: {
        gpio_indicators_off();
#ifdef DFU_TEST_VALIDATE_DOWNLOAD
        // We'll be updating he indicators manually
        DEBUG_PRINTF("DFU validating download\n");
        // Very importantly, tell the chip to use the UART for the at+cftrantx transfer to follow
        fona_send("at+catr=1");
        setstateF(COMM_FONA_DFURPL7);
        break;
#else
        // Initiate the buttonless DFU
        processstateF(COMM_FONA_DFUPREPARE);
        break;
#endif
    }

    case COMM_FONA_DFURPL7: {
        if (commonreplyF()) {
            dfu_terminate(DFU_ERR_BASIC);
            break;
        }
        if (thisargisF("ok"))
            seenF(0x01);
        if (allwereseenF(0x01)) {
            // So that we can validate what has been downloaded, initiate a transfer to the host
            char command[64];
            sprintf(command, "at+cftrantx=\"c:/%s\"", DFU_FIRMWARE);
            fona_send(command);
            setstateF(COMM_FONA_DFURPL8);
            // Disable watchdog because fetching these results takes a LONG time
            watchdog_extend = true;
        }
        break;
    }

    case COMM_FONA_DFURPL8: {
        if (commonreplyF()) {
            dfu_terminate(DFU_ERR_TRANSFER);
            watchdog_extend = false;
            break;
        }
        if (thisargisF("+cftrantx:")) {
            if (thisargisF("+cftrantx: 0")) {
                seenF(0x02);
            } else if (thisargisF("+cftrantx: data")) {
                nextargF();
                thisargisF("*");
                uint16_t len = atoi(nextargF());
                dfu_total_length += len;
                dfu_total_packets++;
                if ((dfu_total_length - dfu_last_message_length) > 25000L) {
                    DEBUG_PRINTF("%lu\n", dfu_total_length);
                    dfu_last_message_length = dfu_total_length;
                }
#ifdef LED_RED
                gpio_pin_set(LED_PIN_RED, (dfu_total_packets & 0x00000008) != 0);
#endif
#ifdef LED_YEL
                gpio_pin_set(LED_PIN_YEL, (dfu_total_packets & 0x00000004) != 0);
#endif
                fona_watchdog_reset();
            } else {
                dfu_terminate(DFU_ERR_TRANSFER);
                watchdog_extend = false;
                break;
            }
        }
        if (thisargisF("ok"))
            seenF(0x01);
        if (allwereseenF(0x03)) {
            DEBUG_PRINTF("DFU download is valid\n");
            watchdog_extend = false;
            // Initiate the buttonless DFU
            processstateF(COMM_FONA_DFUPREPARE);
        }
        break;
    }

    case COMM_FONA_DFUPREPARE: {
        DEBUG_PRINTF("Done preparing for buttonless DFU\n");
        fona_send("at+fsls");
        setstateF(COMM_FONA_DFURPL9);
        break;
    }

    case COMM_FONA_DFURPL9: {
        dfu_terminate(DFU_ERR_NONE);
        break;
    }

        ///////
        /////// This section of state management is related to steady-state operations
        ///////

    case COMM_FONA_MISCRPL: {
        if (commonreplyF())
            break;
        if (thisargisF("ok"))
            seenF(0x01);
        if (allwereseenF(0x01)) {
            setidlestateF();
        }
        break;
    }

    case COMM_STATE_IDLE:
    case COMM_STATE_COMPLETE: {
        if (commonreplyF())
            break;
        setidlestateF();
        break;

    }

    } // switch

    // Now that we've processed a command, make sure
    // that we're reset and prepared to take another command.
    comm_cmdbuf_reset(&fromFona);

} // fona_process()

#endif // FONA
