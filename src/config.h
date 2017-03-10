// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

// Global configuration parameters

#ifndef TTCONFIG_H__
#define TTCONFIG_H__

// Default wireless carrier APN - "" means look it up by CICCID
#define WIRELESS_CARRIER_APN ""

// Interval between when we do DNS lookups, in case service IP changes
#define CELL_DNS_LOOKUP_INTERVAL_MINUTES    (48*60)

// Key performance parameters that impact battery life more than anything else (aside from sensor-defs.h)
#define ONESHOT_FAST_MINUTES                10
#define ONESHOT_MINUTES                     15

// How often we ping the service with stats requests
#define SERVICE_UPDATE_MINUTES              (12*60)

// How often we upload via cellular
#define ONESHOT_CELL_UPLOAD_MINUTES         60

// TTN info as of 2017-02-06 during conversion to TTN V2 API
#define TTN_TTSERVE_APP_EUI "70B3D57EF0003810"
#define TTN_TTSERVE_APP_KEY "5CB50DDCF44CEADA6A27DA8BC6607E6A"  // for device eui ending in 1B22AD

// Parameters defining high-power activities
#define SOC_HIGHPOWER_MIN                   100.0
#define SOC_HIGHPOWER_MAX                   110.0

// Parameters mapping out gateway robustness
#define DEFAULT_RESTART_DAYS                7
#define FAILOVER_CHECK_MINUTES              30
#ifdef FAILOVERDEBUG
#define FAILOVER_RESTART_MINUTES            60
#else
#define FAILOVER_RESTART_MINUTES            (24 * 60)
#endif

// The number of minutes that motion must be stable for us to begin measuring
#ifdef MOTIONDEBUG
#define MOTION_STABLE_MINUTES               1
#else
#define MOTION_STABLE_MINUTES               10
#endif

// This prevents the device from failing to boot if the GPS is bad for whatever reason
#define GPS_ABORT_MINUTES                   15
#define GPS_POLL_SECONDS                    10

// Our service's address
#define SERVICE_HTTP_ADDRESS "tt.safecast.org"
#define SERVICE_HTTP_PORT    80
#define SERVICE_HTTP_TOPIC  "/send"
#define SERVICE_UDP_ADDRESS  "tt-udp.safecast.org"
#define SERVICE_UDP_PORT     8081
#define SERVICE_TCP_ADDRESS  "tt.safecast.org"
#define SERVICE_TCP_PORT     8082
#define SERVICE_FTP_ADDRESS  "tt-ftp.safecast.org"
#define SERVICE_FTP_PORT     8083

// Amount of time we keep comms alive after a one-shot update
#define ONESHOT_UPDATE_SECONDS              (60*3)

// Amount of time we will tolerate "no carrier", rather than hanging the device completely
#define ONESHOT_ABORT_SECONDS               (60*5)

// Battery level auto-adjustment logic (except when debugging, as indicated by BTKEEPALIVE)
#ifdef BATTERYDEBUG
#define BATTERY_AUTOADJUST false
#else
#if defined(TWIMAX17043) || defined(TWIMAX17201) || defined(TWIINA219)
#define BATTERY_AUTOADJUST true
#else
#define BATTERY_AUTOADJUST false
#endif
#endif

// Must be longer than ANY timer below that you want to expire on a manual "drop" command at the console
#define FORCE_EXPIRATION_SECONDS            (60*120)

// Max transmit delay if client-side receive is in effect
#define RECEIVE_TIMEOUT_LISTEN_SECONDS      15
#define RECEIVE_TIMEOUT_RELAY_SECONDS       60

// When we'll stop advertising on Bluetooth
#ifndef POWERDEBUG
#define DROP_BTADVERTISING_SECONDS          (60*10)
#else
#define DROP_BTADVERTISING_SECONDS          20
#endif

// This is our primary app clock, which as a side effect also defines geiger bucket size
#define TT_TIMER_SECONDS                    15

// Geiger parameters
#define GEIGER_BUCKET_SECONDS               10
#define GEIGER_SAMPLE_SECONDS               (60*4)

// Power measurement parameters
#define PWR_SAMPLE_PERIOD_SECONDS           20
#define PWR_SAMPLE_SECONDS                  2

// Various watchdogs that auto-reset
#define CELL_WATCHDOG_SECONDS               60
#define LORA_WATCHDOG_SECONDS               60
#define SLEEP_WATCHDOG_SECONDS              20

// How long we do fast device updates, just to make sure the user knows that everything is working
#ifndef POWERDEBUG
#define FAST_DEVICE_UPDATE_SECONDS          (60*5)
#else
#define FAST_DEVICE_UPDATE_SECONDS          (60*2)
#endif

// Only begin after a full init
#define FAST_DEVICE_UPDATE_BEGIN            60

// Maximum time between updates for relayed bGeigie
#define BGEIGIE_SUPPRESSION_SECONDS         (60*5)

// Air sampling parameers
#define AIR_SAMPLE_SECONDS                  20
#define AIR_SAMPLE_PERIOD_SECONDS           (AIR_SAMPLE_SECONDS*6)
#define AIR_IGNORE_INCOMPLETE_SAMPLES       true

// Derived sampling parameters
// OPC is sampled once per poll interval
// PMS events come in asynchronously at one sample per second
#define AIR_SAMPLE_TOTAL_SECONDS            (((AIR_SAMPLE_PERIOD_SECONDS/AIR_SAMPLE_SECONDS)-1)*AIR_SAMPLE_SECONDS)
#define PMS_SAMPLE_MAX_BINS                 (AIR_SAMPLE_PERIOD_SECONDS+5)
#define OPC_SAMPLE_MAX_BINS                 ((AIR_SAMPLE_PERIOD_SECONDS/AIR_SAMPLE_SECONDS)+5)
#define OPC_SAMPLE_MIN_BINS                 3

// Random #secs added to rx/tx timeouts to keep them staggered
#define DESYNCHRONIZATION_SECONDS           (TT_TIMER_SECONDS+1)
// At least one timer tick unit
#define BOOT_DELAY_UNTIL_INIT               (TT_TIMER_SECONDS)

// Number of times (1-3) we try LoRaWAN before either resetting or switching to Cell.
// Note that you can't just bump this to > 3 without getting a "no_free_ch" error, which
// is why the chip reset is rquired between retries.
// 2017-03-09 I changed it from 3 to 1 because I noticed greater success in trying to
// join than doing multiple retries.
#define LORAWAN_JOIN_RETRIES                1

// Number of ms for Microchip to sleep when idle
#define LPWAN_SLEEP_MILLISECONDS            7000
// Number of ms to wake up serial BEFORE the above, so as to safely receive the "ok" reply
#define SERIAL_SLEEP_SAFETY_ZONE            1000
// Time to wait to flush serial before sleeping
#define SERIAL_FLUSH_MILLISECONDS           250

// Maximum time to wait for a LoRa ping reply
#define PING_REPLY_SECONDS                  (TT_TIMER_SECONDS+1)

// Frequency of service heartbeat pings when configured to do so
#define PING_SERVICE_SECONDS                60

// Forward references

char *app_version();
char *app_build();

#endif // TTCONFIG_H__
