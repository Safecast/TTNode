// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

// Process transmission of a TTProto-formatted message to TTGate or TTServe

#include <stdio.h>
#include <stdlib.h>
#include "debug.h"
#include "config.h"
#include "stats.h"
#include "storage.h"
#include "misc.h"
#include "io.h"

// Static statistics
static stats_t st;

// Update uptime stats
void stats_update() {
    if (!ShouldSuppress(&st.last_minute, 60)) {
        st.uptime_minutes++;
        if (st.uptime_minutes >= 60) {
            st.uptime_minutes = 0;
            st.uptime_hours++;
            if (st.uptime_hours >= 24) {
                st.uptime_hours = 0;
                st.uptime_days++;
                // Reset and update daily stats
                if (st.uptime_days > 1) {
                    st.transmitted_fullday = st.transmitted_today;
                    st.max_transmitted_fullday = st.max_transmitted_today;
                    st.received_fullday = st.received_today;
                    st.messages_fullday = st.messages_today;
                    st.joins_fullday = st.joins_today;
                    st.denies_fullday = st.denies_today;
                }
                st.transmitted_today = 0;
                st.max_transmitted_today = 0;
                st.received_today = 0;
                st.messages_today = 0;
                st.joins_today = 0;
                st.denies_today = 0;
                // Restart the device when appropriate
                if (storage()->restart_days != 0 && st.uptime_days >= storage()->restart_days) {
                    storage()->uptime_days += st.uptime_days;
                    storage_save(true);
                    io_request_restart();
                }
            }
        }
    }
}

// Get stats structure pointer
stats_t *stats() {
    return &st;
}

// Bump stats about sends/receives
void stats_io(uint16_t transmitted, uint16_t received) {
    if (transmitted) {
        st.messages++;
        st.transmitted += transmitted;
        st.transmitted_today += transmitted;
        if (transmitted > st.max_transmitted_today)
            st.max_transmitted_today = transmitted;
    }
    if (received) {
        st.received += received;
        st.received_today += received;
    }
}

// Quick status check
void stats_status_check(bool fVerbose) {
    if (fVerbose) {
        DEBUG_PRINTF("TODAY: xmt:%d mxmt:%d rcv:%d cnt:%d j:%d d:%d\n",
                     st.transmitted_today, st.max_transmitted_today, st.received_today, st.messages_today,
                     st.joins_today, st.denies_today);
        if (st.received_fullday)
            DEBUG_PRINTF("FULLDAY: xmt:%d mxmt:%d rcv:%d cnt:%d j:%d d:%d\n",
                         st.transmitted_fullday, st.max_transmitted_fullday, st.received_fullday, st.messages_fullday,
                         st.joins_fullday, st.denies_fullday);
    }
}
