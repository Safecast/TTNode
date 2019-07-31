// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

// Use interrupt handler
#define OPC_SPI_HANDLER

// Alphasense OPC-N2

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "debug.h"
#include "boards.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_drv_spi.h"
#include "app_util_platform.h"
#include "gpio.h"
#include "config.h"
#include "timer.h"
#include "sensor.h"
#include "storage.h"
#include "comm.h"
#include "misc.h"
#include "spi.h"
#include "opc.h"
#include "io.h"
#include "stats.h"

#ifdef SPIOPC

// Number of errors ignored (the first sample is always invalid and is skipped)
#define OPC_LASER_RETRIES 9
#define OPC_IGNORED_ERRORS (1+3)

// OPC-N2 data (see opn.xls & specs)
#define NumHistogramBins 16
struct opc_s {
    uint16_t binCount[NumHistogramBins];
    uint8_t bin1_mtof;
    uint8_t bin3_mtof;
    uint8_t bin5_mtof;
    uint8_t bin7_mtof;
    float flowRate;
    uint32_t temperature;
    bool haveTemperature;
    uint32_t pressure;
    bool havePressure;
    float  samplePeriod;
    uint16_t checksum;
    float PM1;
    float PM2_5;
    float PM10;
};
typedef struct opc_s opc_t;

static float samples_PM1[OPC_SAMPLE_MAX_BINS];
static float samples_PM2_5[OPC_SAMPLE_MAX_BINS];
static float samples_PM10[OPC_SAMPLE_MAX_BINS];
static uint16_t num_samples;
static uint16_t num_valid_samples;
static uint16_t num_nonzero_samples;
static uint16_t num_samples_recorded;
static uint16_t num_samples_left_to_skip;
static uint16_t num_errors;
static uint16_t num_valid_reports;
static bool opc_polling_ok = false;
static bool request_opc_initialization = false;
static uint16_t opc_init_retries_left = 0;

#ifdef OPCGETVER    // Useful when debugging but worthless in production
static bool request_opc_version = true;
#else
static bool request_opc_version = false;
#endif

static uint32_t count_00_38;
static uint32_t count_00_54;
static uint32_t count_01_00;
static uint32_t count_02_10;
static uint32_t count_05_00;
static uint32_t count_10_00;
static uint16_t count_seconds;
static uint32_t count_began;

static uint16_t consecutive_std;

static bool     reported = false;
static bool     ever_reported = false;
static float    reported_pm_1;
static float    reported_pm_2_5;
static float    reported_pm_10;
static float    reported_std_1;
static float    reported_std_2_5;
static float    reported_std_10;
static uint32_t reported_count_00_38;
static uint32_t reported_count_00_54;
static uint32_t reported_count_01_00;
static uint32_t reported_count_02_10;
static uint32_t reported_count_05_00;
static uint32_t reported_count_10_00;
static uint16_t reported_count_seconds;

static uint8_t rx_buf[100];
static opc_t opc_data;
static char version[100];

// Event in progress
#ifdef OPC_SPI_HANDLER
static uint32_t spi_began = 0;
static uint32_t spi_message = 0;
#endif


// Forward
bool opc_init();
bool opc_version();

// Our SPI event handler.
#ifdef OPC_SPI_HANDLER
void opc_spi_event_handler(nrf_drv_spi_evt_t const * p_event) {

    // Mark the transaction as completed as appropriate
    if (p_event->type == NRF_DRV_SPI_EVENT_DONE)
        spi_began = 0;
    
}
#endif

// Begin an SPI transfer
void opc_spi_transfer_begin() {
    spi_began = get_seconds_since_boot();
    spi_message = spi_began;
}

// Wait for SPI transaction to complete
bool opc_spi_transfer_succeeded() {

    while (true) {

        // Compute duration of transaction
        uint32_t duration = get_seconds_since_boot() - spi_began;
        if (spi_began == 0)
            break;
        
        // Output a debug message
        if (!ShouldSuppress(&spi_message, 1))
            DEBUG_PRINTF("SPI command is taking too long!\n");

        // Exit if it has taken a very long time
        if (duration > 3)
            return false;
        
    }

    return true;
    
}

// Extract data as a struct from the raw OPC data, pointing at the 0xf3
bool unpack_opc_version(char *ver, uint16_t ver_len, uint8_t *spiData, uint16_t spiDataLen)
{
    int i;

    // Bump to just after the 0xf3
    spiData++;
    spiDataLen--;

    // Compute the maximum length as the buffer length
    for (i=0; ; i++) {
        // If we've run out of target buffer length, stop
        if (i >= ver_len-1)
            break;
        // If we've run out of spi data, stop.
        if (i >= spiDataLen-1)
            break;
        // If we see a run of at least two '.' characters, we're done.
        if (spiData[i] == 0x2e && spiData[i+1] == 0x2e)
            break;
        // Copy to output
        ver[i] = spiData[i];
    }

    // Null-terminate
    ver[i] = '\0';

    // Success if we've copied anything at all
    return (i != 0);

}

// Return true if this is a valid PM value, noting that using SPI numbers can get corrupted
// Also, invalidate the number if it is
bool valid_float(char *what, float *val) {
    float pm = *val;
    bool fValid = true;
    char *why = "";

    switch (fpclassify(pm)) {
    case FP_ZERO:
    case FP_NORMAL:
        fValid = true;
        break;
    case FP_NAN:
        fValid = false;
        why = "FP_NAN";
        break;
    case FP_INFINITE:
        fValid = false;
        why = "FP_INFINITE";
        break;
    case FP_SUBNORMAL:
        fValid = false;
        why = "FP_SUBNORMAL";
        break;
    default:
        fValid = false;
        why = "????";
        break;
    }

    // ***
    // Reset the FP hardware state because something, potentially during SPI
    // initialization, has placed the FP hardware into a bad state.  Note
    // that if you remove this code you will take an FP trap (and crash the nRF)
    // on the very first floating point operation after this point.
    // Ray 2017-05-31
    // ***
    volatile float f = 0.0;
    isless(f, 0.0);

    // If not valid, output it
    if (!fValid && *what != '\0') {
        uint8_t *p = (uint8_t *) val;
        DEBUG_PRINTF("*** %s invalid (%s): ", what, why);
        for (int i=0; i<sizeof(float); i++)
            DEBUG_PRINTF("%02x", p[i]);
        DEBUG_PRINTF("\n");
    }

    // If not valid, zero it out so that we don't get an FP fault when calculating with it
    if (!fValid)
        *val = 0.0;

    // Valid
    return fValid;
}

// Extract data as a struct from the raw OPC data, pointing at the 0xf3
bool unpack_opc_data(opc_t *opc, uint8_t *spiData, uint16_t spiDataLen)
{
    uint8_t pos = 0;

    // If debugging, print but ignore ALL data
    if (debug(DBG_SENSOR_SUPERMAX)) {
        DEBUG_PRINTF("OPC raw data: ");
        int i;
        for (i=0; i<spiDataLen; i++) {
            DEBUG_PRINTF("%02x", spiData[i]);
            if ((i % 8) == 7)
                DEBUG_PRINTF("        ");
        }
        DEBUG_PRINTF("\n");
    }

    // Exit if we're skipping samples, waiting for settling, and remember when
    // the sampling actually began.
    if (num_samples_left_to_skip != 0) {
        if (--num_samples_left_to_skip == 0) {
            if (debug(DBG_SENSOR_SUPERMAX))
                DEBUG_PRINTF("Sample should be skipped.\n");
            count_began = get_seconds_since_boot();
        }
        return false;
    }

    // Get Bin counts, assuming that our local machine arch is little-endian
    for (int i=0; i<NumHistogramBins; i++) {
        opc->binCount[i] = 0;
        for (int j=0; j<2; j++) {
            pos++;
            opc->binCount[i] |=  (((uint16_t)spiData[pos]) << 8*j);
        }
    }

    // Get mtof data
    opc->bin1_mtof = spiData[33];
    opc->bin3_mtof = spiData[34];
    opc->bin5_mtof = spiData[35];
    opc->bin7_mtof = spiData[36];

    // Get flow rate, and zero out any corrupt FP numbers
    opc->flowRate = *(float *) &spiData[37];
    valid_float("", &opc->flowRate);

    // Get Temperature or pressure (alternating)
    uint32_t tempPressVal = 0;
    tempPressVal |= spiData[41];
    tempPressVal |= spiData[42] << 8;
    tempPressVal |= spiData[43] << 16;
    tempPressVal |= spiData[44] << 24;
    if (tempPressVal < 1000) {
        opc->temperature = tempPressVal;
        opc->pressure = 0;
        opc->haveTemperature = true;
        opc->havePressure = false;
    } else {
        opc->pressure = tempPressVal;
        opc->temperature = 0;
        opc->haveTemperature = false;
        opc->havePressure = true;
    }

    // Get sampling period, and zero out any corrupt FP numbers
    opc->samplePeriod = *(float *) &spiData[45];
    valid_float("", &opc->samplePeriod);

    // Get checksom
    opc->checksum  = ((uint16_t)spiData[49]);
    opc->checksum |= ((uint16_t)spiData[50]) << 8;

    // Check PM and ensure that the values are valid floating point numbers
    bool isValid = true;
    opc->PM1   = *(float *) &spiData[51];
    if (!valid_float("PM01", &opc->PM1))
        isValid = false;
    opc->PM2_5 = *(float *) &spiData[55];
    if (!valid_float("PM25", &opc->PM2_5))
        isValid = false;
    opc->PM10  = *(float *) &spiData[59];
    if (!valid_float("PM10", &opc->PM10))
        isValid = false;

    // Zero value testing, just to make sure the code works and allows 0's all the way through
#ifdef AIR_ZERO_TEST
    for (int i=0; i<NumHistogramBins; i++)
        opc->binCount[i] = 0;
    opc->PM1 = opc->PM2_5 = opc->PM10 = 0.0;
#endif

    // Check buckets
    uint32_t bin_sum = 0;
    for (int i=0; i<NumHistogramBins; i++)
        bin_sum += opc->binCount[i];

    // Report total samples analyzed
    num_samples++;

    // Report nonzero values analyzed
    if (bin_sum != 0 || opc->PM1 != 0.0 || opc->PM2_5 != 0.0 || opc->PM10 != 0.0)
        num_nonzero_samples++;

    // Report valid values analyzed
    if (isValid)
        num_valid_samples++;

    // Done
    return isValid;

}

// Show the current value
bool s_opc_show_value(uint32_t when, char *buffer, uint16_t length) {
    static uint32_t last = 0;
    char msg[128];
    if (when == last)
        return false;
    last = when;
    if (ever_reported)
        sprintf(msg, "OPC %.2f %.2f %.2f", reported_pm_1, reported_pm_2_5, reported_pm_10);
    else
        sprintf(msg, "OPC not yet measured");
    strlcpy(buffer, msg, length);
    return true;
}

// The main access method for our data
bool s_opc_get_value(float *ppm_01_0, float *ppm_02_5, float *ppm_10_0,
                     float *pstd_01_0, float *pstd_02_5, float *pstd_10_0,
                     uint32_t *pcount_00_38, uint32_t *pcount_00_54, uint32_t *pcount_01_00,
                     uint32_t *pcount_02_10, uint32_t *pcount_05_00, uint32_t *pcount_10_00,
                     uint16_t *pcount_seconds) {

    if (ppm_01_0 != NULL)
        *ppm_01_0 = reported_pm_1;
    if (ppm_02_5 != NULL)
        *ppm_02_5 = reported_pm_2_5;
    if (ppm_10_0 != NULL)
        *ppm_10_0 = reported_pm_10;
    if (pstd_01_0 != NULL)
        *pstd_01_0 = reported_std_1;
    if (pstd_02_5 != NULL)
        *pstd_02_5 = reported_std_2_5;
    if (pstd_10_0 != NULL)
        *pstd_10_0 = reported_std_10;
    if (pcount_00_38 != NULL)
        *pcount_00_38 = reported_count_00_38;
    if (pcount_00_54 != NULL)
        *pcount_00_54 = reported_count_00_54;
    if (pcount_01_00 != NULL)
        *pcount_01_00 = reported_count_01_00;
    if (pcount_02_10 != NULL)
        *pcount_02_10 = reported_count_02_10;
    if (pcount_05_00 != NULL)
        *pcount_05_00 = reported_count_05_00;
    if (pcount_10_00 != NULL)
        *pcount_10_00 = reported_count_10_00;
    if (pcount_seconds != NULL)
        *pcount_seconds = reported_count_seconds;

    return reported;

}

// Clear it out
void s_opc_clear_measurement() {
    reported = false;
    num_samples_recorded = 0;
    consecutive_std = 0;
    count_00_38 = count_00_54 = count_01_00 = count_02_10 = count_05_00 = count_10_00 = 0;
    count_began = get_seconds_since_boot();
}

// Get init params
void s_opc_get_spi(uint16_t *pin, nrf_drv_spi_handler_t *handler) {
    *pin = SPI_PIN_SS_OPC;
    // Handle SPI commands synchronously
#ifdef OPC_SPI_HANDLER
    *handler = opc_spi_event_handler;
#else
    *handler = NULL;
#endif
}

// Transmit an SPI command, and return the value in rx_buf buffer
bool spi_cmd(uint8_t *tx, uint16_t txlen, uint16_t rxlen) {
    uint32_t err_code;
    int i;

    // This of course will never happen.  Defensive programming.
    if (rxlen > sizeof(rx_buf)) {
        DEBUG_PRINTF("Buffer overrun!\n");
        rxlen = sizeof(rx_buf);
    }

    // We've found that we cannot execute commands too quickly else we get garbage,
    // as indicated by the very first byte of the reply not being 0xf3
    nrf_delay_ms(500);

    // Do special handling for the commands requiring large receives
    if (tx[0] != 0x3f && tx[0] != 0x30) {

        // Issue the normal command if it's not a special one
        opc_spi_transfer_begin();
        err_code = nrf_drv_spi_transfer(spi_context(), tx, txlen, rx_buf, rxlen);
        if (err_code != NRF_SUCCESS) {
            DEBUG_PRINTF("SPI Transfer (rcv) result = %04x\n", err_code);
            stats()->errors_spi++;
            stats()->errors_opc++;
            return false;
        }
        if (!opc_spi_transfer_succeeded()) {
            DEBUG_PRINTF("SPI Transfer TIMEOUT\n");
            stats()->errors_spi++;
            stats()->errors_opc++;
            return false;
        }

    } else {

        // Send just the first byte of the command.  If we send the second byte, it
        // has an impact on the first byte of what is ultimately received.  No, I don't know why.
        opc_spi_transfer_begin();
        err_code = nrf_drv_spi_transfer(spi_context(), tx, txlen, &rx_buf[0], 1);
        if (err_code != NRF_SUCCESS) {
            DEBUG_PRINTF("SPI Transfer (cmd) result = %04x\n", err_code);
            stats()->errors_spi++;
            stats()->errors_opc++;
            return false;
        }
        if (!opc_spi_transfer_succeeded()) {
            DEBUG_PRINTF("SPI Transfer TIMEOUT\n");
            stats()->errors_spi++;
            stats()->errors_opc++;
            return false;
        }

        if (rx_buf[0] == 0xf3) {

            // Wait 5ms so that we skip over whatever trash was returned to us immediately
            // following the command.  This ensures that whatever we get afterward, which
            // comes after quite a bit of a delay, will start cleanly.
            nrf_delay_ms(5);

            // Receive each of these bytes individually.  We've found that we can't do a single large
            // read because the bytes apparently aren't yet available, and this technique introduces
            // sufficient delay so as to pick them up individually successfully.
            for (i=1; i<rxlen; i++) {
                opc_spi_transfer_begin();
                err_code = nrf_drv_spi_transfer(spi_context(), NULL, 0, &rx_buf[i], 1);
                if (err_code != NRF_SUCCESS) {
                    DEBUG_PRINTF("OPC %02x error rcv[%d]\n", tx[0], i);
                    stats()->errors_spi++;
                    stats()->errors_opc++;
                    return false;
                }
                if (!opc_spi_transfer_succeeded()) {
                    DEBUG_PRINTF("OPC %02x xfer #%d TIMEOUT\n", tx[0], i);
                    stats()->errors_spi++;
                    stats()->errors_opc++;
                    return false;
                }
            }

        }
    }


    // Not ok if the first returned byte wasn't our OPC signature
    if (rx_buf[0] != 0xf3) {
        // Allow several pieces corrupt piece of data per reading, silently.  This is
        // an arbitrary number, however a) randomness on the SPI bus does indeed happen from
        // time to time, and 2) we don't want to leave ALL corruption unflagged
        if (++num_errors > OPC_IGNORED_ERRORS) {
            stats()->errors_opc++;
            DEBUG_PRINTF("OPC cmd 0x%02x bad header 0x%02x != 0xF3\n", tx[0], rx_buf[0]);
        }
        return false;
    }

    // Do special command processing to unfold into other statics
    bool good = true;
    if (tx[0] == 0x30)
        good = unpack_opc_data(&opc_data, rx_buf, 63);
    else if (tx[0] == 0x3f)
        good = unpack_opc_version(version, sizeof(version), rx_buf, 61);

    if (!good) {
        // The first piece of data that comes back is ALWAYS invalid, and is skipped.
        // But even beyond this, aw allow several pieces corrupt piece of data per reading,
        // silently.  This is a somewhat arbitrary number, however a) randomness on the
        // SPI bus does indeed happen from time to time, and 2) we don't want to leave
        // ALL corruption unflagged.
        if (++num_errors > OPC_IGNORED_ERRORS) {
            stats()->errors_opc++;
            DEBUG_PRINTF("OPC error: zero %d, invalid %d, total %d\n", num_samples-num_nonzero_samples, num_samples-num_valid_samples, num_samples);
        }
        if (debug(DBG_SENSOR_SUPERMAX))
            DEBUG_PRINTF("Skipping sample.\n");
        return false;
    }

    return (true);

}

// Measurement needed?
bool s_opc_upload_needed(void *s) {
    return(s_opc_get_value(NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL));
}

// Measure it
void s_opc_measure(void *s) {
    float std1, std2_5, std10;

    // Compute the reported
    reported_pm_1 = reported_pm_2_5 = reported_pm_10 = 0.0;
    reported_std_1 = reported_std_2_5 = reported_std_10 = 0.0;
    std1 = std2_5 = std10 = 0.0;
    reported_count_00_38 = 0;
    reported_count_00_54 = 0;
    reported_count_01_00 = 0;
    reported_count_02_10 = 0;
    reported_count_05_00 = 0;
    reported_count_10_00 = 0;
    reported_count_seconds = 0;

    // Avoid div by zero!
    if (num_samples_recorded) {
        int i;

        for (i=0; i<num_samples_recorded; i++) {
            reported_pm_1 += samples_PM1[i];
            reported_pm_2_5 += samples_PM2_5[i];
            reported_pm_10 += samples_PM10[i];
        }
        reported_pm_1 = reported_pm_1 / num_samples_recorded;
        reported_pm_2_5 = reported_pm_2_5 / num_samples_recorded;
        reported_pm_10 = reported_pm_10 / num_samples_recorded;

        reported_count_00_38 = count_00_38;
        reported_count_00_54 = count_00_54;
        reported_count_01_00 = count_01_00;
        reported_count_02_10 = count_02_10;
        reported_count_05_00 = count_05_00;
        reported_count_10_00 = count_10_00;

        // Compute the standard deviations
        reported_std_1 = std1 = compute_maximum_deviation(samples_PM1, num_samples_recorded);
        reported_std_2_5 = std2_5 = compute_maximum_deviation(samples_PM2_5, num_samples_recorded);
        reported_std_10 = std10 = compute_maximum_deviation(samples_PM10, num_samples_recorded);

        // Apply a filter to the reported STD values to save bandwidth
        if (reported_pm_1 < AIR_MATERIAL_PM || reported_std_1 < (reported_pm_1*AIR_MATERIAL_STD_MULTIPLE))
            reported_std_1 = 0;
        if (reported_pm_2_5 < AIR_MATERIAL_PM || reported_std_2_5 < (reported_pm_2_5*AIR_MATERIAL_STD_MULTIPLE))
            reported_std_2_5 = 0;
        if (reported_pm_10 < AIR_MATERIAL_PM || reported_std_10 < (reported_pm_10*AIR_MATERIAL_STD_MULTIPLE))
            reported_std_10 = 0;

        // Valid
        reported_count_seconds = count_seconds;
        num_valid_reports++;

    }

    // If we haven't measured sufficiently long, it's an error
    if (num_samples_recorded >= OPC_SAMPLE_MIN_BINS)
        reported = ever_reported = true;
    else
        stats()->errors_opc++;

    // If high variance, don't allow it to pollute our data.  If repeated high variance, it's an error.
    if (reported_std_1 != 0 || reported_std_2_5 != 0 || reported_std_10 != 0) {
        if (++consecutive_std > 3)
            stats()->errors_opc++;
    } else
        consecutive_std = 0;

    // Debug
    if (debug(DBG_SENSOR_MAX)) {
        uint16_t num_zero = num_samples - num_nonzero_samples;
        uint16_t num_invalid = num_samples - num_valid_samples;
        if (!reported || consecutive_std != 0)
            DEBUG_PRINTF("OPC FAIL (recorded %d, zero %d, invalid %d, total %d) %.2f %.2f %.2f", num_samples_recorded, num_zero, num_invalid, num_samples, reported_pm_1, reported_pm_2_5, reported_pm_10);
        else {
            char extra[64] = "";
            if (num_zero != 0 || num_invalid != 0)
                sprintf(extra, " (recorded %d, zero %d, invalid %d, total %d)", num_samples_recorded, num_zero, num_invalid, num_samples);
            DEBUG_PRINTF("OPC reported%s %.2f %.2f %.2f", extra, reported_pm_1, reported_pm_2_5, reported_pm_10);
            DEBUG_PRINTF(" {%.0f %.0f %.0f} in %ds\n", std1, std2_5, std10, reported_count_seconds);
        }
    }

    // Done with this sensor
    sensor_measurement_completed(s);

}

// Poller
void s_opc_poll(void *s) {

    // Exit if we're not supposed to be here
    if (!sensor_is_polling_valid(s))
        return;

    // Initialize the device if it hasn't yet been initialized
    if (!opc_init())
        return;

    // Get the version if needed
    if (!opc_version())
        return;

    // If init isn't completed, come back next poll
    if (!opc_polling_ok)
        return;

    // Take samples and drop them into the appropriate bin
    if (num_samples_recorded < OPC_SAMPLE_MAX_BINS) {

        // Take a sample via spi
        static uint8_t req_data[] = {0x30};
        static uint16_t rsp_data_length = 63;
        bool success = spi_cmd(req_data, sizeof(req_data), rsp_data_length);

        if (success) {

            // For timing reasons, verify num_samples_recorded once again after the spi_cmd
            if (num_samples_recorded < OPC_SAMPLE_MAX_BINS) {

                // Drop it into a bin
                samples_PM1[num_samples_recorded] = opc_data.PM1;
                samples_PM2_5[num_samples_recorded] = opc_data.PM2_5;
                samples_PM10[num_samples_recorded] = opc_data.PM10;
                num_samples_recorded++;

                // Bump total counts
                count_00_38 += opc_data.binCount[0];
                count_00_54 += opc_data.binCount[1] + opc_data.binCount[2];
                count_01_00 += opc_data.binCount[3] + opc_data.binCount[4] + opc_data.binCount[5];
                count_02_10 += opc_data.binCount[6] + opc_data.binCount[7] + opc_data.binCount[8];
                count_05_00 += opc_data.binCount[9] + opc_data.binCount[10] + opc_data.binCount[11];
                count_10_00 += opc_data.binCount[12] + opc_data.binCount[13] + opc_data.binCount[14] + opc_data.binCount[15];
                count_seconds = (uint16_t) (get_seconds_since_boot() - count_began);

                // Debug
                if (debug(DBG_SENSOR_MAX))
                    DEBUG_PRINTF("OPC %.2f %.2f %.2f\n", opc_data.PM1, opc_data.PM2_5, opc_data.PM10);
            }
        }
    }

}

// Init sensor just after each power-on
bool s_opc_init(void *s, uint16_t param) {

    // Disable polling for now
    opc_polling_ok = false;

    // Init SPI
    if (!spi_init()) {
        DEBUG_PRINTF("OPC SPI init failure\n");
        stats()->errors_opc++;
        return false;
    }

    // Request the initiialization
    request_opc_initialization = true;
    opc_init_retries_left = OPC_LASER_RETRIES;

    return true;
}

// Version handling
bool opc_version() {

    // Exit if we're not supposed to be here
    if (!request_opc_version)
        return true;

    // Get the version IF we can, but only try once.  The most common failure is that
    // we get the first byte "O", but that the remainder of the command fails
    static uint8_t req_version[] = {0x3f};
    static uint16_t rsp_version_length = 61;
    if (!spi_cmd(req_version, sizeof(req_version), rsp_version_length))
        strcpy(version, "(cannot get version)");

    DEBUG_PRINTF("%s\n", version);

    // Only give this a single attempt
    request_opc_version = false;

    // Force a timer quantum upon return, to keep SPI commands from stacking up
    return false;
}

// The real init, which is performed during polling
bool opc_init() {
    uint16_t opc_init_retry = (OPC_LASER_RETRIES - opc_init_retries_left) + 1;

    // Exit if we're not supposed to be here
    if (!request_opc_initialization)
        return true;

    // Turn fan and laser power) ON
    static uint8_t req_everything_on[] = {0x03, 0x00};
    static uint8_t rsp_everything_on[] = {0xf3, 0x03};
    if (!spi_cmd(req_everything_on, sizeof(req_everything_on), sizeof(rsp_everything_on)) && (rx_buf[1] == rsp_everything_on[1])) {
        if (debug(DBG_SENSOR_SUPERMAX))
            DEBUG_PRINTF("OPC Fan+Laser FAILURE (%d/%d)\n", opc_init_retry, OPC_LASER_RETRIES);
        if (--opc_init_retries_left == 0) {
            stats()->errors_opc++;
            request_opc_initialization = false;
            DEBUG_PRINTF("Giving up on OPC.\n");
        }
        return false;
    }

    // Success
    if (debug(DBG_SENSOR_SUPERMAX)) {
        if (opc_init_retry == 1)
            DEBUG_PRINTF("OPC Fan+Laser ON\n");
        else
            DEBUG_PRINTF("OPC Fan+Laser ON (%d/%d)\n", opc_init_retry, OPC_LASER_RETRIES);
    }

    // Initialize state
    s_opc_clear_measurement();
    num_errors = 0;
    num_valid_reports = 0;
    num_samples = 0;
    num_valid_samples = 0;
    num_nonzero_samples = 0;
    opc_polling_ok = true;

    // The first value that we receive is ALWAYS zero
    num_samples_left_to_skip = 1;

    // Don't come back
    request_opc_initialization = false;

    // Force a timer quantum upon return, to keep SPI commands from stacking up
    return false;

}

// Term sensor just before each power-off
bool s_opc_term() {

    // Cancel the init request if it isn't done yet
    request_opc_initialization = false;

    // Disable polling as a defensive measure
    opc_polling_ok = false;

    // Terminate SPI
    spi_term();

    // Done
    if (num_valid_reports == 0) {
        DEBUG_PRINTF("OPC term: no valid reports!\n");
        stats()->errors_opc++;
        return false;
    }
    return true;

}

#endif // SPIOPC
