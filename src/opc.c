// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

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

struct sample_s {
    float PM1;
    float PM2_5;
    float PM10;
};
typedef struct sample_s sample_t;
static sample_t samples[OPC_SAMPLE_MAX_BINS];
static uint16_t num_samples;
static uint16_t num_errors;
static uint16_t num_valid_reports;
static bool opc_polling_ok = false;

static uint32_t count_00_38;
static uint32_t count_00_54;
static uint32_t count_01_00;
static uint32_t count_02_10;
static uint32_t count_05_00;
static uint32_t count_10_00;
static uint16_t count_seconds;

static bool reported = false;
static sample_t reported_pm;
static uint32_t reported_count_00_38;
static uint32_t reported_count_00_54;
static uint32_t reported_count_01_00;
static uint32_t reported_count_02_10;
static uint32_t reported_count_05_00;
static uint32_t reported_count_10_00;
static uint16_t reported_count_seconds;

static bool settling = true;

static uint8_t rx_buf[100];
static opc_t opc_data;
static char opc_version[100];

// Extract data as a struct from the raw OPC data, pointing at the 0xf3
bool unpack_opc_version(char *ver, uint16_t ver_len, uint8_t *spiData)
{
    int i;
    // Bump to just after the 0xf3
    spiData++;
    // Extract the version
    for (i=0; i<(MIN(sizeof(rx_buf),ver_len)-1); i++) {
        // Note that we need a run of at least two, because the version # contains a decimal point
        if (spiData[i] == 0x2e && spiData[i+1] == 0x2e)
            break;
        ver[i] = spiData[i];
        return true;
    }
    ver[i] = '\0';
    return false;
}

// Extract data as a struct from the raw OPC data, pointing at the 0xf3
bool unpack_opc_data(opc_t *opc, uint8_t *spiData)
{
    uint8_t pos = 0;
    bool isValid = true;

    // Get Bin counts, assuming that our local machine arch is little-endian
    for (int i=0; i<NumHistogramBins; i++)
    {
        opc->binCount[i] = 0;
        for (int j=0; j<2; j++)
        {
            pos++;
            opc->binCount[i] |=  (((uint16_t)spiData[pos]) << 8*j);
        }
    }

    // Get mtof data
    opc->bin1_mtof = spiData[33];
    opc->bin3_mtof = spiData[34];
    opc->bin5_mtof = spiData[35];
    opc->bin7_mtof = spiData[36];

    // Get flow rate
    opc->flowRate = *(float *) &spiData[37];

    // Get Temperature or pressure (alternating)
    uint32_t tempPressVal = 0;
    tempPressVal |= spiData[41];
    tempPressVal |= spiData[42] << 8;
    tempPressVal |= spiData[43] << 16;
    tempPressVal |= spiData[44] << 24;
    if (tempPressVal < 1000)
    {
        opc->temperature = tempPressVal;
        opc->pressure = 0;
        opc->haveTemperature = true;
        opc->havePressure = false;
    }
    else
    {
        opc->pressure = tempPressVal;
        opc->temperature = 0;
        opc->haveTemperature = false;
        opc->havePressure = true;
    }

    // Get sampling period
    opc->samplePeriod = *(float *) &spiData[45];

    // Get checksom
    opc->checksum  = ((uint16_t)spiData[49]);
    opc->checksum |= ((uint16_t)spiData[50]) << 8;

    // Get PM  values
    opc->PM1   = *(float *) &spiData[51];
    opc->PM2_5 = *(float *) &spiData[55];
    opc->PM10  = *(float *) &spiData[59];
    
    // Validate checksum
#if 0   // Can't get checksum working and can't get info on algorith
    int i;
    uint16_t chk62 = 0;
    for (i=1; i<63; i++)
        chk62 += spiData[i];
    if (chk62 != opc->checksum)
        isValid = false;
#endif

    // Do some raw validation
    if (isnan(opc_data.PM1) || isnan(opc_data.PM2_5) || isnan(opc_data.PM10))
        isValid = false;
    if (opc_data.PM1 < 0 || opc_data.PM1 > 10000)
        isValid = false;
    if (opc_data.PM2_5 < 0 || opc_data.PM2_5 > 10000)
        isValid = false;
    if (opc_data.PM10 < 0 || opc_data.PM10 > 10000)
        isValid = false;

    // Debug data dump
    if (!isValid || debug(DBG_AIR)) {
        int i;
        DEBUG_PRINTF("OPC bad data detected:\n");
        for (i=0; i<62; i++) {
            DEBUG_PRINTF("%02x", spiData[i+1]);
            if ((i & 0x03) == 3)
                DEBUG_PRINTF("  ");
        }
        DEBUG_PRINTF("\n");
    }

    // Debug
    if (isValid && debug(DBG_SENSOR_MAX|DBG_SENSOR_SUPERMAX) && !settling) {
        if (debug(DBG_SENSOR_SUPERMAX))
            DEBUG_PRINTF("OPC %.3f %.3f %.3f (%d %d %d %d %d %d %d %d)\n", opc->PM1, opc->PM2_5, opc->PM10, opc->binCount[0], opc->binCount[1], opc->binCount[2], opc->binCount[3], opc->binCount[4], opc->binCount[5],  opc->binCount[6],  opc->binCount[7]);
        else
            DEBUG_PRINTF("OPC %.3f %.3f %.3f\n", opc->PM1, opc->PM2_5, opc->PM10);
    }            

    return isValid;

}

// The main access method for our data
bool s_opc_get_value(float *ppm_01_0, float *ppm_02_5, float *ppm_10_0,
                     uint32_t *pcount_00_38, uint32_t *pcount_00_54, uint32_t *pcount_01_00,
                     uint32_t *pcount_02_10, uint32_t *pcount_05_00, uint32_t *pcount_10_00,
                     uint16_t *pcount_seconds) {
    if (!reported)
        return false;

    if (ppm_01_0 != NULL)
        *ppm_01_0 = reported_pm.PM1;
    if (ppm_02_5 != NULL)
        *ppm_02_5 = reported_pm.PM2_5;
    if (ppm_10_0 != NULL)
        *ppm_10_0 = reported_pm.PM10;
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

    return true;
}

// Clear it out
void s_opc_clear_measurement() {
    reported = false;
    num_samples = 0;
    count_00_38 = count_00_54 = count_01_00 = count_02_10 = count_05_00 = count_10_00 = 0;
    count_seconds = 0;
    settling = false;
}

// Get init params
void s_opc_get_spi(uint16_t *pin, nrf_drv_spi_handler_t *handler) {
    *pin = SPI_PIN_SS_OPC;
    // Handle SPI commands synchronously
    *handler = NULL;
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
        err_code = nrf_drv_spi_transfer(spi_context(), tx, txlen, rx_buf, rxlen);
        if (err_code != NRF_SUCCESS) {
            DEBUG_PRINTF("SPI Transfer result = %04x\n", err_code);
            stats()->errors_opc++;
            return false;
        }

    } else {

        // Send just the first byte of the command.  If we send the second byte, it
        // has an impact on the first byte of what is ultimately received.  No, I don't know why.
        err_code = nrf_drv_spi_transfer(spi_context(), tx, txlen, &rx_buf[0], 1);
        if (err_code == NRF_SUCCESS && rx_buf[0] == 0xf3) {

            // Wait 5ms so that we skip over whatever trash was returned to us immediately
            // following the command.  This ensures that whatever we get afterward, which
            // comes after quite a bit of a delay, will start cleanly.
            nrf_delay_ms(5);

            // Receive each of these bytes individually.  We've found that we can't do a single large
            // read because the bytes apparently aren't yet available, and this technique introduces
            // sufficient delay so as to pick them up individually successfully.
            for (i=1; i<rxlen; i++) {
                err_code = nrf_drv_spi_transfer(spi_context(), NULL, 0, &rx_buf[i], 1);
                if (err_code != NRF_SUCCESS) {
                    DEBUG_PRINTF("OPC %02x error rcv[%d]\n", tx[0], i);
                    stats()->errors_opc++;
                    return false;
                }
            }
        }
    }

#if defined(SPIDEBUG)
    DEBUG_PRINTF("OPC %02x %s: ", tx[0], ((rx_buf[0] != 0xf3) ? "NOT READY" : "returned"));
    for (i=0; i<rxlen; i++)
        DEBUG_PRINTF("%02x", rx_buf[i]);
    DEBUG_PRINTF("\n");
#endif

    // Not ok if the first returned byte wasn't our OPC signature
    if (rx_buf[0] != 0xf3) {
        DEBUG_PRINTF("OPC cmd 0x%02x received bad header 0x%02x != 0xF3\n", tx[0], rx_buf[0]);
        if (++num_errors > OPC_IGNORED_SPI_ERRORS)
            stats()->errors_opc++;
        else
            DEBUG_PRINTF("(ignored:%d/%d valid:%d)\n", num_errors, OPC_IGNORED_SPI_ERRORS, num_valid_reports);
        return false;
    }

    // Do special command processing to unfold into other statics
    bool good = true;
    if (tx[0] == 0x30)
        good = unpack_opc_data(&opc_data, rx_buf);
    else if (tx[0] == 0x3f)
        good = unpack_opc_version(opc_version, sizeof(opc_version), rx_buf);

    if (!good) {
        DEBUG_PRINTF("OPC cmd 0x%02x received corrupt data\n", tx[0]);
        if (++num_errors > OPC_IGNORED_SPI_ERRORS)
            stats()->errors_opc++;
        else
            DEBUG_PRINTF("(ignored:%d/%d valid:%d\n", num_errors, OPC_IGNORED_SPI_ERRORS, num_valid_reports);
        return false;
    }

    return (true);

}

// Measurement needed?
bool s_opc_upload_needed(void *s) {
    return(s_opc_get_value(NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL));
}

// Measure it
void s_opc_measure(void *s) {
    int i;

    // Compute the reported
    reported_pm.PM1 = reported_pm.PM2_5 = reported_pm.PM10 = 0.0;
    reported_count_00_38 = 0;
    reported_count_00_54 = 0;
    reported_count_01_00 = 0;
    reported_count_02_10 = 0;
    reported_count_05_00 = 0;
    reported_count_10_00 = 0;
    reported_count_seconds = 0;

    // Avoid div by zero!
    if (num_samples) {

        for (i=0; i<num_samples; i++) {
            reported_pm.PM1 += samples[i].PM1;
            reported_pm.PM2_5 += samples[i].PM2_5;
            reported_pm.PM10 += samples[i].PM10;
        }

        reported_pm.PM1 = reported_pm.PM1 / num_samples;
        reported_pm.PM2_5 = reported_pm.PM2_5 / num_samples;
        reported_pm.PM10 = reported_pm.PM10 / num_samples;

        reported_count_00_38 = count_00_38;
        reported_count_00_54 = count_00_54;
        reported_count_01_00 = count_01_00;
        reported_count_02_10 = count_02_10;
        reported_count_05_00 = count_05_00;
        reported_count_10_00 = count_10_00;

        reported_count_seconds = count_seconds;

        num_valid_reports++;

    }

    if (num_samples >= OPC_SAMPLE_MIN_BINS)
        reported = true;
    else
        stats()->errors_opc++;
        

    if (debug(DBG_SENSOR_MAX))
        DEBUG_PRINTF("%sOPC reported(%d/%d) %.3f %.3f %.3f\n",
                     num_samples < OPC_SAMPLE_MIN_BINS ? "BAD: " : "",
                     num_samples, reported_count_seconds,
                     reported_pm.PM1, reported_pm.PM2_5, reported_pm.PM10);

    // Done with this sensor
    sensor_measurement_completed(s);

}

// Poller
void s_opc_poll(void *s) {

    // Exit if we're not supposed to be here
    if (!sensor_is_polling_valid(s))
        return;
    if (!opc_polling_ok)
        return;

    // Take samples and drop them into the appropriate bin
    if (count_seconds < AIR_SAMPLE_TOTAL_SECONDS  && num_samples < OPC_SAMPLE_MAX_BINS) {

        // Take a sample via spi
        static uint8_t req_data[] = {0x30};
        static uint16_t rsp_data_length = 63;
        if (spi_cmd(req_data, sizeof(req_data), rsp_data_length)) {

            // The initial sample after power-on is always 0.0
            if (opc_data.PM1 != 0.0 || opc_data.PM2_5 != 0.0 || opc_data.PM10 != 0.0) {
                // Drop it into a bin
                samples[num_samples].PM1 = opc_data.PM1;
                samples[num_samples].PM2_5 = opc_data.PM2_5;
                samples[num_samples].PM10 = opc_data.PM10;
                num_samples++;
                // Bump total counts
                count_00_38 += opc_data.binCount[0];
                count_00_54 += opc_data.binCount[1] + opc_data.binCount[2];
                count_01_00 += opc_data.binCount[3] + opc_data.binCount[4] + opc_data.binCount[5];
                count_02_10 += opc_data.binCount[6] + opc_data.binCount[7] + opc_data.binCount[8];
                count_05_00 += opc_data.binCount[9] + opc_data.binCount[10] + opc_data.binCount[11];
                count_10_00 += opc_data.binCount[12] + opc_data.binCount[13] + opc_data.binCount[14] + opc_data.binCount[15];
                count_seconds += AIR_SAMPLE_SECONDS;
            }
        }
    }

}

// Init sensor just after each power-on
void s_opc_done_settling() {

    // Clear out the values
    s_opc_clear_measurement();

}

// Init sensor just after each power-on
bool s_opc_init(void *s, uint16_t param) {
    int i, j;
    bool fEnabled = false;

    // OPC inter-command SPI Settling Delay - found by careful trial and error
#define OPC_SPI_SETTLING_DELAY 1500

    // Init state
    settling = true;
    num_errors = 0;
    num_valid_reports = 0;
    opc_polling_ok = false;

    // Do THREE FULL ATTEMPTS before giving up
    for (i=0; i<3; i++) {

        // Init SPI
        if (!spi_init()) {
            DEBUG_PRINTF("OPC SPI init failure\n");
            stats()->errors_opc++;
            return false;
        }

        // Turn on the laser and fan.  This works 99.9% of the time, but I've noticed that occasionally it will
        // fail to start and will then return NAN for the data values.
        fEnabled = false;

        for (j=0; j<5; j++) {

            // Let SPI settle down
            nrf_delay_ms(OPC_SPI_SETTLING_DELAY);

            // Turn fan and laser power) ON
            static uint8_t req_everything_on[] = {0x03, 0x00};
            static uint8_t rsp_everything_on[] = {0xf3, 0x03};
            if (spi_cmd(req_everything_on, sizeof(req_everything_on), sizeof(rsp_everything_on)) && (rx_buf[1] == rsp_everything_on[1])) {
                fEnabled = true;
                break;
            }
        }

        // Sometimes when this fails, it is because of the OPC borking up the SPI subsystem.  Try again.
        if (fEnabled)
            break;

        // This iteration failed, so terminate SPI
        spi_term();

        // Try again after a settling
        nrf_delay_ms(OPC_SPI_SETTLING_DELAY);

    }

    // Exit if we still haven't managed to enable it.  Note that SPI has already been terminated if !fEnabled
    if (!fEnabled) {
        DEBUG_PRINTF("OPC Laser & Fan FAILURE\n");
        stats()->errors_opc++;
        return false;
    }

    // Get the version
    static bool getVersion = true;
    if (getVersion) {
        nrf_delay_ms(OPC_SPI_SETTLING_DELAY);
        static uint8_t req_version[] = {0x3f};
        static uint16_t rsp_version_length = 61;
        if (spi_cmd(req_version, sizeof(req_version), rsp_version_length))
            getVersion = false;
        else
            strcpy(opc_version, "(cannot get version)");
    }

    if (debug(DBG_SENSOR_MAX))
        DEBUG_PRINTF("%s\n", opc_version);

    // Success
    opc_polling_ok = true;
    return true;

}

// Term sensor just before each power-off
bool s_opc_term() {

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
