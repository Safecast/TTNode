// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

// Miscellaneous utility functions

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "debug.h"
#include "misc.h"
#include "timer.h"

// Utility function to handle suppression timers, with date wraparound handling
bool WouldSuppress(uint32_t *lastTime, uint32_t suppressionSeconds) {
    uint32_t currentTime = get_seconds_since_boot();

    if (*lastTime != 0) {                                           // don't suppress upon initial entry
        if (currentTime < suppressionSeconds)                       // too early to subtract
            return true;
        if (currentTime >= *lastTime)                               // don't suppress if currentTime wrapped
            if ((currentTime - suppressionSeconds) < *lastTime)     // don't suppress if outside the suppression interval
                return true;
    }

    return false;
}

// Utility function to handle suppression timers, with date wraparound handling
bool ShouldSuppress(uint32_t *lastTime, uint32_t suppressionSeconds) {

    if (WouldSuppress(lastTime, suppressionSeconds))
        return true;

    *lastTime = get_seconds_since_boot();
    return false;

}

// Utility function that tries to maintain a consistent interval while still moving time forward
// The net effect of this function is that what's pointed to by lastTime will always increment
// in units of intervalSeconds, as opposed to being updated to the actual time when the work was done.
// This means that it will next fire earlier than it might've by using ShouldSuppress() alone.
bool ShouldSuppressConsistently(uint32_t *lastTime, uint32_t intervalSeconds) {

    // Suppress consistently
    uint32_t prevTime = *lastTime;
    uint32_t nextScheduledTime = prevTime + intervalSeconds;
    bool fSuppress = ShouldSuppress(lastTime, intervalSeconds);
    uint32_t thisDoneTime = *lastTime;
    if (!fSuppress && nextScheduledTime < thisDoneTime) {
        while ((nextScheduledTime + intervalSeconds) < thisDoneTime)
            nextScheduledTime += intervalSeconds;
        *lastTime = nextScheduledTime;
    }

    return fSuppress;
}

// Utility function to convert GPS DDDMM.MMMM N/S strings to signed degrees
float GpsEncodingToDegrees(char *inlocation, char *inzone) {
    float a, r;
    int i, d;
    a = atof(inlocation);
    i = (int) a;
    d = i / 100;
    a = a - (((float)d) * 100);
    r = ((float) d) + (a / 60);
    if (inzone[0] == 'S' || inzone[0] == 's' || inzone[0] == 'W' || inzone[0] == 'w')
        r = -r;
    return (r);
}

// Convert a pair of chars to hex
bool HexValue(char hiChar, char loChar, uint8_t *pValue) {
    uint8_t hi, lo;

    if (hiChar >= '0' && hiChar <= '9')
        hi = hiChar - '0';
    else if (hiChar >= 'A' && hiChar <= 'F')
        hi = (hiChar - 'A') + 10;
    else if (hiChar >= 'a' && hiChar <= 'f')
        hi = (hiChar - 'a') + 10;
    else
        return false;

    if (loChar >= '0' && loChar <= '9')
        lo = loChar - '0';
    else if (loChar >= 'A' && loChar <= 'F')
        lo = (loChar - 'A') + 10;
    else if (loChar >= 'a' && loChar <= 'f')
        lo = (loChar - 'a') + 10;
    else
        return false;

    if (pValue != NULL)
        *pValue = (hi << 4) | lo;

    return true;
}

// Extract the hexadecimal characters from a data byte
void HexChars(uint8_t databyte, char *hiChar, char *loChar) {
    char *hexchar = "0123456789ABCDEF";
    *hiChar = hexchar[((databyte >> 4) & 0x0f)];
    *loChar = hexchar[(databyte & 0x0f)];
}

// Create a null-terminated string of the form <string-prefix><hexified-bytes>
void HexCommand(char *buffer, uint16_t bufflen, char *prefix, uint8_t *bytes, uint16_t length) {
    char hiChar, loChar;
    int i;
    strlcpy(buffer, prefix, bufflen);
    i = strlen(buffer);
    buffer += i;
    bufflen -= (i + 1);
    for (i = 0; i < length; i++) {
        if (bufflen-- == 0)
            break;
        HexChars(bytes[i], &hiChar, &loChar);
        *buffer++ = hiChar;
        *buffer++ = loChar;
    }
    *buffer++ = '\0';
}

// Utility function to compute a "bracketed" standard deviation from an array of floats,
// where the bracket is the number of highest and lowest values that will be used to reflect
// the extremities variance there is within a given sample set.
#define BRACKET 2
float compute_maximum_deviation(float *values, uint16_t num_values) {
    float variance, mean, std, highest[BRACKET], lowest[BRACKET];
    int i, j, bracket_entries;

    // Exit if there are no values, so we don't divide by zero
    if (num_values == 0)
        return 0.0;
    
    // Track the highest and lowest values
    bracket_entries = 0;
    for (i=0; i<num_values; i++) {
        if (bracket_entries < BRACKET) {
            lowest[bracket_entries] = highest[bracket_entries] = values[i];
            bracket_entries++;
        } else {
            for (j=0; j<bracket_entries; j++)
                if (values[i] < lowest[j]) {
                    lowest[j] = values[i];
                    break;
                }
            for (j=0; j<bracket_entries; j++)
                if (values[i] > highest[j]) {
                    highest[j] = values[i];
                    break;
                }
        }
    }

    // If the number of bracket entries is zero, exit so we don't divide by 0
    if (bracket_entries == 0)
        return 0.0;
            
    // Compute the mean of just the bracketed entries
    mean = 0.0;
    for (i=0; i<bracket_entries; i++)
        mean += lowest[i];
    for (i=0; i<bracket_entries; i++)
        mean += highest[i];
    mean = mean / (bracket_entries * 2);
    
    // Compute the variance (the mean of the squared differences-from-mean) of the bracketed values
    variance = 0.0;
    for (i=0; i<bracket_entries; i++)
        variance += (lowest[i] - mean) * (lowest[i] - mean);
    for (i=0; i<bracket_entries; i++)
        variance += (highest[i] - mean) * (highest[i] - mean);
    variance = variance / (bracket_entries * 2);

    // Compute the standard deviation
    if (variance <= 0)
        std = 0.0;
    else
        std = sqrtf(variance);
        
    // Done
    return std;
    
}
