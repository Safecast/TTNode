// Utility functions

#include <stdlib.h>
#include <string.h>
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
    strncpy(buffer, prefix, bufflen);
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
