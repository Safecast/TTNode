// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

#ifndef UTIL_H__
#define UTIL_H__

float GpsEncodingToDegrees(char *inlocation, char *inzone);
bool WouldSuppress(uint32_t *lastTransmitTime, uint32_t suppressionSeconds);
bool ShouldSuppress(uint32_t *lastTransmitTime, uint32_t suppressionSeconds);
bool ShouldSuppressConsistently(uint32_t *lastTransmitTime, uint32_t suppressionSeconds);
bool HexValue(char hiChar, char loChar, uint8_t *pValue);
void HexChars(uint8_t databyte, char *hiChar, char *loChar);
void HexCommand(char *buffer, uint16_t bufflen, char *prefix, uint8_t *bytes, uint16_t length);
float compute_maximum_deviation(float *values, uint16_t num_values);

#endif // UTIL_H__
