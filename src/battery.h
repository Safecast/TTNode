// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

#ifndef BATTERY_H__
#define BATTERY_H__
void battery_set_soc_to_unknown();
void battery_set_soc(float SOC);
float battery_soc();
float battery_soc_from_voltage(float voltage);
char *battery_status_name();

// Only one mode is ever active, however this is defined bitwise so that
// we can test using a bitwise-AND operator rather than just == or switch.
#define BAT_NO_SENSORS          0x0000
#define BAT_FULL                0x0001
#define BAT_NORMAL              0x0002
#define BAT_LOW                 0x0004
#define BAT_WARNING             0x0008
#define BAT_EMERGENCY           0x0010
#define BAT_DEAD                0x0020
#define BAT_TEST                0x0040
#define BAT_MOBILE              0x0080
#define BAT_BURN                0x0100
#define BAT_HEALTHY             (BAT_FULL|BAT_NORMAL|BAT_LOW|BAT_WARNING|BAT_TEST|BAT_MOBILE|BAT_BURN)
#define BAT_NOT_DEAD            (BAT_HEALTHY|BAT_EMERGENCY|BAT_MOBILE)
#define BAT_ALL                 (BAT_NOT_DEAD|BAT_DEAD)
uint16_t battery_status();

#endif // BATTERY_H__
