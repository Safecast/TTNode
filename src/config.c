// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.

// Compiled in the makefile every time we LINK the product

#include "config.h"

#ifndef APPVERSION
#define APPVERSION unknown
#endif

// This takes advantage of the obscure C preprocssor "stringification" feature for macro argument processing
// https://gcc.gnu.org/onlinedocs/cpp/Stringification.html
#define xstr(s) str(s)
#define str(s) #s

char *app_version() {
    return(xstr(APPVERSION));
}

char *app_build() {
    return(xstr(APPBUILD));
}
