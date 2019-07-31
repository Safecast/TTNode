
// Copyright 2017 Inca Roads LLC.  All rights reserved.
// Use of this source code is governed by licenses granted by the
// copyright holder including that found in the LICENSE file.
// The strlcpy/strlcat functions Copyright 1998 Todd C. Miller,
// extracted from the libbsd source distribution under open license.

// This package implements miscellaneous string functions that aren't defined by standard C libraries
/*  $OpenBSD: strlcpy.c,v 1.10 2005/08/08 08:05:37 espie Exp $  */

#include <sys/types.h>
#include <string.h>

// Copy src to string dst of size siz.  At most siz-1 characters
// will be copied.  Always NUL terminates (unless siz == 0).
// Returns strlen(src); if retval >= siz, truncation occurred.
#ifndef LIBBSD
size_t strlcpy(char *dst, const char *src, size_t siz) {
    char *d = dst;
    const char *s = src;
    size_t n = siz;

    // Copy as many bytes as will fit
    if (n != 0) {
        while (--n != 0) {
            if ((*d++ = *s++) == '\0')
                break;
        }
    }

    // Not enough room in dst, add NUL and traverse rest of src
    if (n == 0) {
        if (siz != 0)
            *d = '\0';
        while (*s++)
            ;
    }

    // count does not include '\0'
    return(s - src - 1);

}
#endif //LIBBSD

// Appends src to string dst of size siz (unlike strncat, siz is the
// full size of dst, not space left).  At most siz-1 characters
// will be copied.  Always NUL terminates (unless siz <= strlen(dst)).
// Returns strlen(src) + MIN(siz, strlen(initial dst)).
// If retval >= siz, truncation occurred.
#ifndef LIBBSD
size_t strlcat(char *dst, const char *src, size_t siz) {
    char *d = dst;
    const char *s = src;
    size_t n = siz;
    size_t dlen;

    // Find the end of dst and adjust bytes left but don't go past end
    while (n-- != 0 && *d != '\0')
        d++;
    dlen = d - dst;
    n = siz - dlen;
    if (n == 0)
        return(dlen + strlen(s));

    while (*s != '\0') {
        if (n != 1) {
            *d++ = *s;
            n--;
        }
        s++;
    }
    *d = '\0';

    // count does not include '\0'
    return(dlen + (s - src));

}
#endif //LIBBSD
