/*
 * operating_system.c
 * Copyright (C) 2012-2015 Analog Devices
 * Author : Robin Getz <robin.getz@analog.com>
 *
 * fru-dump is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * fru-dump is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * fru-dump includes the files fru.c and fru.h, which are released under a
 * BSD-like license. These files can be used seperately from fru-dump,
 * and include in non-GPL software.
 */

//#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
//#include <unistd.h>
//#include <getopt.h>
//#include <sys/stat.h>
//#include <fcntl.h>
//#include <strings.h>
//#include <string.h>
//#define __USE_XOPEN /* needed for strptime */
//#include <time.h>
#include <stdarg.h>
//#include <stdbool.h>


#include "fru.h"

int quiet = 0;
int verbose = 0;

void * x_calloc(size_t nmemb, size_t size)
{
    unsigned int *ptr;

    if ((ptr = (unsigned int *)calloc(nmemb, size)) == NULL)
    {
        printf_err("ERROR: calloc could not allocate requested memory\n");
    }

    return (void *)ptr;
}


void printf_err(const char * fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
}

void printf_warn(const char * fmt, ...)
{
    va_list ap;

    if (quiet || !verbose) 
    {
        return;
    }

    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
}

void printf_info(const char * fmt, ...)
{
    va_list ap;

    if (quiet)
    {
        return;
    }
       

    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
}