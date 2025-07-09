// dump1090, a Mode S messages decoder for RTLSDR devices.
//
// Copyright (C) 2015  Travis Painter <travispainter@gmail.com>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
// This file incorporates work covered by the following copyright and  
// permission notice:
//
// Copyright (C) 2014 by Malcolm Robb <support@attavionics.com>
//
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//  *  Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//
//  *  Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// This file provides basic Windows implementation of Linux specific functions
// used in the dump1090 project. This allows dump1090 to be compiled and debugged 
// using Microsoft Visual C++ 6.0
//
// Note that not all functions actually provide equivalent functionality to their 
// Linux equivalents. They are simply stubs to allow the project to compile.
//
#ifndef __WINSTUBS_H
#define __WINSTUBS_H

#include <winsock2.h>
#include <windows.h>
#include <basetsd.h>

typedef UCHAR    uint8_t;
typedef USHORT   uint16_t;
typedef UINT32   uint32_t;
typedef UINT64   uint64_t;
//typedef UINT32   mode_t;
typedef long     ssize_t;
typedef int      socklen_t;

#include <stdio.h>
#include <time.h>
#include <sys/timeb.h>
#include <sys/stat.h>
#include <signal.h>
#include <io.h>
#include <fcntl.h>

#define M_PI 3.14159265358979323846
#include <math.h>
#include <pthread.h>
#include <winposixclock.h>
#include <endian.h>

#ifdef __cplusplus
extern "C" {
#endif

//Remove gcc specific __atribute__
#define __attribute__(A)

//Functions not included in the MSVC maths library. This will do for our use.
_inline double round(double d) {return floor(d + 0.5);}
_inline double trunc(double d) {return (d>0) ? floor(d):ceil(d) ;}

//usleep works in microseconds, and isn't supported in Windows. This will do for our use.
_inline void usleep(UINT32 ulSleep) {Sleep(ulSleep/1000);} 
_inline uint64_t strtoll(const char *p, void *e, UINT32 base) {return _atoi64(p);}
_inline int inet_aton(const char * cp, DWORD * ulAddr) { *ulAddr = inet_addr(cp); return (INADDR_NONE != *ulAddr);} 
#define snprintf  _snprintf
#define vsnprintf _vsnprintf
#define strcasecmp _stricmp
#define realpath(N,R) _fullpath((R),(N),_MAX_PATH)

enum {
	PATH_MAX = MAX_PATH
};

//Windows doesn't have localtime_r
_inline struct tm *localtime_r(time_t *_clock, struct tm *_result)
{
    _localtime64_s(_result, _clock);
    return _result;
}

_inline void cls() {
    HANDLE hStdOut = GetStdHandle(STD_OUTPUT_HANDLE);
    COORD coord = {0, 0};
    DWORD count;

    CONSOLE_SCREEN_BUFFER_INFO csbi;
    GetConsoleScreenBufferInfo(hStdOut, &csbi);

    FillConsoleOutputCharacter(hStdOut, ' ', csbi.dwSize.X * csbi.dwSize.Y, coord, &count);

    SetConsoleCursorPosition(hStdOut, coord);
}

//PostgreSQL function:
/*
 * gettimeofday.c
 *    Win32 gettimeofday() replacement
 *
 * src/port/gettimeofday.c
 *
 * Copyright (c) 2003 SRA, Inc.
 * Copyright (c) 2003 SKC, Inc.
 *
 * Permission to use, copy, modify, and distribute this software and
 * its documentation for any purpose, without fee, and without a
 * written agreement is hereby granted, provided that the above
 * copyright notice and this paragraph and the following two
 * paragraphs appear in all copies.
 *
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE TO ANY PARTY FOR DIRECT,
 * INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, INCLUDING
 * LOST PROFITS, ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS
 * DOCUMENTATION, EVEN IF THE UNIVERSITY OF CALIFORNIA HAS BEEN ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * THE AUTHOR SPECIFICALLY DISCLAIMS ANY WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE.  THE SOFTWARE PROVIDED HEREUNDER IS ON AN "AS
 * IS" BASIS, AND THE AUTHOR HAS NO OBLIGATIONS TO PROVIDE MAINTENANCE,
 * SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
 */

/* FILETIME of Jan 1 1970 00:00:00. */
static const unsigned __int64 epoch = ((unsigned __int64) 116444736000000000ULL);

/*
 * timezone information is stored outside the kernel so tzp isn't used anymore.
 *
 * Note: this function is not for Win32 high precision timing purpose. See
 * elapsed_time().
 */
_inline int gettimeofday(struct timeval * tp, struct timezone * tzp)
{
    FILETIME    file_time;
    SYSTEMTIME  system_time;
    ULARGE_INTEGER ularge;

    GetSystemTime(&system_time);
    SystemTimeToFileTime(&system_time, &file_time);
    ularge.LowPart = file_time.dwLowDateTime;
    ularge.HighPart = file_time.dwHighDateTime;

    tp->tv_sec = (long) ((ularge.QuadPart - epoch) / 10000000L);
    tp->tv_usec = (long) (system_time.wMilliseconds * 1000);

    return 0;
}


#define STDIN_FILENO 0
#define EINPROGRESS  WSAEINPROGRESS
#define EWOULDBLOCK  WSAEWOULDBLOCK

#ifdef __cplusplus
}
#endif

#endif // __WINSTUBS_H
