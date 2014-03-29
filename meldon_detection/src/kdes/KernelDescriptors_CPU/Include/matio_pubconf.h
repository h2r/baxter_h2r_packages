/* src/matio_pubconf.h.  Generated from matio_pubconf.h.in by configure.  */
/*
 * Copyright (C) 2010   Christopher C. Hulbert
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */
#ifndef MATIO_PUBCONF_H
#define MATIO_PUBCONF_H 1

/* Default file format */
#define MAT_FT_DEFAULT MAT_FT_MAT5

/* Define to 1 if you have the <stdint.h> header file. */
#define MATIO_HAVE_STDINT_H 1

/* Define to 1 if you have the <inttypes.h> header file. */
#define MATIO_HAVE_STDINT_H 1

/* int16 type */
#define _mat_int16_t int16_t

/* int32 type */
#define _mat_int32_t int32_t

/* int64 type */
#define _mat_int64_t int64_t

/* int8 type */
#define _mat_int8_t int8_t

/* int16 type */
#define _mat_uint16_t uint16_t

/* int32 type */
#define _mat_uint32_t uint32_t

/* int64 type */
#define _mat_uint64_t uint64_t

/* int8 type */
#define _mat_uint8_t uint8_t

#if MATIO_HAVE_INTTYPES_H
#   include <inttypes.h>
#endif

#if MATIO_HAVE_STDINT_H
#   include <stdint.h>
#endif

#ifdef _mat_int64_t
    typedef _mat_int64_t mat_int64_t;
#endif
#ifdef _mat_uint64_t
    typedef _mat_uint64_t mat_uint64_t;
#endif
#ifdef _mat_int32_t
    typedef _mat_int32_t mat_int32_t;
#endif
#ifdef _mat_uint32_t
    typedef _mat_uint32_t mat_uint32_t;
#endif
#ifdef _mat_int16_t
    typedef _mat_int16_t mat_int16_t;
#endif
#ifdef _mat_uint16_t
    typedef _mat_uint16_t mat_uint16_t;
#endif
#ifdef _mat_int8_t
    typedef _mat_int8_t mat_int8_t;
#endif
#ifdef _mat_uint8_t
    typedef _mat_uint8_t mat_uint8_t;
#endif

#endif /* MATIO_PUBCONF_H */
