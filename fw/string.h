/* -*- indent-tabs-mode:T; c-basic-offset:8; tab-width:8; -*- vi: set ts=8:
 * $Id: string.h 9 2003-10-02 01:27:10Z tlb $
 *
 * (c) 2002 Trammell Hudson <hudson@swcp.com>
 *************
 *
 *  This file is part of the autopilot onboard code package.
 *  
 *  Autopilot is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *  
 *  Autopilot is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *  
 *  You should have received a copy of the GNU General Public License
 *  along with Autopilot; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef _string_h_
#define _string_h_

#include <avr/io.h>
#include "uart.h"
#include <avr/pgmspace.h>	/* For attributes */
#include <string.h>		/* For memset etc */

/*************************************************************************
 *
 * String and number output functions
 *
 */
extern void
_puts(
	PGM_P			s
);

/* Only deal with flash strings */
#define puts( s ) _puts( PSTR( s ) )


static inline char
hexdigit(
	uint8_t			i
)
{
	if( i < 0xA )
		return i + '0';
	return i + 'a' - 0xA;
}


extern void
put_uint8_t(
	uint8_t			i
);


extern void
put_uint12_t(
	uint16_t		i
);


extern void
put_uint16_t(
	uint16_t		i
);


static inline void
put_int16_t(
	int16_t			i
)
{
	putc( i < 0 ? '-' : '+' );
	put_uint16_t( i < 0 ? -i : i );
}


static inline void
put_uint32_t(
	uint32_t		i
)
{
	put_uint16_t( ( i >> 16 ) & 0xFFFF );
	put_uint16_t( ( i >>  0 ) & 0xFFFF );
}


extern char float_buf[];


extern void
put_float(
	const float 		f
);


static inline void
putnl( void )
{
	putc( '\r' );
	putc( '\n' );
}

#endif
