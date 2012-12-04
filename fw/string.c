/* -*- indent-tabs-mode:T; c-basic-offset:8; tab-width:8; -*- vi: set ts=8:
 * $Id: string.c 2 2003-09-23 03:49:00Z tlb $
 *
 * Basic string and number output functions.  These should be generalized
 * to allow for sharing with the LCD module.
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

#include <avr/io.h>
#include <stdlib.h>

#include "string.h"
#include "uart.h"


/*************************************************************************
 *
 * String and number output functions
 *
 */
void
_puts(
	PGM_P			s
)
{
	char			c;
	while( (c = PRG_RDB( s++ )) )
		putc( c );
}


void
put_uint8_t(
	uint8_t			i
)
{
	putc( hexdigit( (i >> 4) & 0x0F ) );
	putc( hexdigit( (i >> 0) & 0x0F ) );
}


void
put_uint12_t(
	uint16_t		i
)
{
	putc( hexdigit( (i >> 8) & 0x0F ) );
	put_uint8_t(  (i >> 0) & 0xFF );
}


void
put_uint16_t(
	uint16_t		i
)
{
	put_uint8_t(  (i >> 8) & 0xFF );
	put_uint8_t(  (i >> 0) & 0xFF );
}



char		float_buf[12];

void
put_float(
	const float 		f
)
{
	uint8_t			i;


	dtostrf(
		f,
		sizeof( float_buf )-1,
		5,
		float_buf
	);

	for( i=0 ; i<sizeof(float_buf) ; i++ )
	{
		char c = float_buf[i];
		if( !c )
			break;
		putc( c );

		/* Stop on a NaN */
		if( i == 2 && c == 'N' )
		{
			putc( ' ' );
			break;
		}
	}
}

