/*
 * $Id: timer.h 38 2004-01-16 16:57:41Z tlb $
 *
 * The Mega163 has three timers and we're using all of them.  See
 * timer_init() for a description of what they are doing.
 *
 * (c) 2002 Trammell Hudson <hudson@rotomotion.com>
 * (c) 2003 Trevor Blackwell <tlb@tlb.org>
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


#ifndef _timer_h_
#define _timer_h_

void
timer_init( void );

extern float timer1_seconds_conv;

#endif
