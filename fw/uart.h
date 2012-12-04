/*
 * $Id: uart.h 9 2003-10-02 01:27:10Z tlb $
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

#ifndef _UART_H_
#define _UART_H_

#include "debug.h"
#include <avr/signal.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <inttypes.h>


/*************************************************************************
 *
 *  UART code.
 */
extern void uart_init_for_debug(void);

extern void uart_init_for_roboteq(void);

extern void putc(uint8_t c);



extern uint8_t uart_send_empty(void);
extern uint8_t uart_tx_qlen(void);

void log_uint8_t(uint8_t x);
void log_uint16_t(uint16_t x);
void log_int16_t(int16_t x);
void log_float(float x);

#ifdef DEBUG
#define LOG_uint8_t(X) log_uint8_t(X)
#define LOG_uint16_t(X) log_uint16_t(X)
#define LOG_int16_t(X) log_int16_t(X)
#define LOG_float(X) log_float(X)
#define LOG_str(X) puts(X)
#else
#define LOG_uint8_t(X)
#define LOG_uint16_t(X)
#define LOG_int16_t(X)
#define LOG_float(X)
#define LOG_str(X)
#endif


#endif
