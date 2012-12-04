/*
 * (c) 2002 Trammell Hudson <hudson@swcp.com>
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

#include "debug.h"
#include <avr/signal.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include "uart.h"
#include "timer.h"


#define TX_BUF_SIZE		256
uint8_t				tx_head;
volatile uint8_t		tx_tail;
static uint8_t			tx_buf[TX_BUF_SIZE];

/*
 * UART Baud rate generation settings:
 *
 * With 16.0 MHz clock, UBRR=25 => 38400 baud
 * With 16.0 Mhz clock, UBRR=8 => 115200 baud
 *
 */

void
uart_init_for_debug( void )
{
  /* Baudrate is 115200 for a 16 Mhz clock */
#if CLOCK_SPEED == 16
  UBRRL = 16;
  sbi(UCSRA, U2X);
#else
#error "Unsupported clock"
#endif

  /* Enable the UART for sending and receiving */

  sbi(UCSRB, TXEN);

  tx_head = tx_tail = 0;
}

void
uart_init_for_roboteq( void )
{
  // The blasted thing takes 9600 7E1
#if CLOCK_SPEED == 16
  UBRRL = 103;
#else
#error "Unsupported clock"
#endif

  // Set for 7E1
  // enable 2 stop bits for now, to try to avoid lockup.
  UCSRC = (1<<URSEL) | (1<<UPM1) | (1<<UCSZ1) | (1<<USBS);

  sbi(UCSRB, TXEN); // Don't enable receiver

  tx_head = tx_tail = 0;
}


static inline void
write_to_uart( void )
{
  uint8_t next_tail;

  if (tx_head == tx_tail) {
    cbi(UCSRB, UDRIE);
    return;
  }
  
  next_tail = (tx_tail+1)%TX_BUF_SIZE;
  
  UDR = tx_buf[tx_tail];
  tx_tail = next_tail;
}

SIGNAL(SIG_UART_DATA)
{
  write_to_uart();
}

void 
putc(uint8_t c) 
{
  uint8_t next_head;
  // busy loop, waiting for space in buffer
  while (1) {
    next_head = (tx_head+1)%TX_BUF_SIZE;
    if (next_head!=tx_tail) break;
  }
  
  tx_buf[tx_head] = c;
  tx_head=next_head;

  sbi(UCSRB, UDRIE);
}


uint8_t
uart_send_empty( void )
{
  return tx_head == tx_tail;
}

uint8_t
uart_tx_qlen( void )
{
  return (tx_head+TX_BUF_SIZE-tx_tail) % TX_BUF_SIZE;
}


void log_uint8_t(uint8_t x)
{
  putc(x);
}

void log_uint16_t(uint16_t x)
{
  putc(x&0xff);
  putc(x>>8);
}

void log_int16_t(int16_t x)
{
  putc(x&0xff);
  putc(x>>8);
}

void log_float(float x)
{
  union {
    float f;
    uint8_t c[4];
  } tmp;
  tmp.f=x;
  putc(tmp.c[0]);
  putc(tmp.c[1]);
  putc(tmp.c[2]);
  putc(tmp.c[3]);
}
