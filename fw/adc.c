/*
 * (c) 2003 Trevor Blackwell <tlb@tlb.org>
 *
 *  This file is part of the scooter onboard code package.
 *  
 *  Scooter is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *  
 *  Scooter is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *  
 *  You should have received a copy of the GNU General Public License
 *  along with Autopilot; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *
 */


#include <avr/signal.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include "adc.h"


static uint16_t adc_current_samples[8];
uint16_t adc_filtered_samples[8];
uint8_t adc_sample_count;

void
adc_init( void )
{
  /* Ensure that our port is for input with no pull-ups */
  PORTA = 0x00;
  DDRA = 0x00;

  /* Select our external voltage ref, which is tied to Vcc */
  ADMUX = 0x00;

  /* Turn off the analog comparator */
  sbi(ACSR, ACD);

  /* Select out clock, turn on the ADC interrupt and start conversion */
  ADCSRA = 0
    | (1<<ADPS0) | (1<<ADPS1) | (1<<ADPS2) /* divide by 128 */
    | (1 << ADEN)
    | (1 << ADIE)
    | (1 << ADSC);
}

/*
  If there are at least minsamples samples of each ADC input, store
  their sum in samples[0..7] and return the number of samples.
  Otherwise return 0.

  You'll want to divide samples[*] by the return value to get the
  average.
*/
uint8_t adc_collect_samples(uint16_t *samples, int minsamples)
{
  uint8_t tmp_count;
  int i;

  if (adc_sample_count < minsamples) return 0;

  cli();
  for (i=0; i<8; i++) {
    samples[i] = adc_filtered_samples[i];
    adc_filtered_samples[i]=0;
  }
  tmp_count = adc_sample_count;
  adc_sample_count=0;
  sei();

  return tmp_count;
}


/**
 * Called when the voltage conversion is finished
 */
SIGNAL( SIG_ADC )
{
  uint8_t adci = ADMUX & 0x7;
  uint8_t i;

  adc_current_samples[adci] = ADCW;

  if (adci == 7) {

    if (adc_sample_count >= 60) {
      /* bad thing, shouldn't happen often */
      for (i=0; i<8; i++) {
        adc_filtered_samples[i] /= 2;
      }
      adc_sample_count/=2;
    }

    for (i=0; i<8; i++) {
      adc_filtered_samples[i] += adc_current_samples[i];
    }
    adc_sample_count++;
  }

  ADMUX = (adci+1)&7;

  /* Restart the conversion */
  sbi(ADCSRA, ADSC);
}
