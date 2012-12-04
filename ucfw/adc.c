/*
 * (c) 2004 Trevor Blackwell <tlb@tlb.org>
 *
 */


#include <avr/io.h>
#include <avr/signal.h>
#include <avr/interrupt.h>
#include "adc.h"
#include "debug.h"
#include "uart.h"

static uint16_t adc_current_samples[8][ADC_HIST];
static uint8_t totsamples_0;

static uint8_t admux_vals[]={
  // VREF=AVCC, ext capacitor at AREF
  0x40, 0x41, 0x42, 0x43,
  0x40, 0x41, 0x42, 0x44,
  0x40, 0x41, 0x42, 0x45,
  0x40, 0x41, 0x42, 0x46,
  0x40, 0x41, 0x42, 0x47,
};

#define N_ADMUX_VALS (sizeof(admux_vals)/sizeof(uint8_t))
static uint8_t admuxi;

void
adc_init( void )
{
  // Input, no pullups
  PORTF = 0x00;
  DDRF = 0x00;

  ACSR |= (1<<ACD); // Turn off the analog comparator

  admuxi=0;
  ADMUX = admux_vals[admuxi];

  ADCSRA = 0
    | (1<<ADPS0) | (1<<ADPS1) | (1<<ADPS2) // div by 128
    | (1<<ADEN) // enable ADC
    | (1<<ADIE) // enable interrupts
    | (1<<ADSC); // start first conversion
  // At 16 MHz:
  //  125 kHz ADC clock
  //  Conversions take 13 cycles 
  //  We do 4 per short cycle
  //  So about 2400 cycles/sec
}

/*
  
*/
uint8_t adc_collect_samples(uint16_t *samples, uint8_t *lasts0)
{
  uint8_t i,j;

  cli();
  if (*lasts0 == totsamples_0) {
    sei();
    return 0;
  }
  *lasts0 = totsamples_0;
  for (i=0; i<8; i++) {
    uint16_t tot=0;
    for (j=0; j<ADC_HIST; j++) {
      tot+=adc_current_samples[i][j];
    }
    samples[i] = tot;
  }
  sei();
  return 1;
}

#ifdef DEBUG
#define MAX_ADC_LOG 128
static uint8_t n_adc_log;
static uint16_t adc_log[MAX_ADC_LOG];
#endif

SIGNAL( SIG_ADC )
{
  uint8_t adci = ADMUX & 0x7;
  uint8_t i;

  for (i=0; i<ADC_HIST-1; i++) {
    adc_current_samples[adci][i]=adc_current_samples[adci][i+1];
  }
  adc_current_samples[adci][ADC_HIST-1] = ADCW;
#if 1 && defined(DEBUG)
  if (adci==1 && n_adc_log<MAX_ADC_LOG-3) {
    adc_log[n_adc_log++]=adc_current_samples[adci][ADC_HIST-1];
  }
#endif

  admuxi++;
  if (admuxi>=N_ADMUX_VALS) {
    admuxi=0;
  }
  
  if ((admux_vals[admuxi]&7)==0) totsamples_0++;
  ADMUX = admux_vals[admuxi];

  ADCSRA |= (1<<ADSC); // Start next conv
}

void dump_adc_log()
{
  uint8_t i;
  uint8_t don=n_adc_log;
  if (don>0) {
    uart_tx_uint8('l');
    uart_tx_uint8(don);
    for (i=0; i<don; i++) {
      uart_tx_uint16(adc_log[i]);
    }
    n_adc_log=0;
  }
}
