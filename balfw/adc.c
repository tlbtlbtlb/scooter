/*
  ADC conversion for the balancing robot project

  COPYRIGHT
*/

#include <avr/io.h>
#include <avr/signal.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include "adc.h"
#include "uart.h"
#include "hwdefs.h"

/*
  Define the order in which we read ADC inputs. The point is that we need to sample
  things like pitch_rate faster than knob1.
  
  If an ADCNO isn't listed here, it won't get sampled, and you will be
  disappointed when you try to read it.

  Danger: adcpattern and adc_config are PROGMEM, so you have to use pgm_read_byte
*/

static uint8_t adcpattern[] PROGMEM = {

#ifdef __BAL__
  ADCNO_BAT_VOLTAGE, ADCNO_PITCH_RATE, ADCNO_FWD_ACCEL, ADCNO_YAW_RATE, ADCNO_STEERING,
#endif

#ifdef __BAL2__
  ADCNO_BAT_VOLTAGE, ADCNO_PITCH_RATE, ADCNO_FWD_ACCEL, ADCNO_YAW_RATE, ADCNO_STEERING,
  ADCNO_BAT_VOLTAGE, ADCNO_PITCH_RATE, ADCNO_FWD_ACCEL, ADCNO_YAW_RATE, ADCNO_KNOB1, ADCNO_LMIS,
  ADCNO_BAT_VOLTAGE, ADCNO_PITCH_RATE, ADCNO_FWD_ACCEL, ADCNO_YAW_RATE, ADCNO_STEERING, ADCNO_RMIS, 
  ADCNO_BAT_VOLTAGE, ADCNO_PITCH_RATE, ADCNO_FWD_ACCEL, ADCNO_YAW_RATE, ADCNO_KNOB2, ADCNO_B1IS,
  ADCNO_BAT_VOLTAGE, ADCNO_PITCH_RATE, ADCNO_FWD_ACCEL, ADCNO_YAW_RATE, ADCNO_STEERING, ADCNO_B2IS,
  ADCNO_BAT_VOLTAGE, ADCNO_PITCH_RATE, ADCNO_FWD_ACCEL, ADCNO_YAW_RATE, ADCNO_KNOB3, ADCNO_LFOOTPAD,
  ADCNO_BAT_VOLTAGE, ADCNO_PITCH_RATE, ADCNO_FWD_ACCEL, ADCNO_YAW_RATE, ADCNO_STEERING, ADCNO_RFOOTPAD,
#endif

#ifdef __BAL3__
  ADCNO_BAT_VOLTAGE, ADCNO_PITCH_RATE, ADCNO_YAW_RATE, ADCNO_RIGHT_ACCEL, ADCNO_FWD_ACCEL, ADCNO_STEERING, ADCNO_BATIS, ADCNO_PWMTEMPA,
  ADCNO_BAT_VOLTAGE, ADCNO_PITCH_RATE, ADCNO_YAW_RATE, ADCNO_RIGHT_ACCEL, ADCNO_FWD_ACCEL, ADCNO_STEERING, ADCNO_LMIS, ADCNO_PWMTEMPB,
  ADCNO_BAT_VOLTAGE, ADCNO_PITCH_RATE, ADCNO_YAW_RATE, ADCNO_RIGHT_ACCEL, ADCNO_FWD_ACCEL, ADCNO_STEERING, ADCNO_RMIS, ADCNO_PWMTEMPC,
  ADCNO_BAT_VOLTAGE, ADCNO_PITCH_RATE, ADCNO_YAW_RATE, ADCNO_RIGHT_ACCEL, ADCNO_FWD_ACCEL, ADCNO_STEERING, ADCNO_KNOB1, ADCNO_PWMTEMPD,
  ADCNO_BAT_VOLTAGE, ADCNO_PITCH_RATE, ADCNO_YAW_RATE, ADCNO_RIGHT_ACCEL, ADCNO_FWD_ACCEL, ADCNO_STEERING, ADCNO_KNOB2, ADCNO_KNOB3,
#endif

#ifdef __BAL4__
  ADCNO_BAT_VOLTAGE, ADCNO_PITCH_RATE, ADCNO_FWD_ACCEL, ADCNO_BATIS, ADCNO_LMIS, ADCNO_KNOB1,
  ADCNO_BAT_VOLTAGE, ADCNO_PITCH_RATE, ADCNO_FWD_ACCEL, ADCNO_BATIS, ADCNO_LMIS, ADCNO_KNOB2,
  ADCNO_BAT_VOLTAGE, ADCNO_PITCH_RATE, ADCNO_FWD_ACCEL, ADCNO_BATIS, ADCNO_LMIS, ADCNO_KNOB3,
#endif
};
#define N_ADCPATTERN (sizeof(adcpattern)/sizeof(uint8_t))

static volatile uint16_t adc_current_samples[ADCNO_END][ADC_HIST];
uint8_t volatile adc_samplecount[ADCNO_END];

/* For detailed ADC value logging */
//#define MAX_ADC_LOG 1024

struct adc_config_t {
  uint8_t admux;
#if !defined(__BAL__)
  uint8_t extramux;
#endif  
};

/*
  This defines how the ADCs are connected. Note: the values of the
  enum ADCNO_* don't correspond to the Atmel ADC pin numbers! Instead,
  we use those numbers as an index into this table to get both the
  value for the ADMUX register and the extramux output pins.

  All the ADMUX values are 0x40+(adc_num), which sets the mode to
  "VREF=AVCC, ext capacitor at AREF"

  (This uses gcc's non-ANSI syntax for initializing arrays with element labels)
*/
static struct adc_config_t adc_config[ADCNO_END] PROGMEM = {

#ifdef __BAL__
  [ADCNO_BAT_VOLTAGE]       {0x40} , 
  [ADCNO_PITCH_RATE]         {0x41}, 
  [ADCNO_FWD_ACCEL]          {0x42}, 
  [ADCNO_FWD_ACCEL_TEMP]     {0x43},
  [ADCNO_STEERING]           {0x44}, 
  [ADCNO_YAW_RATE]           {0x46}, 
#endif

#ifdef __BAL2__
  [ADCNO_BAT_VOLTAGE] {0x40, 0},
  [ADCNO_PITCH_RATE] {0x41, 0},
  [ADCNO_FWD_ACCEL] {0x42, 0},
  [ADCNO_FWD_ACCEL_TEMP] {0x43, 0},
  [ADCNO_STEERING] {0x44, 0},
  [ADCNO_YAW_RATE] {0x45, 0},
  [ADCNO_KNOB1] {0x46, 0},
  [ADCNO_LFOOTPAD] {0x47, 0},
  [ADCNO_RFOOTPAD] {0x47, 1},
  [ADCNO_RMIS] {0x47, 2},
  [ADCNO_LMIS] {0x47, 3},
  [ADCNO_KNOB2] {0x47, 4},
  [ADCNO_KNOB3] {0x47, 5},
  [ADCNO_B1IS] {0x47, 6},
  [ADCNO_B2IS] {0x47, 7},
#endif

#ifdef __BAL3__
  [ADCNO_BAT_VOLTAGE] {0x40, 0},
  [ADCNO_PITCH_RATE] {0x41, 0},
  [ADCNO_YAW_RATE] {0x42, 0},
  [ADCNO_ROLL_RATE] {0x43, 0},
#ifdef __JORDANWAY__
  [ADCNO_FWD_ACCEL] {0x44, 0},
  [ADCNO_RIGHT_ACCEL] {0x45, 0},
#else
  [ADCNO_RIGHT_ACCEL] {0x44, 0},
  [ADCNO_FWD_ACCEL] {0x45, 0},
#endif

  [ADCNO_LFOOTPAD1] {0x46, 0},
  [ADCNO_RFOOTPAD1] {0x46, 1},
  [ADCNO_LFOOTPAD2] {0x46, 2},
  [ADCNO_RFOOTPAD2] {0x46, 3},
  [ADCNO_PWMTEMPA] {0x46, 4},
  [ADCNO_PWMTEMPB] {0x46, 5},
  [ADCNO_PWMTEMPC] {0x46, 6},
  [ADCNO_PWMTEMPD] {0x46, 7},

  [ADCNO_KNOB1] {0x47, 0},
  [ADCNO_KNOB2] {0x47, 1},
  [ADCNO_LMIS] {0x47, 2},
  [ADCNO_RMIS] {0x47, 3},
  [ADCNO_KNOB3] {0x47, 4},
  [ADCNO_STEERING] {0x47, 5},
  [ADCNO_SG2] {0x47, 6},
  [ADCNO_BATIS] {0x47, 7},
#endif

#ifdef __BAL4__
  [ADCNO_BAT_VOLTAGE] {0x40, 0},
  [ADCNO_PITCH_RATE] {0x41, 0},
  [ADCNO_RIGHT_ACCEL] {0x44, 0},
  [ADCNO_FWD_ACCEL] {0x45, 0},

  [ADCNO_KNOB1] {0x47, 0},
  [ADCNO_KNOB2] {0x47, 1},
  [ADCNO_BATIS] {0x47, 2},
  [ADCNO_KNOB3] {0x47, 4},
  [ADCNO_LMIS] {0x47, 7},
#endif

};


static volatile uint8_t adcpatterni;   // The current index into adcpattern
static volatile uint8_t adcpattern_loops;
static volatile uint8_t adci;          // The ADCNO_* value of the currently active conversion.
                              // index into ADC_CONFIG and adc_current_samples

void
start_conversion()
{
  adci = pgm_read_byte(&adcpattern[adcpatterni]);
#if !defined(__BAL__)
  set_extramux(pgm_read_byte(&adc_config[adci].extramux));
#endif
  ADMUX = pgm_read_byte(&adc_config[adci].admux);
  ADCSRA |= (1<<ADSC); // Start next conv
}

void
adc_init()
{
  // PORTF is input, no pullups
  PORTF = 0x00;
  DDRF = 0x00;

  ACSR |= (1<<ACD); // Turn off the analog comparator

  ADCSRA = 0
    | (1<<ADPS0) | (1<<ADPS1) | (1<<ADPS2) // div by 128
    | (1<<ADEN) // enable ADC
    | (1<<ADIE) // enable interrupts
    ;
  // At 14.7 MHz:
  //  115 kHz ADC clock
  //  Conversions take 13 cycles 
  //  We do 4 per short cycle
  //  So about 2211 cycles/sec

  adcpatterni=0;
  start_conversion();
}

void
adc_disable()
{
  ADCSRA &= ~(1<<ADIE);
}

/*
  Collect the most recent samples, if any, into samples. Return 1 if there were new samples,
  or 0 if not. Pass a pointer to last_samplecount to keep track of the caller's last sample sequence number,
  so we know if there have been more samples since the last time he called it.
*/
uint8_t adc_collect_samples(uint16_t *samples, uint8_t *last_samplecount, uint8_t whichadc)
{
  uint8_t i,j;

  cli();
  if (*last_samplecount == adc_samplecount[whichadc]) {
    sei();
    return 0;
  }
  *last_samplecount = adc_samplecount[whichadc];
  for (i=0; i<ADCNO_END; i++) {
    uint16_t tot=0;
    for (j=0; j<ADC_HIST; j++) {
      tot+=adc_current_samples[i][j];
    }
    samples[i] = tot;
  }
  sei();
  return 1;
}

#ifdef MAX_ADC_LOG
static uint16_t adc_log_ptr;
static uint16_t adc_log[MAX_ADC_LOG];
#endif

/*
  The ADC completion interrupt. Store the sample in the adc_current_samples shift register
  and start the next conversion.
*/
SIGNAL( SIG_ADC )
{
  uint8_t i;

  for (i=0; i<ADC_HIST-1; i++) {
    adc_current_samples[adci][i]=adc_current_samples[adci][i+1];
  }
  adc_current_samples[adci][ADC_HIST-1] = ADCW;
  adc_samplecount[adci]++;

#ifdef MAX_ADC_LOG
  adc_log[adc_log_ptr] = adc_current_samples[adci][ADC_HIST-1] | (adci<<10);
  adc_log_ptr++;
  if (adc_log_ptr >= MAX_ADC_LOG) adc_log_ptr=0;
#endif

  adcpatterni++;
  if (adcpatterni >= N_ADCPATTERN) {
    adcpatterni=0;
    adcpattern_loops++;
  }
  start_conversion();
}

/*
  Send logged ADC readings to host. For analog debugging, to measure noise before we
  average the ADC_HIST samples together.
*/
void dump_adc_log()
{
#ifdef MAX_ADC_LOG

  for (uint16_t i=0; i<MAX_ADC_LOG; ) {
    while (!uart_tx_empty()) {}

    pkt_tx_start();
    pkt_tx_uint8('l');
    pkt_tx_uint16(i);
    for (uint8_t j=0; j<16; j++) {
      pkt_tx_uint16(adc_log[(i+adc_log_ptr) % MAX_ADC_LOG]);
      i++;
    }
    pkt_tx_end();
  }
#endif
}

/*
  Run the ADC for a while, do flush any initial power-up transients
 */
void adc_stabilize()
{
  while (1) {
    if (adcpattern_loops >= ADC_HIST+2) break;
  }  
}    

