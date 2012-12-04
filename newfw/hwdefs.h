/*
  Hardware definitions for the BAL and BAL2 controller boards.

  Copyright (c) 2005 Trevor Blackwell <tlb@tlb.org>
*/

#ifndef _SCOOTER_BAL_IOBITS_H
#define _SCOOTER_BAL_IOBITS_H

#include "balconf.h"

#include <iobits.h>

// We use the OC outputs in 9-bit mode
#define OCR1_MAX 511

/*
  Internal numbers for ADC. Note: these don't correspond to the
  numbering of the ADCn pins! (We have more of these than ADC pins
  anyway, so ADC7 is further multiplexed) The mapping to various
  sources is defined in adc.c
*/
enum {

  ADCNO_YAW_RATE,   
  ADCNO_YAW_VREF,   
  ADCNO_YAW_TEMP,   
  ADCNO_YAW_AX,     
  ADCNO_YAW_AY,     
  ADCNO_PITCH_RATE, 
  ADCNO_PITCH_VREF, 
  ADCNO_PITCH_TEMP, 

  ADCNO_PITCH_AY,   
  ADCNO_PITCH_AX,   
  ADCNO_ROLL_RATE,  
  ADCNO_ROLL_VREF,  
  ADCNO_ROLL_TEMP,  
  ADCNO_ROLL_AX,    
  ADCNO_ROLL_AY,    
  ADCNO_BAT_VOLTAGE,


  ADCNO_BATIS,
  ADCNO_RMIS,
  ADCNO_LMIS,
  ADCNO_SG1,
  ADCNO_SG2,

  ADCNO_KNOB1,
  ADCNO_KNOB2,
  ADCNO_KNOB3,
  
  ADCNO_LFOOTPAD1,
  ADCNO_RFOOTPAD1,
  ADCNO_LFOOTPAD2,
  ADCNO_RFOOTPAD2,

  ADCNO_END
};

// -A--------------------------------------------------------------------

DEFBITNOT(ads8344_cs1, A, 4);
DEFBITNOT(ads8344_cs2, A, 5);

// -B--------------------------------------------------------------------
DEFBIT(l_pwm_oc, B, 6);      // Output compare signals, which we use in PWM mode.
DEFBIT(r_pwm_oc, B, 5);
DEFBIT(ads8344_dclk, B, 1);
DEFBIT(ads8344_di, B, 2);
DEFBIT(ads8344_dout, B, 3);

// Set the PWM values between 0 and OCR1_MAX
static inline void set_l_pwm(uint16_t x) { OCR1B = x; }
static inline void set_r_pwm(uint16_t x) { OCR1A = x; }


// -C--------------------------------------------------------------------
DEFBIT(l_pwm_fwd, C, 0);     // enables left fwd PWM when the output compare is high
DEFBIT(l_pwm_rev, C, 1);     // ""      ""   rev ""
DEFBIT(r_pwm_fwd, C, 2);
DEFBIT(r_pwm_rev, C, 3);
DEFBIT(accel_st, C, 4);      // activates self test mode on the accelerometer
DEFBITNOT(l_pwm_disable, C, 5);
DEFBITNOT(r_pwm_disable, C, 6);


// -D--------------------------------------------------------------------
DEFBIT(led1, D, 7);          // LEDs on board
DEFBIT(led2, D, 6);

// -E--------------------------------------------------------------------
DEFBIT(extramux2, E, 7);     // Controls the 4051 mux that expands ADC7
DEFBIT(extramux1, E, 6);
DEFBIT(extramux0, E, 5);
DEFBITGROUP(extramux, E, 5, 3);
DEFBITNOT(beeper, E, 2);        // The beeper on the console

#endif
