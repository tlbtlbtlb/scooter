/*
  Hardware definitions for the BAL and BAL2 controller boards.

  COPYRIGHT
*/

#ifndef _SCOOTER_BAL_IOBITS_H
#define _SCOOTER_BAL_IOBITS_H

#include "balconf.h"

#include "iobits.h"

// We use the OC outputs in 9-bit mode
#define OCR1_MAX 511

/*
  Internal numbers for ADC. Note: these don't correspond to the
  numbering of the ADCn pins! (We have more of these than ADC pins
  anyway, so ADC7 is further multiplexed) The mapping to various
  sources is defined in adc.c
*/
enum {

#if defined(__BAL__)

  ADCNO_BAT_VOLTAGE,    // vbatt/10 from left OSMC pin 3
  ADCNO_PITCH_RATE,     // from gyro 1
  ADCNO_FWD_ACCEL,      // from the filtered output of the ADXL105
  ADCNO_FWD_ACCEL_TEMP, // temperature sensor of above
  ADCNO_YAW_RATE,       // from gyro 2
  ADCNO_STEERING,       // steering input from dashboard

#elif defined(__BAL2__)

  ADCNO_BAT_VOLTAGE,   // vbatt/10 from left OSMC pin 3
  ADCNO_PITCH_RATE,     // from gyro 1
  ADCNO_FWD_ACCEL,      // from the filtered output of the ADXL105
  ADCNO_FWD_ACCEL_TEMP, // temperature sensor of above
  ADCNO_YAW_RATE,       // from gyro 2
  ADCNO_STEERING,       // steering input from dashboard
  ADCNO_KNOB1,          // knob 1 on dashboard
  ADCNO_LFOOTPAD,       // Left pressure sensor
  ADCNO_RFOOTPAD,
  ADCNO_LMIS,           // Left motor current sensor
  ADCNO_RMIS,
  ADCNO_KNOB2,
  ADCNO_KNOB3,
  ADCNO_B1IS,           // Battery 1 current sensor
  ADCNO_B2IS,


#elif defined(__BAL3__)

  ADCNO_BAT_VOLTAGE,
  ADCNO_PITCH_RATE,
  ADCNO_YAW_RATE,
  ADCNO_ROLL_RATE,
  ADCNO_RIGHT_ACCEL,
  ADCNO_FWD_ACCEL,

  ADCNO_LFOOTPAD1,
  ADCNO_RFOOTPAD1,
  ADCNO_LFOOTPAD2,
  ADCNO_RFOOTPAD2,
  ADCNO_PWMTEMPA,
  ADCNO_PWMTEMPB,
  ADCNO_PWMTEMPC,
  ADCNO_PWMTEMPD,

  ADCNO_BATIS,
  ADCNO_SG2,
  ADCNO_STEERING,
  ADCNO_KNOB3,
  ADCNO_RMIS,
  ADCNO_LMIS,
  ADCNO_KNOB2,
  ADCNO_KNOB1,

#elif defined(__BAL4__)

  ADCNO_BAT_VOLTAGE,
  ADCNO_PITCH_RATE,
  ADCNO_RIGHT_ACCEL,
  ADCNO_FWD_ACCEL,

  ADCNO_BATIS,
  ADCNO_KNOB3,
  ADCNO_LMIS,
  ADCNO_KNOB2,
  ADCNO_KNOB1,

#endif

  ADCNO_END
};


DEFBIT(l_pwm_fwd, C, 0);     // enables left fwd PWM when the output compare is high
DEFBIT(l_pwm_rev, C, 1);     // ""      ""   rev ""
DEFBIT(r_pwm_fwd, C, 2);
DEFBIT(r_pwm_rev, C, 3);
DEFBIT(accel_st, C, 4);      // activates self test mode on the accelerometer

#if defined(__BAL4__)
  DEFBITNOT(l_pwm_disable, C, 5);
#elif defined(__BAL3__)
  DEFBITNOT(l_pwm_disable, C, 5);
  DEFBITNOT(r_pwm_disable, C, 6);
#elif defined(__BAL2__)
  DEFBIT(l_pwm_disable, C, 5); // disables the low part of the PWM signal, letting it freewheel
  DEFBIT(r_pwm_disable, C, 6);
#endif

#if defined(__BAL4__)
  DEFBITNOT(led1, D, 7);
  DEFBITNOT(led2, D, 6);
  DEFBITNOT(led3, D, 5);
#else
  DEFBIT(led1, D, 7);
  DEFBIT(led2, D, 6);
#endif

DEFBIT(extramux2, E, 7);     // Controls the 4051 mux that expands ADC7
DEFBIT(extramux1, E, 6);
DEFBIT(extramux0, E, 5);
DEFBITGROUP(extramux, E, 5, 3);
DEFBIT(beeper, E, 2);        // The beeper on the console

DEFBIT(l_pwm_oc, B, 6);      // Output compare signals, which we use in PWM mode.
DEFBIT(r_pwm_oc, B, 5);

#if defined(__BAL3__)
  // NOTE: this was wrong in the board submitted as scooter3
  // I had separate SEL lines but a single /OE line
  DEFBITGROUP(enc_data, A, 0, 8);
  DEFBIT(enc_oe1_not, B, 7);
  DEFBIT(enc_oe2_not, B, 0);
  DEFBIT(enc_rst_not, B, 2);
  DEFBIT(enc_sel1, B, 3);
  DEFBIT(enc_sel2, B, 4);
#endif

#if defined(__BAL2__)
#define R_MOTOR_POLARITY -
#define L_MOTOR_POLARITY +
#else
#define R_MOTOR_POLARITY +
#define L_MOTOR_POLARITY +
#endif

// Set the PWM values between 0 and OCR1_MAX
static inline void set_l_pwm(uint16_t x) { OCR1B = x; }
static inline void set_r_pwm(uint16_t x) { OCR1A = x; }

#endif
