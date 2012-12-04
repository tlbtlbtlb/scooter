/*
Copyright (c) 2002, Trevor Blackwell. All rights reserved.
The author can be reached at tlb@tlb.org.

(This is the MIT License for open source software with capitalization
fixed.)

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions: 

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

The software is provided "as is", without warranty of any kind,
express or implied, including but not limited to the warranties of
merchantability, fitness for a particular purpose and noninfringement.
In no event shall the authors or copyright holders be liable for any
claim, damages or other liability, whether in an action of contract,
tort or otherwise, arising from, out of or in connection with the
software or the use or other dealings in the software.

*/

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <math.h>

#include <iobits.h>
#include "uart.h"
#include "string.h"
#include "balconf.h"
#include "hwdefs.h"
#include "dsp.h"

#define MOMENTS_PER_SEC 5.0
// in moments
#define APPROX_TIMESTEP 0.008

#define LM_PWM_SIGN +
#define RM_PWM_SIGN -
#define LM_IS_SIGN +
#define RM_IS_SIGN -

enum {
  MODE_INITIAL,
  MODE_PENDING,
  MODE_STARTUP,
  MODE_RUNNING,
  MODE_TIPPED,
  MODE_HWADJ,
  MODE_HWADJ_DONE,
  MODE_TEST_MOTORS,
  MODE_EXPLORE_MOTORS,
};

uint16_t ticks_since_senderr;
uint16_t ticks_since_beacon;

/* 
   All the state and config variables are defined in balconf.h, so I can get at them externally.
*/
struct bal_config bc;
struct bal_state bs;
struct bal_filters bf;
struct bal_internal bi;


void pkt_tx_dsp_8dot24(dsp_t data)
{
  pkt_tx_uint8((data>>0)&0xff);
  pkt_tx_uint8((data>>8)&0xff);
  pkt_tx_uint8((data>>16)&0xff);
  pkt_tx_uint8((data>>24)&0xff);
}

void pkt_tx_dsp_8dot8(dsp_t data)
{
  pkt_tx_uint8((data>>16)&0xff);
  pkt_tx_uint8((data>>24)&0xff);
}

void pkt_tx_dsp_6dot10(dsp_t data)
{
  pkt_tx_uint8((data>>14)&0xff);
  pkt_tx_uint8((data>>22)&0xff);
}

void pkt_tx_dsp_4dot12(dsp_t data)
{
  pkt_tx_uint8((data>>12)&0xff);
  pkt_tx_uint8((data>>20)&0xff);
}

void pkt_tx_dsp_2dot14(dsp_t data)
{
  pkt_tx_uint8((data>>10)&0xff);
  pkt_tx_uint8((data>>18)&0xff);
}

void
io_init()
{
  /*
    All the IO pins which control the PWM have pull up/down resistors to make their high-Z
    state safe. First, we set the correct (neutral) levels, then we turn them into
    outputs.
   */
  set0_r_pwm_oc();
  set1_r_pwm_disable();
  set0_r_pwm_rev();
  set0_r_pwm_fwd();

  set0_l_pwm_oc();
  set1_l_pwm_disable();
  set0_l_pwm_rev();
  set0_l_pwm_fwd();

  set0_accel_st();

  setout_r_pwm_oc();
  setout_r_pwm_disable();
  setout_r_pwm_rev();
  setout_r_pwm_fwd();

  setout_l_pwm_oc();
  setout_l_pwm_disable();
  setout_l_pwm_rev();
  setout_l_pwm_fwd();

  setout_accel_st();

  set0_led1();
  set0_led2();
  set0_beeper();
  set_extramux(0);

  setout_led1();
  setout_led2();
  setout_beeper();
  setout_extramux();

}


dsp_t filter2_run(struct filter2_coeff *c, struct filter2_state *s, dsp_t v) 
{
  for (uint8_t i=0; i<2; i++) {
    s->x[i]=s->x[i+1];
    s->y[i]=s->y[i+1];
  }
  s->x[2] = v;
    
  dsp_t newy=0;
  for (uint8_t i=0; i<=2; i++) {
    newy += dspmul(s->x[i], c->bcoeff[i]);
  }
    
  for (uint8_t i=0; i<2; i++) {
    newy -= dspmul(s->y[i], c->acoeff[i]);
  }
    
  s->y[2] = newy;
  return newy;
}

void filter2_clear(struct filter2_state *s)
{
  s->x[0] = s->x[1] = s->x[2] = s->y[0] = s->y[1] = s->y[2];
}

dsp_t filter2_deriv(struct filter2_state *s)
{
  return (s->y[2] - s->y[1]) * (int32_t)(1.0/APPROX_TIMESTEP);
}


/* 
   Set the PWM values for the motors. +1.0 is full forward, -1.0 is full reverse.
*/


void set_motors(dsp_t left, dsp_t right)
{
  int16_t deadband=bc.deadband;
  if (deadband>20) deadband=20;
  if (deadband<0) deadband=0;

  static int8_t lefti_sign, righti_sign;

  dsp_t lim=OCR1_MAX-deadband-2;
  int16_t lefti = LM_PWM_SIGN dsplim(left/(DSP_ONE/(OCR1_MAX+1)), -lim, lim);
  int16_t righti = RM_PWM_SIGN dsplim(right/(DSP_ONE/(OCR1_MAX+1)), -lim, lim);

  /* 
    It's OK to update these asynchronously with the PWM clock, because OCRn[AB] is double buffered 
    and only changes when the counter reaches TOP 
  */

  if (lefti  <= -bc.hysteresis && lefti_sign  >= 0) lefti_sign  = -1;
  if (lefti  >= +bc.hysteresis && lefti_sign  <= 0) lefti_sign  = +1;
  if (righti <= -bc.hysteresis && righti_sign >= 0) righti_sign = -1;
  if (righti >= +bc.hysteresis && righti_sign <= 0) righti_sign = +1;

  cli();
  if (lefti<0 && lefti_sign<0) {
    /* For reverse, set PWM_FWD=0, PWM_DISABLE=0, PWM_REV=1.
       The order here is carefully chosen to avoid glitches.
       Note also that we have interrupts disabled so there's no chance of dead time.
    */
    set0_l_pwm_fwd();
    set0_l_pwm_disable();
    set1_l_pwm_rev();
    set_l_pwm(-lefti+deadband);
  } 
  else if (lefti>0 && lefti_sign>0) {
    // For forward, set PWM_FWD=1, PWM_DISABLE=0, PWM_REV=0
    set0_l_pwm_rev();
    set0_l_pwm_disable();
    set1_l_pwm_fwd();
    set_l_pwm(lefti+deadband);
  }
  else {
    set0_l_pwm_rev();
    set0_l_pwm_fwd();
    set0_l_pwm_disable();
    set_l_pwm(0);
  }
  
  if (righti<0 && righti_sign<0) {
    set0_r_pwm_fwd();
    set0_r_pwm_disable();
    set1_r_pwm_rev();
    set_r_pwm(OCR1_MAX+righti-deadband);
  } 
  else if (righti>0 && righti_sign>0) {
    set0_r_pwm_rev();
    set0_r_pwm_disable();
    set1_r_pwm_fwd();
    set_r_pwm(OCR1_MAX-righti-deadband);
  }
  else {
    set0_r_pwm_rev();
    set0_r_pwm_fwd();
    set0_r_pwm_disable();
    set_r_pwm(OCR1_MAX);
  }

  sei();
}

/*
  Different from setting the motors to zero speed, this disables both legs of the H bridge
  so the motor can freewheel. This is a better thing to do in an emergency than locking
  the wheel by calling set_motors(0.0, 0.0). 
*/

void set_motor_idle()
{
  set_l_pwm(0);
  set_r_pwm(0);

  set1_r_pwm_disable();
  set0_r_pwm_rev();
  set0_r_pwm_fwd();

  set1_l_pwm_disable();
  set0_l_pwm_rev();
  set0_l_pwm_fwd();
}

// ----------------------------------------------------------------------

uint16_t adc_samples[ADCNO_END];
uint8_t adc_last_waittime;
uint8_t spitrans_fail;

inline uint8_t spitrans(uint8_t outbyte)
{
  if (spitrans_fail) return 0;
  uint8_t waittime=0;
  SPDR = outbyte;
  while (!(SPSR & (1<<SPIF))) {
    waittime++;
    if (waittime>250) {
      spitrans_fail++;
      break;
    }
  }
  adc_last_waittime = waittime;
  return SPDR;
}

inline uint16_t spitransw(uint8_t outh, uint8_t outl)
{
  uint8_t inh=spitrans(outh);
  uint8_t inl=spitrans(outl);
  return ((uint16_t)inh<<8) | inl;
}

inline uint16_t read_adc(uint8_t muxi)
{
  /* The ordering of mux bits is stupid */
  spitrans((1<<7) | ((muxi&6)<<3) | ((muxi&1)<<6) | 0x07);
  
  uint8_t in2=spitrans(0);
  uint8_t in1=spitrans(0);
  uint8_t in0=spitrans(0);

  return (in0>>7) | (in1<<1) | (in2<<9);
}

struct ads8344_config_t {
  uint8_t adcno;
  uint8_t chipno;
  uint8_t muxno;
};
struct ads8344_config_t ads8344_config[] PROGMEM = {
  {ADCNO_YAW_RATE,    1, 0},
  {ADCNO_YAW_VREF,    1, 1},
  {ADCNO_YAW_TEMP,    1, 2},
  {ADCNO_YAW_AX,      1, 3},
  {ADCNO_YAW_AY,      1, 4},
  {ADCNO_PITCH_RATE,  1, 5},
  {ADCNO_PITCH_VREF,  1, 6},
  {ADCNO_PITCH_TEMP,  1, 7},

  {ADCNO_PITCH_AY,    2, 0},
  {ADCNO_PITCH_AX,    2, 1},
  {ADCNO_ROLL_RATE,   2, 2},
  {ADCNO_ROLL_VREF,   2, 3},
  {ADCNO_ROLL_TEMP,   2, 4},
  {ADCNO_ROLL_AX,     2, 5},
  {ADCNO_ROLL_AY,     2, 6},
  {ADCNO_BAT_VOLTAGE, 2, 7},
};

struct onboardadc_config_t {
  uint8_t adcno;
  uint8_t muxno;
  uint8_t extramux;
};
struct onboardadc_config_t onboardadc_config[] PROGMEM = {
  {ADCNO_BATIS,   (1<<REFS0)|(1<<ADLAR)|5, 0},
  {ADCNO_LMIS,    (1<<REFS0)|(1<<ADLAR)|3, 0},
  {ADCNO_RMIS,    (1<<REFS0)|(1<<ADLAR)|4, 0},
  {ADCNO_SG1,     (1<<REFS0)|(1<<ADLAR)|1, 0},
  {ADCNO_SG2,     (1<<REFS0)|(1<<ADLAR)|2, 0},

  {ADCNO_KNOB1,     (1<<REFS0)|(1<<ADLAR)|7, 0},
  {ADCNO_KNOB2,     (1<<REFS0)|(1<<ADLAR)|7, 1},
  {ADCNO_KNOB3,     (1<<REFS0)|(1<<ADLAR)|7, 2},
  //{ADCNO_LFOOTPAD1, (1<<REFS0)|(1<<ADLAR)|7, 3},
  //{ADCNO_RFOOTPAD1, (1<<REFS0)|(1<<ADLAR)|7, 4},
  //{ADCNO_LFOOTPAD2, (1<<REFS0)|(1<<ADLAR)|7, 5},
  //{ADCNO_RFOOTPAD2, (1<<REFS0)|(1<<ADLAR)|7, 6},

};

uint8_t onboardadc_phase;
uint8_t onboardadc_wraps;

void read_adcs()
{
  for (uint8_t i=0; i<sizeof(ads8344_config)/sizeof(ads8344_config[0]); i++) {
    uint8_t adcno = pgm_read_byte(&ads8344_config[i].adcno);
    uint8_t chipno = pgm_read_byte(&ads8344_config[i].chipno);
    uint8_t muxno = pgm_read_byte(&ads8344_config[i].muxno);
    
    if (chipno==1) {
      set1_ads8344_cs1();
      adc_samples[adcno] = read_adc(muxno);
      set0_ads8344_cs1();
    }
    else if (chipno==2) {
      set1_ads8344_cs2();
      adc_samples[adcno] = read_adc(muxno);
      set0_ads8344_cs2();
    }
  }

  // Read one and start next
  if (ADCSRA & (1<<ADIF)) {
    uint8_t adcno = pgm_read_byte(&onboardadc_config[onboardadc_phase].adcno);

    adc_samples[adcno] = ADCW;

    onboardadc_phase++;
    if (onboardadc_phase >= sizeof(onboardadc_config)/sizeof(onboardadc_config[0])) {
      onboardadc_phase=0;
      onboardadc_wraps++;
    }

    set_extramux(pgm_read_byte(&onboardadc_config[onboardadc_phase].extramux));
    ADMUX = pgm_read_byte(&onboardadc_config[onboardadc_phase].muxno);
    ADCSRA |= (1<<ADSC) | (1<<ADIF); // clear flag, start next conv
  }
}

void
adc_init()
{
  set0_ads8344_cs1();
  set0_ads8344_cs2();
  set0_ads8344_dclk();
  set0_ads8344_di();
  set0_ads8344_dout();

  setout_ads8344_cs1();
  setout_ads8344_cs2();
  setout_ads8344_dclk();
  setout_ads8344_di();
  setin_ads8344_dout();

  DDRB |= 0x01;   // the SS pin (PB0) must be an output or it messes up the SPI

  SPCR = (1<<SPE)|(0<<DORD)|(1<<MSTR)|(0<<CPOL)|(0<<CPHA)|(1<<SPR0);
  SPSR = (1<<SPI2X);

  // port F is ADC, so set inputs with pullups off
  PORTF = 0x00;
  DDRF = 0x00;

  ACSR |= (1<<ACD); // Turn off the analog comparator
  ADCSRA = 0
    | (1<<ADPS0) | (1<<ADPS1) | (1<<ADPS2) // div by 128
    | (1<<ADEN) // enable ADC
    ;
  // At 14.7 MHz:
  //  115 kHz ADC clock
  //  Conversions take 13 cycles 

  onboardadc_phase=0;
  set_extramux(pgm_read_byte(&onboardadc_config[onboardadc_phase].extramux));
  ADMUX = pgm_read_byte(&onboardadc_config[onboardadc_phase].muxno);
  ADCSRA |= (1<<ADSC); // Start next conv
  
}


static uint16_t last_balance_tcnt3;

void balance_configure()
{
  memset(&bc, 0, sizeof(bc));

  bc.vp_height=DSPNUM(0.5);

  bc.vel_err_lim=DSPNUM(0.3);
  bc.vel_err_i_lim=DSPNUM(0.6);
  bc.yaw_rate_err_lim=DSPNUM(0.2);
  bc.yaw_rate_err_i_lim=DSPNUM(0.4);
  
  bc.vel_coeff=DSPNUM(0.1);
  bc.vel_i_coeff=DSPNUM(0.0);
#if defined(__SCOOTER8__)
  bc.yaw_rate_coeff=DSPNUM(0.0);
  bc.yaw_rate_i_coeff=DSPNUM(0.0);
#elif defined(__TACKLIND_W0__)
  bc.yaw_rate_coeff=DSPNUM(0.0);
  bc.yaw_rate_i_coeff=DSPNUM(0.0);
#endif

#if defined(__SCOOTER8__)
  bc.pitch_coeff=DSPNUM(2.0);
  bc.pitch_rate_coeff=DSPNUM(0.4);
  bc.pitch_i_coeff=DSPNUM(0.9);
  bc.target_pitch_lim=DSPNUM(0.2);
  bc.current_accel_coeff = DSPNUM(0.10);
  bc.current_vel_coeff = DSPNUM(+0.0);
#elif defined(__TACKLIND_W0__)
  bc.pitch_coeff=DSPNUM(2.0);
  bc.pitch_rate_coeff=DSPNUM(0.75);
  bc.pitch_i_coeff=DSPNUM(0.1);
  bc.target_pitch_lim=DSPNUM(+0.2)
#endif
  bc.pitch_err_i_lim=DSPNUM(0.5);

  bc.yaw_steering_coeff = DSPNUM(+0.5);
  bc.steering_corr = DSPNUM(-0.021);

  bc.hard_fwd_lim=DSPNUM(0.99);   // TEMP
  bc.hard_yaw_lim=DSPNUM(0.30);
  bc.deadband = 6;
  bc.hysteresis = 0;

  // A plausible 0.1 moment filter (assuming 0.008 moments cycle time)
  // We just set these up so the filter outputs won't start from zero, especially the pitch filter.
  bc.fwd_cmd_filter.bcoeff[0]=DSPNUM(0.012924194);
  bc.fwd_cmd_filter.bcoeff[1]=DSPNUM(0.025848388);
  bc.fwd_cmd_filter.bcoeff[2]=DSPNUM(0.012924194);
  bc.fwd_cmd_filter.acoeff[0]=DSPNUM(0.645568847);
  bc.fwd_cmd_filter.acoeff[1]=DSPNUM(-1.593872070);

#if defined(__SCOOTER8__)
  bc.pitch_temp_corr.c0=DSPNUM(+0.028);
  bc.roll_temp_corr.c0=DSPNUM(+0.000);
  bc.yaw_temp_corr.c0=DSPNUM(-0.00);
#elif defined(__TACKLIND_W0__)
  bc.yaw_temp_corr.c0=DSPNUM(-0.000);
  bc.pitch_temp_corr.c0=DSPNUM(-0.440);
  bc.roll_temp_corr.c0=DSPNUM(+0.000);
#endif

  bc.yaw_cmd_filter = bc.fwd_cmd_filter;

  // 0.8 moments
  bc.target_pitch_filter.bcoeff[0]=DSPNUM(+0.00024);
  bc.target_pitch_filter.bcoeff[1]=DSPNUM(+0.00047);
  bc.target_pitch_filter.bcoeff[2]=DSPNUM(+0.00024);
  bc.target_pitch_filter.acoeff[0]=DSPNUM(+0.93940);
  bc.target_pitch_filter.acoeff[1]=DSPNUM(-1.93846);
  bc.target_pitch_filter = bc.fwd_cmd_filter;

  bc.fwd_vel_lim = DSPNUM(+0.3); // meters/sec roughly
  bc.rev_vel_lim = DSPNUM(-0.3);


}

void balance_init()
{
  last_balance_tcnt3=TCNT3;

  memset(&bi, 0, sizeof(bi));
  bi.lmis_bias = 32768;
  bi.rmis_bias = 32768;
  bi.batis_bias = 32768;

  memset(&bs, 0, sizeof(bs));
  bs.mode = MODE_PENDING;
  bs.target_age = 83886080L;

  memset(&bf, 0, sizeof(bf));

  set_motor_idle();
  set0_accel_st();

  if (0) {
    bs.mode = MODE_EXPLORE_MOTORS;
  }
}


uint16_t bal_tickno;            // Counter used to time stick shake oscillator and other things

enum {
  LIMIT_PITCH,
  LIMIT_CMD,
  LIMIT_CURRENT,
  LIMIT_BAT_VOLTAGE,
  LIMIT_WHEELIE,
};

void enforce_limits()
{
  if (bs.mode==MODE_TIPPED) return;

  if (bs.at_limit & (1<<LIMIT_CMD)) {
    if (++bs.limit_cmd_error>400) {
      bs.mode=MODE_TIPPED;
      set_motor_idle();

      pkt_tx_start();
      pkt_tx_str_P(PSTR("!limit fwd_cmd="));
      pkt_tx_uint32_hex(bs.fwd_cmd);
      pkt_tx_str_P(PSTR(" yaw_cmd="));
      pkt_tx_uint32_hex(bs.yaw_cmd);
      pkt_tx_end();
    }
  } else if (bs.limit_cmd_error) {
    bs.limit_cmd_error--;
  }
  if (bs.at_limit & (1<<LIMIT_CURRENT)) {
    if (++bs.limit_current_error>400) {
      bs.mode=MODE_TIPPED;
      set_motor_idle();

      pkt_tx_start();
      pkt_tx_str_P(PSTR("!limit current batis="));
      pkt_tx_uint32_hex(bs.batis);
      pkt_tx_str_P(PSTR(" lmis="));
      pkt_tx_uint32_hex(bs.lmis);
      pkt_tx_str_P(PSTR(" rmis="));
      pkt_tx_uint32_hex(bs.rmis);
      pkt_tx_end();
    }
  } else if (bs.limit_current_error) {
    bs.limit_current_error--;
  }
  if (bs.at_limit & (1<<LIMIT_BAT_VOLTAGE)) {
    if (++bs.limit_bat_voltage_error>250) {
      bs.mode=MODE_TIPPED;
      set_motor_idle();

      pkt_tx_start();
      pkt_tx_str_P(PSTR("!limit bat_voltage="));
      pkt_tx_uint32_hex(bs.bat_voltage);
      pkt_tx_end();
    }
    
  } else if (bs.limit_bat_voltage_error) {
    bs.limit_bat_voltage_error--;
  }
  if (bs.at_limit & (1<<LIMIT_PITCH)) {
    if (++bs.limit_pitch_error>250) {
      bs.mode=MODE_TIPPED;
      set_motor_idle();

      pkt_tx_start();
      pkt_tx_str_P(PSTR("!limit pitch="));
      pkt_tx_uint32_hex(bs.pitch);
      pkt_tx_str_P(PSTR(" neutral_pitch="));
      pkt_tx_uint32_hex(bs.neutral_pitch);
      pkt_tx_end();
    }
  } else if (bs.limit_pitch_error) {
    bs.limit_pitch_error--;
  }
  if (bs.at_limit & (1<<LIMIT_WHEELIE)) {
    if (++bs.limit_wheelie_error>250) {
      bs.mode=MODE_TIPPED;
      set_motor_idle();

      pkt_tx_start();
      pkt_tx_str_P(PSTR("!limit wheelie "));
      pkt_tx_uint32_hex(bs.target_pitch);
      pkt_tx_uint8(' ');
      pkt_tx_uint32_hex(bs.pitch_err);
      pkt_tx_uint8(' ');
      pkt_tx_uint32_hex(bs.pitch);
      pkt_tx_uint8(' ');
      pkt_tx_uint32_hex(bs.speed_est);
      pkt_tx_end();
    }
  } else if (bs.limit_wheelie_error) {
    bs.limit_wheelie_error--;
  }

  if (bs.thermal_mode>=3) {
    bs.mode=MODE_TIPPED;
    set_motor_idle();

    pkt_tx_start();
    pkt_tx_str_P(PSTR("!limit overheat "));
    pkt_tx_uint32_hex(bs.lmis);
    pkt_tx_uint8(' ');
    pkt_tx_uint32_hex(bs.rmis);
    pkt_tx_end();
  }
}


void balance(void)
{
  bs.at_limit=0;

  bal_tickno++;

  read_adcs();
  if (spitrans_fail) {
    pkt_tx_start();
    pkt_tx_str_P(PSTR("!spi_trans_failures "));
    pkt_tx_uint8_hex(spitrans_fail);
    pkt_tx_end();
    spitrans_fail=0;
    return;
  }

  {
    cli();
    // interval = time in seconds since we last updated things.
    uint16_t cur_tcnt3 = TCNT3;
    sei();

    uint16_t ticks = cur_tcnt3 - last_balance_tcnt3;
    last_balance_tcnt3 = cur_tcnt3;

    // TCNT3 runs at 14756000/64 = 230562 Hz. We want 5*DSP_ONE = 83886080, so multiply by 364.
    // Limit to 0.0005 to 0.005 seconds. Typical is 1 mS 
    bs.interval = dsplim((dsp_t)ticks * DSPNUM(MOMENTS_PER_SEC / (CLOCK_SPEED/64.0)),
                         DSPNUM(0.0025), DSPNUM(0.025));

    bs.inverse_interval = DSP_ONE/bs.interval;
  }

  {
    dsp_t new_realtime = bs.realtime + bs.interval;
    bs.tickmask = (new_realtime>>16) ^ (bs.realtime>>16);
    bs.realtime = new_realtime;
  }

#if defined(__SCOOTER8__)
  bs.neutral_pitch = DSPNUM(-0.00);
#elif defined(__TACKLIND_W0__)
  bs.neutral_pitch = DSPNUM(-0.00);
#endif
  bs.neutral_pitch_rate = DSPNUM(0.0);

  // Ramp up softstart over the first second
  if (bs.mode==MODE_STARTUP) {
    bs.softstart = bs.softstart + bs.interval/4;
    if (bs.softstart>=DSPNUM(1.0)) {
      bs.softstart=DSPNUM(1.0);
      bs.mode=MODE_RUNNING;
    }
  }

  // convert 24v => 1
#if defined(__SCOOTER8__)
  bs.bat_voltage =  (dsp_t)adc_samples[ADCNO_BAT_VOLTAGE] * DSPNUM(3.030/65536.0);
#elif defined(__TACKLIND_W0__)
  bs.bat_voltage =  (dsp_t)adc_samples[ADCNO_BAT_VOLTAGE] * DSPNUM(4.200/65536.0);
#endif


  // ADXRS300 rate gyro, with 100K resistor from SUMJ to RATEOUT => convert 1.7857 mv/deg/sec to radians/moment.
  // ADXRS150 rate gyro, with no resistor from SUMJ to RATEOUT => convert 12.5 mv/deg/sec to radians/moment.
  // ADXRS300 rate gyro, with no resistor from SUMJ to RATEOUT => convert 6.25 mv/deg/sec to radians/moment.

  // convert to temp to 0=27 C, 1=127 C, -1=-73C. sensor gives 8.4 mv/degree
  bs.pitch_gyro = ((dsp_t)adc_samples[ADCNO_PITCH_RATE] - (dsp_t)adc_samples[ADCNO_PITCH_VREF])
    * DSPNUM(-(M_PI/180.0) / 12.5e-3 / MOMENTS_PER_SEC * (5.0/65536.0));
  bs.pitch_temp = ((dsp_t)adc_samples[ADCNO_PITCH_TEMP] - (dsp_t)adc_samples[ADCNO_PITCH_VREF]) * DSPNUM(1.0 / (8.4e-3 * 100) * (5.0/65536.0));

  bs.yaw_gyro = ((dsp_t)adc_samples[ADCNO_YAW_RATE] - (dsp_t)adc_samples[ADCNO_YAW_VREF])
    * DSPNUM((M_PI/180.0) / 6.25e-3 / MOMENTS_PER_SEC * (5.0/65536.0));
  bs.yaw_temp = ((dsp_t)adc_samples[ADCNO_YAW_TEMP] - (dsp_t)adc_samples[ADCNO_YAW_VREF]) * DSPNUM(1.0 / (8.4e-3 * 100) * (5.0/65536.0));

  bs.roll_gyro = ((dsp_t)adc_samples[ADCNO_ROLL_VREF] - (dsp_t)adc_samples[ADCNO_ROLL_RATE])
    * DSPNUM((M_PI/180.0) / 12.5e-3 / MOMENTS_PER_SEC * (5.0/65536.0));
  bs.roll_temp = ((dsp_t)adc_samples[ADCNO_ROLL_TEMP] - (dsp_t)adc_samples[ADCNO_ROLL_VREF]) * DSPNUM(1.0 / (8.4e-3 * 100) * (5.0/65536.0));

  bs.steering_sensor = ((dsp_t)adc_samples[ADCNO_SG2] - (dsp_t)32768) * DSPNUM(2.0/65536.0) - bc.steering_corr;

  // Current sensors
  if (bs.tickmask & 0xfffc) { // 64x / moment, 320 Hz
    // L08 current sensors
    // Convert 150A/2v to units of 100A
    bs.lmis =  LM_IS_SIGN ((dsp_t)adc_samples[ADCNO_LMIS] - bi.lmis_bias) * DSPNUM(1.875/32767.0);
    bs.rmis =  RM_IS_SIGN ((dsp_t)adc_samples[ADCNO_RMIS] - bi.rmis_bias) * DSPNUM(1.875/32767.0);
    bs.batis = ((dsp_t)adc_samples[ADCNO_BATIS] - bi.batis_bias) * DSPNUM(1.875/32767.0);

    // These use the onboard ADCs and don't seem to be very high precision.
    if (bs.mode==MODE_INITIAL || bs.mode==MODE_PENDING) {   // should be zero current
      if (adc_samples[ADCNO_LMIS] > bi.lmis_bias) bi.lmis_bias++; else bi.lmis_bias--;
      if (adc_samples[ADCNO_RMIS] > bi.rmis_bias) bi.rmis_bias++; else bi.rmis_bias--;
      if (adc_samples[ADCNO_BATIS] > bi.batis_bias) bi.batis_bias++; else bi.batis_bias--;
    }

    if (bs.batis > DSPNUM(0.8) || bs.batis < DSPNUM(-0.8)) {
      bs.at_limit |= (1<<LIMIT_CURRENT);
    }
  }


  // ADXL322 accelerometer
  // Convert from 700 mv/G into Gs
  bs.x_accel = (32768L - (dsp_t)adc_samples[ADCNO_YAW_AY]) * DSPNUM(5.0 / 0.7 / 65536.0);
  bs.y_accel = ((dsp_t)adc_samples[ADCNO_YAW_AX] - 32768L) * DSPNUM(-5.0 / 0.7 / 65536.0);
  bs.z_accel = (32768L - (dsp_t)adc_samples[ADCNO_PITCH_AY]) * DSPNUM(5.0 / 0.7 / 65536.0);

  if (bs.mode==MODE_INITIAL) return;

  if (bs.mode==MODE_TEST_MOTORS) {
    dsp_t pwm=DSPNUM(+0.15);
    set_motors(pwm, pwm);
    return;
  }

  {
    dsp_t x=bs.pitch_temp;
    dsp_t y=bc.pitch_temp_corr.c0;
    
    y += dspmul(x, bc.pitch_temp_corr.c1);
    x = dspmul(x, bs.pitch_temp);
    y += dspmul(x, bc.pitch_temp_corr.c2);
    x = dspmul(x, bs.pitch_temp);
    y += dspmul(x, bc.pitch_temp_corr.c3);
    
    bs.pitch_gyro_temp_corr = y;
  }
  
  {
    dsp_t x=bs.roll_temp;
    dsp_t y=bc.roll_temp_corr.c0;
    
    y += dspmul(x, bc.roll_temp_corr.c1);
    x = dspmul(x, bs.roll_temp);
    y += dspmul(x, bc.roll_temp_corr.c2);
    x = dspmul(x, bs.roll_temp);
    y += dspmul(x, bc.roll_temp_corr.c3);
    
    bs.roll_gyro_temp_corr = y;
  }
  
  {
    dsp_t x=bs.yaw_temp;
    dsp_t y=bc.yaw_temp_corr.c0;
    
    y += dspmul(x, bc.yaw_temp_corr.c1);
    x = dspmul(x, bs.yaw_temp);
    y += dspmul(x, bc.yaw_temp_corr.c2);
    x = dspmul(x, bs.yaw_temp);
    y += dspmul(x, bc.yaw_temp_corr.c3);
    
    bs.yaw_gyro_temp_corr = y;
  }
  
  if (!bs.pitch_inited) {
    bs.pitch = -bs.y_accel;
    bs.roll = +bs.x_accel;
    bs.pitch_inited=1;
  }

  bs.est_fwd_accel = dspmul(bs.rmis + bs.lmis, bc.current_accel_coeff);


  // First order corrections for pitch and roll from X and Y accelerometers
  {
    dsp_t accel_rate = (-(bs.y_accel - bs.est_fwd_accel) - bs.pitch)/16;
    bs.pitch_rate = bs.pitch_gyro - bs.pitch_gyro_temp_corr + accel_rate - bs.pitch_ltr_corr;
    bs.pitch += dspmul(bs.pitch_rate, bs.interval);
    
    if (accel_rate>0) bs.pitch_ltr_corr--;
    if (accel_rate<0) bs.pitch_ltr_corr++;
  }

  {
    dsp_t accel_rate = (+bs.x_accel - bs.roll)/16;
    bs.roll_rate = bs.roll_gyro - bs.roll_gyro_temp_corr + accel_rate - bs.roll_ltr_corr;
    bs.roll += dspmul(bs.roll_rate, bs.interval);
    
    if (accel_rate>0) bs.roll_ltr_corr--;
    if (accel_rate<0) bs.roll_ltr_corr++;
  }

  bs.yaw_rate = bs.yaw_gyro - bs.yaw_gyro_temp_corr;

  /* If we've tipped past 0.6 radians (45 degrees) abandon hope.
     This happens, for example, when the wheels slip on wet grass or snow and it falls over.
     It's better to give up than keep spinning the wheels trying to right ourselves.
     We probably can't recover anyway.
  */

  // check for tip
  bs.effective_pitch = bs.pitch - bs.neutral_pitch;
  if (bs.mode != MODE_TIPPED && 
      bs.mode != MODE_PENDING &&
      bs.mode != MODE_EXPLORE_MOTORS && 
      (bs.effective_pitch > DSPNUM(0.6) || bs.effective_pitch < -DSPNUM(0.6))) {
    bs.at_limit |= (1<<LIMIT_PITCH);
  }

  // compute vp_vel
  bs.vp_vel = -dspmul(bs.pitch_rate-bs.neutral_pitch_rate, bc.vp_height);
  bs.vp_vel_abs = bs.speed_est + bs.vp_vel;

  dsp_t steering_denom;
  if (bs.vp_vel_abs >= 0) {
    steering_denom = DSPNUM(0.6) + dspmul(bs.vp_vel_abs, DSPNUM(2.0));
  } else {
    steering_denom = DSPNUM(0.6) + dspmul(-bs.vp_vel_abs, DSPNUM(3.0));
  }

  dsp_t steering_mult = (0x40000000L / (steering_denom>>8)) << 10;
  dsp_t steering_rate = dspmul(bs.steering_sensor, steering_mult);

  bs.target_yaw_rate = steering_rate;
  
  bs.yaw_rate_err = dsplim(bs.target_yaw_rate - bs.yaw_rate, 
                           -bc.yaw_rate_err_lim, bc.yaw_rate_err_lim);

  // Below this we can't count on enough voltage to keep ourselves or the PWM drives running.
  if (bs.bat_voltage < DSPNUM(0.6)) {
    bs.at_limit |= (1<<LIMIT_BAT_VOLTAGE);
  }
  
  if (bs.vp_vel_abs > bc.fwd_vel_lim) {
    bs.vel_err = bs.vp_vel_abs - bc.fwd_vel_lim;
  } 
  else if (bs.vp_vel_abs < bc.rev_vel_lim) {
    bs.vel_err = bs.vp_vel_abs - bc.rev_vel_lim;
  }
  else {
    bs.vel_err = 0.0;
  }

  // compute target pitch
  bs.target_pitch = filter2_run(&bc.target_pitch_filter, &bf.target_pitch_filter,
                                dsplim(dspmul(bc.vel_coeff, bs.vel_err),
                                       -bc.target_pitch_lim, bc.target_pitch_lim));
  bs.target_pitch_rate = 0; // filter2_deriv(&bf.target_pitch_filter);
  
  bs.pitch_err = bs.target_pitch - bs.effective_pitch;
  bs.pitch_rate_err = bs.target_pitch_rate + bs.neutral_pitch_rate - bs.pitch_rate;

  if (bs.mode==MODE_RUNNING) {
    bs.pitch_err_i = dsplim(bs.pitch_err_i + dspmul(bs.pitch_err, bs.interval),
                            -bc.pitch_err_i_lim, bc.pitch_err_i_lim);
  } else {
    bs.pitch_err_i=0;
  }

  // compute fwd_cmd, yaw_cmd
  bs.fwd_cmd = filter2_run(&bc.fwd_cmd_filter, &bf.fwd_cmd_filter,
                           - dspmul(bc.pitch_coeff, bs.pitch_err)
                           - dspmul(bc.pitch_rate_coeff, bs.pitch_rate_err)
                           - dspmul(bc.pitch_i_coeff, bs.pitch_err_i));

  bs.yaw_cmd = filter2_run(&bc.yaw_cmd_filter, &bf.yaw_cmd_filter,
                           + dspmul(bc.yaw_rate_coeff, bs.yaw_rate_err)
                           + dspmul(bc.yaw_steering_coeff, steering_rate));

  if (bs.mode==MODE_RUNNING || bs.mode==MODE_STARTUP) {
    if (bs.fwd_cmd > bc.hard_fwd_lim) {
      bs.fwd_cmd=bc.hard_fwd_lim;
      bs.at_limit |= (1<<LIMIT_CMD);
    }
    else if (bs.fwd_cmd < -bc.hard_fwd_lim) {
      bs.fwd_cmd=-bc.hard_fwd_lim;
      bs.at_limit |= (1<<LIMIT_CMD);
    }
    if (bs.yaw_cmd > bc.hard_yaw_lim) {
      bs.yaw_cmd=bc.hard_yaw_lim;
      bs.at_limit |= (1<<LIMIT_CMD);
    }
    else if (bs.yaw_cmd < -bc.hard_yaw_lim) {
      bs.yaw_cmd=-bc.hard_yaw_lim;
      bs.at_limit |= (1<<LIMIT_CMD);
    }
  }    

  // PENDING->STARTUP if pretty level
  if (bs.mode==MODE_PENDING && 
      bs.pitch_err > DSPNUM(-0.2) && bs.pitch_err < DSPNUM(0.2) &&
      bs.fwd_cmd > DSPNUM(-0.8) && bs.fwd_cmd < DSPNUM(0.8)) {
    bs.softstart=0;
    bs.mode=MODE_STARTUP;
  }

  // PENDING->TIPPED after 15 seconds
  if (bs.mode==MODE_PENDING && 
      bs.realtime > DSPNUM(15.0)) {
    bs.mode=MODE_TIPPED;
  }
  
  // softstart
  if (bs.mode==MODE_STARTUP) {
    bs.fwd_cmd = dspmul(bs.fwd_cmd, bs.softstart);
    bs.yaw_cmd = dspmul(bs.yaw_cmd, bs.softstart);
  }

  if (bs.mode==MODE_RUNNING) {
    bs.speed_est += (bs.fwd_cmd - dspmul(bc.current_vel_coeff, bs.rmis + bs.lmis) - bs.speed_est)/128;
  } else {
    bs.speed_est = 0;
  }

  // Compute batfac=1/bs.bat_voltage
  dsp_t batfac = (0x40000000L / (bs.bat_voltage>>8)) << 10;

  bs.l_pwm = dspmul(bs.fwd_cmd - bs.yaw_cmd, batfac);
  bs.r_pwm = dspmul(bs.fwd_cmd + bs.yaw_cmd, batfac);
  
  if (bs.at_limit && (bs.mode!=MODE_TIPPED && bs.mode!=MODE_EXPLORE_MOTORS)) {
    set1_beeper();
  }
  else if (bs.thermal_mode && ((bal_tickno>>5)&3) <= bs.thermal_mode) {
    set1_beeper();
  }
  else {
    set0_beeper();
  }

  enforce_limits();

  // set motors
  if (bs.mode==MODE_STARTUP || bs.mode==MODE_RUNNING) {
    set_motors(bs.l_pwm, bs.r_pwm);
  } 
  else if (bs.mode==MODE_EXPLORE_MOTORS) {
    int tn=(bs.realtime>>24) & 7;
    bs.l_pwm=0;
    bs.r_pwm=0;
    if (tn==0 || tn==1) {  // one long pulse for right
      bs.r_pwm=DSPNUM(0.20);
    }
    else if (tn==4 || tn==6) { // two short pulses for left
      bs.l_pwm=DSPNUM(0.20);
    }
    
    set_motors(bs.l_pwm, bs.r_pwm);

    if (bs.realtime>DSPNUM(32)) {
      bs.mode=MODE_PENDING;
    }
  }
  else {
    bs.l_pwm=0;
    bs.r_pwm=0;
    set_motor_idle();
  }
}

void
timer_init(void)
{
  /*
    Timer 1:

    PWM mode is "PWM, Phase Correct, 9-bit"
    14 MHz / 1 / 512 / 2 gives 14 kHz, just inaudible

    Currently the PWM signals for the two wheels are in phase.
    This unnecessarily increases AC current to the batteries.
    I should make them anti-phase.
    
  */

  TCCR1A = 0 |
    (1<<COM1A1) | (1<<COM1A0) | // clear on match up, set on match down
    (1<<COM1B1) | (0<<COM1B0) | // clear on match up, set on match down
#if OCR1_MAX==1023
    (1<<WGM11) | (1<<WGM10);
#elif OCR1_MAX==511
    (1<<WGM11);
#elif OCR1_MAX==255
    (1<<WGM11);
#else
#error "Unknown OCR1_MAX"
#endif

  TCCR1B = 0 |
    (1<<CS10); // prescaler divide by 1


  /*
    Timer 3:
    
    Frequency is about 14756000/64 = 230562 kHz, so it wraps 3.5 times/second
  */

  TCCR3A = 0;
  TCCR3B = 0 |
    (0<<CS32) | (1<<CS31) | (1<<CS30); // prescale clk/64

}

/*
  Receive a packet over the serial port.
 */
void
handle_rx()
{
  uint8_t *pkt;
  uint8_t pktlen=pkt_rx(&pkt);
  if (!pktlen) return;

  ticks_since_beacon=0; // don't send beacons when we're being driven

  uint8_t cmd=*((uint8_t *)pkt); pkt+=sizeof(uint8_t);
  /* Send current adcs */
  if (cmd=='A') {
    pkt_tx_start();
    pkt_tx_uint8('a');
    for (uint8_t i=0; i<ADCNO_END; i++) {
      pkt_tx_uint16(adc_samples[i]);
    }
    pkt_tx_end();
  }

  /* Update */
  else if (cmd=='U') {
    pkt_tx_start();
    pkt_tx_uint8('u');
    pkt_tx_uint8(bs.mode);
    pkt_tx_dsp_8dot24(bs.realtime);
    pkt_tx_dsp_4dot12(bs.vp_vel);
    pkt_tx_dsp_4dot12(bs.speed_est);

    pkt_tx_dsp_4dot12(bs.pitch);
    pkt_tx_dsp_4dot12(bs.roll);

    pkt_tx_dsp_4dot12(bs.pitch_rate);
    pkt_tx_dsp_4dot12(bs.roll_rate);
    pkt_tx_dsp_4dot12(bs.yaw_rate);

    pkt_tx_dsp_4dot12(bs.pitch_gyro);
    pkt_tx_dsp_4dot12(bs.roll_gyro);
    pkt_tx_dsp_4dot12(bs.yaw_gyro);

    pkt_tx_dsp_4dot12(bs.pitch_temp);
    pkt_tx_dsp_4dot12(bs.roll_temp);
    pkt_tx_dsp_4dot12(bs.yaw_temp);

    pkt_tx_dsp_4dot12(bs.pitch_gyro_temp_corr);
    pkt_tx_dsp_4dot12(bs.roll_gyro_temp_corr);
    pkt_tx_dsp_4dot12(bs.yaw_gyro_temp_corr);
    
    pkt_tx_dsp_4dot12(bs.steering_sensor);


    pkt_tx_dsp_4dot12(bs.fwd_cmd);
    pkt_tx_dsp_4dot12(bs.yaw_cmd);
    pkt_tx_dsp_4dot12(bs.vel_err);
    pkt_tx_dsp_4dot12(bs.yaw_rate_err);
    pkt_tx_dsp_4dot12(bs.pitch_err);
    pkt_tx_dsp_4dot12(bs.pitch_err_i);
    pkt_tx_dsp_4dot12(bs.pitch_rate_err);

    pkt_tx_dsp_4dot12(bs.x_accel);
    pkt_tx_dsp_4dot12(bs.y_accel);
    pkt_tx_dsp_4dot12(bs.z_accel);

    pkt_tx_dsp_4dot12(bs.target_pitch);
    pkt_tx_dsp_4dot12(bs.target_pitch_rate);
    pkt_tx_dsp_4dot12(bs.target_yaw_rate);

    pkt_tx_dsp_4dot12(bs.batis);
    pkt_tx_dsp_4dot12(bs.lmis);
    pkt_tx_dsp_4dot12(bs.rmis);
    pkt_tx_dsp_4dot12(bs.bat_voltage);

    pkt_tx_dsp_4dot12(bs.neutral_pitch);
    pkt_tx_dsp_4dot12(bs.neutral_pitch_rate);
    pkt_tx_dsp_4dot12(bs.softstart);

    pkt_tx_end();
  }


  /* Send current bal_config */
  else if (cmd=='R') {
    pkt_tx_start();
    pkt_tx_uint8('r');
    for (uint8_t i=0; i<sizeof(bc); i++) {
      pkt_tx_uint8(((uint8_t *)&bc)[i]);
    }
    pkt_tx_end();
  }

  /* Accept new bal_config */
  else if (cmd=='W') {
    struct bal_config *newbc = ((struct bal_config *)pkt); pkt+=sizeof(struct bal_config);
    pkt_tx_start();
    pkt_tx_uint8('w');
    bc = *newbc;
    pkt_tx_uint8(1);
    pkt_tx_end();
    filter2_clear(&bf.fwd_cmd_filter);
    filter2_clear(&bf.yaw_cmd_filter);
    filter2_clear(&bf.target_pitch_filter);
    filter2_clear(&bf.target_vel_filter);
    filter2_clear(&bf.target_yaw_rate_filter);
    if (bs.mode != MODE_RUNNING && bs.mode != MODE_PENDING) {
      bs.mode = MODE_PENDING;
      bs.softstart=0;

      pkt_tx_start();
      pkt_tx_str_P(PSTR("!set pending"));
      pkt_tx_end();
    }
  }

  /* Test motors */
  else if (cmd=='M') {
    bs.mode = MODE_EXPLORE_MOTORS;
    bs.realtime=0;
  }

  /* Reboot request, handy for uploading new firmware without hitting the big red button */
  else if (cmd=='Q') {
    // Only 'l33t people can reboot it. 
    // (This is mainly just so line noise on the serial port doesn't hose me)
    if (pkt[0] != '3' || pkt[1] != 'b' || pkt[2] != '0' || pkt[3] != '0' || pkt[4] != 't') return;
      
    set_motor_idle();

    // I could be more careful and turn off all the interrupt sources, but this should work too.
    cli();

    // Enable watchdog timer and go into an infinite loop to get a proper hardware reset
    WDTCR = (0<<WDCE) | (1<<WDE) | (0<<WDP2) | (0<<WDP1) | (0<<WDP0);
    while (1) {
    }
  }

  else {
    // discard cmd
  }
}

#define VERSION "scooter 8.0"
uint32_t beacon_count;

int main( void )
{
  uart_init_tx();
  uart_init_rx();

  sei();

  PKT_ANNOUNCE_VERSION(VERSION);

  io_init();
  adc_init();
  timer_init();

  set1_led2();
  while (onboardadc_wraps<3) {
    read_adcs();
  }
  set0_led2();

  balance_configure();
  balance_init();

  set1_led1();
  set0_led2();

  while (1) {

    handle_rx();

    set0_led1();
    balance();
    set1_led1();

    ticks_since_senderr++;
    ticks_since_beacon++;

    if (ticks_since_beacon > 1000) {
      /* Send periodic beacons to inform balctl that we're here */
      pkt_tx_start();
      pkt_tx_str_P(PSTR("!" VERSION " "));
      pkt_tx_uint32_hex(beacon_count++);
      pkt_tx_end();

      ticks_since_beacon=0;
    }

    if (ticks_since_senderr > 1000) {
      err_tx_start();
      err_tx_comm();
      err_tx_end();

      ticks_since_senderr=0;
    }
  }
}
