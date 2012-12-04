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

// #define FAST_TEMP_TEST 1

#define LM_PWM_SIGN +
#define RM_PWM_SIGN +
#define LM_IS_SIGN +
#define RM_IS_SIGN +
#define LM_ENC_SIGN +
#define RM_ENC_SIGN -

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
  set0_powerhold();

  setout_led1();
  setout_led2();
  setout_beeper();
  setout_powerhold();

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

void ls7266_write(uint8_t which, uint8_t data)
{
  set_ls7266_data(data);
  setout_ls7266_data();

  if (which==0) {
    set0_ls7266_y_x();
    set1_ls7266_cs1();
    set1_ls7266_wr();
  }
  else if (which==1) {
    set1_ls7266_y_x();
    set1_ls7266_cs1();
    set1_ls7266_wr();
  }
  else if (which==2) {
    set0_ls7266_y_x();
    set1_ls7266_cs2();
    set1_ls7266_wr();
  }
  else if (which==3) {
    set1_ls7266_y_x();
    set1_ls7266_cs2();
    set1_ls7266_wr();
  }
  set0_ls7266_wr();
  set0_ls7266_cs1();
  set0_ls7266_cs2();

  setin_ls7266_data();
}

void ls7266_write_ctl(uint8_t which, uint8_t data)
{
  set1_ls7266_ctl_data();
  ls7266_write(which, data);
}

void ls7266_write_data(uint8_t which, uint8_t data)
{
  set0_ls7266_ctl_data();
  ls7266_write(which, data);
}

uint8_t ls7266_read(uint8_t which)
{
  setin_ls7266_data();

  if (which==0) {
    set0_ls7266_y_x();
    set1_ls7266_cs1();
    set1_ls7266_rd();
  }
  else if (which==1) {
    set1_ls7266_y_x();
    set1_ls7266_cs1();
    set1_ls7266_rd();
  }
  else if (which==2) {
    set0_ls7266_y_x();
    set1_ls7266_cs2();
    set1_ls7266_rd();
  }
  else if (which==3) {
    set1_ls7266_y_x();
    set1_ls7266_cs2();
    set1_ls7266_rd();
  }
  asm volatile("nop\nnop\nnop\n");
  uint8_t ret=getpin_ls7266_data();
  set0_ls7266_rd();
  set0_ls7266_cs1();
  set0_ls7266_cs2();
  return ret;
}

uint8_t ls7266_read_ctl(uint8_t which)
{
  set1_ls7266_ctl_data();
  return ls7266_read(which);
}

uint8_t ls7266_read_data(uint8_t which)
{
  set0_ls7266_ctl_data();
  return ls7266_read(which);
}

// ----------------------------------------------------------------------

void ls7266_write_pr(uint8_t which, uint8_t data)
{
  ls7266_write_data(which, data);
}
void ls7266_write_rld(uint8_t which, uint8_t data)
{
  ls7266_write_ctl(which, (data&0x1f)|0x00);
}
void ls7266_write_cmr(uint8_t which, uint8_t data)
{
  ls7266_write_ctl(which, (data&0x1f)|0x20);
}
void ls7266_write_ior(uint8_t which, uint8_t data)
{
  ls7266_write_ctl(which, (data&0x1f)|0x40);
}
void ls7266_write_idr(uint8_t which, uint8_t data)
{
  ls7266_write_ctl(which, (data&0x1f)|0x60);
}
uint8_t ls7266_read_flag(uint8_t which)
{
  return ls7266_read_ctl(which);
}
uint8_t ls7266_read_ol(uint8_t which)
{
  return ls7266_read_data(which);
}

// ----------------------------------------------------------------------


void
ls7266_update_encoder(uint8_t which, int32_t *ctr)
{
  ls7266_write_rld(which, 0x01 | 0x10); // reset bp, transfer counters to OL

  uint8_t oldb2=((uint8_t *)ctr)[2];

  ((uint8_t *)ctr)[0] = ls7266_read_data(which);
  ((uint8_t *)ctr)[1] = ls7266_read_data(which);
  ((uint8_t *)ctr)[2] = ls7266_read_data(which);
  uint8_t newb2=((uint8_t *)ctr)[2];

  uint8_t dir=newb2 - oldb2;
  if ((dir&0x80) && !(oldb2&0x80) && (newb2&0x80)) {
    ((uint8_t *)ctr)[3]--;
  }
  else if (!(dir&0x80) && (oldb2&0x80) && !(newb2&0x80)) {
    ((uint8_t *)ctr)[3]++;
  }
}

void ls7266_zero_counter(uint8_t which)
{
  ls7266_write_rld(which, 0x01); // reset bp
  ls7266_write_pr(which, 0);
  ls7266_write_pr(which, 0);
  ls7266_write_pr(which, 0);
  ls7266_write_rld(which, 0x08); // transfer PR to CNTR
}

void ls7266_configure(uint8_t which)
{
  ls7266_write_rld(which, 0x02); // reset cntr
  ls7266_write_rld(which, 0x04); // reset BT, CT, CPT, S
  ls7266_write_rld(which, 0x06); // reset E

  ls7266_write_rld(which, 0x01); // reset bp
  /*
    I don't know how to choose a prescale value. Presumably if the
    frequency is too low, it'll screw up when the wheel velocity gets
    too high. But higher frequencies must be more susceptible to
    noise. The ls7266s are clocked at 10 MHz, so with a prescale of 2
    we should have a max count rate of 5.0 MHz. 8000 RPM of the motor
    corresponds to 1.1 million counts/sec (or 270 kHz signals) with an
    8192-count wheel. It says you want fFCKn >= 8*fQA, and I think we
    have fFCKn > 18 fQA.

    It would majorly suck if we hit some upper frequency limit, either
    because of the counters or because of some limit in the optical
    encoders. We'd probably detect a large velocity and yaw error and
    wipe out at high speed. The HEDS module has HCMOS-type drivers
    (sink or source 8 mA) so 270 kHz shouldn't be a problem.

    I should probably change to lower-resolution encoders anyway. And
    make sure to use LS series for maximum noise immunity (the HEDS
    outputs TTL logic levels.)

    The LS7266 already has appropriate (TTL) input thresholds for the
    encoders.
  */
  ls7266_write_pr(which, 1);     // prescale by 2.
  ls7266_write_rld(which, 0x18); // transfer PR0 to PSC

  ls7266_write_cmr(which, 0x18);  // quadrature x4, normal, binary

  ls7266_write_ior(which, 0x01);  // enable A&B, LCTNR, RCNTR, /carry & /borrow

  ls7266_write_idr(which, 0x00);  // disable index

  ls7266_zero_counter(which);
}

void ls7266_init()
{
  set0_ls7266_ctl_data();
  set0_ls7266_wr();
  set0_ls7266_cs1();
  set0_ls7266_cs2();
  set0_ls7266_rd();
  set0_ls7266_y_x();

  setout_ls7266_ctl_data();
  setout_ls7266_wr();
  setout_ls7266_cs1();
  setout_ls7266_cs2();
  setout_ls7266_rd();
  setout_ls7266_y_x();
  
  for (uint8_t i=0; i<4; i++) ls7266_configure(i);
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
  {ADCNO_PITCH_RATE,    1, 0},
  {ADCNO_PITCH_VREF,    1, 1},
  {ADCNO_PITCH_TEMP,    1, 2},
  {ADCNO_X_ACCEL,       1, 3},
  {ADCNO_Y_ACCEL,       1, 4},
  {ADCNO_BAT_VOLTAGE,   1, 5},
};

struct onboardadc_config_t {
  uint8_t adcno;
  uint8_t muxno;
};
struct onboardadc_config_t onboardadc_config[] PROGMEM = {
  {ADCNO_BATIS,   (1<<REFS0)|(1<<ADLAR)|0},
  {ADCNO_LMIS,    (1<<REFS0)|(1<<ADLAR)|1},
  {ADCNO_RMIS,    (1<<REFS0)|(1<<ADLAR)|2},
  {ADCNO_TEMPA,   (1<<REFS0)|(1<<ADLAR)|3},
  {ADCNO_TEMPB,   (1<<REFS0)|(1<<ADLAR)|4},
  {ADCNO_TEMPC,   (1<<REFS0)|(1<<ADLAR)|5},
  {ADCNO_TEMPD,   (1<<REFS0)|(1<<ADLAR)|6},
  {ADCNO_SUPVMON, (1<<REFS0)|(1<<ADLAR)|7},
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

    ADMUX = pgm_read_byte(&onboardadc_config[onboardadc_phase].muxno);
    ADCSRA |= (1<<ADSC) | (1<<ADIF); // clear flag, start next conv
  }
}

void
adc_init()
{
  set0_ads8344_cs1();
  set0_ads8344_dclk();
  set0_ads8344_di();
  set0_ads8344_dout();

  setout_ads8344_cs1();
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
  ADMUX = pgm_read_byte(&onboardadc_config[onboardadc_phase].muxno);
  ADCSRA |= (1<<ADSC); // Start next conv
  
}


static uint16_t last_balance_tcnt3;

void balance_configure()
{
  memset(&bc, 0, sizeof(bc));
#if 0
  bc.vp_height=DSPNUM(2.0);

  bc.vel_err_lim=DSPNUM(0.1);
  bc.vel_err_i_lim=DSPNUM(0.1);
  bc.yaw_rate_err_lim=DSPNUM(0.1);
  bc.yaw_rate_err_i_lim=DSPNUM(0.1);
  
  bc.vel_coeff=DSPNUM(0.1);
  bc.vel_i_coeff=DSPNUM(0.1);
  bc.yaw_rate_coeff=DSPNUM(0.1);
  bc.yaw_rate_i_coeff=DSPNUM(0.1);

  bc.pitch_coeff=DSPNUM(0.1);
  bc.pitch_rate_coeff=DSPNUM(0.1);
  bc.pitch_i_coeff=DSPNUM(0.1);

  bc.hard_fwd_lim=DSPNUM(0.6);
  bc.hard_yaw_lim=DSPNUM(0.2);
  bc.deadband = 5;
  bc.hysteresis = 2;
  
  // A plausible 0.1 moment filter (assuming 0.008 moments cycle time)
  // We just set these up so the filter outputs won't start from zero, especially the pitch filter.
  bc.fwd_cmd_filter.bcoeff[0]=DSPNUM(0.012924194);
  bc.fwd_cmd_filter.bcoeff[1]=DSPNUM(0.025848388);
  bc.fwd_cmd_filter.bcoeff[2]=DSPNUM(0.012924194);
  bc.fwd_cmd_filter.acoeff[0]=DSPNUM(0.645568847);
  bc.fwd_cmd_filter.acoeff[1]=DSPNUM(-1.593872070);

  bc.yaw_cmd_filter = bc.fwd_cmd_filter;
  bc.target_pitch_filter = bc.fwd_cmd_filter;
  bc.target_vel_filter = bc.fwd_cmd_filter;
  bc.target_yaw_rate_filter = bc.fwd_cmd_filter;
#endif
}

void balance_init()
{
  last_balance_tcnt3=TCNT3;

  memset(&bi, 0, sizeof(bi));
  bi.lmis_bias = 32768;
  bi.rmis_bias = 32768;
  bi.batis_bias = 32768;

  memset(&bs, 0, sizeof(bs));
  bs.mode = MODE_INITIAL;
  bs.target_age = 83886080L;

  memset(&bf, 0, sizeof(bf));

  set_motor_idle();
  set0_accel_st();
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
      pkt_tx_uint32_hex(bs.avg_vel);
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
    pkt_tx_uint8(' ');
    pkt_tx_uint32_hex(bs.lmtemp);
    pkt_tx_uint8(' ');
    pkt_tx_uint32_hex(bs.rmtemp);
    pkt_tx_end();
  }
}

void update_motor_temps() // called 4x/tick
{
  dsp_t lmisq = dspmul(bs.lmis, bs.lmis);
  dsp_t rmisq = dspmul(bs.rmis, bs.rmis);

  /*
    Estimate heat input = I**2 * R, R=0.15   (R=0.11 at room temp)
    Thermal mass:
    0.8 kg copper @ 337 J/kg/K
    0.5 kg steel @ 444 J/kg/K
    (I weighed the rotor at 1.34 kg)
    Total: Cp=491 J/K
    Thermal resistance: Rt=1.5 K/W    (the Magmotor C30-300 is 1.3 K/W)

    So, max continuous current for 100 K temp increase is 21 amps
    Time constant = 840 seconds

    T' = I**2 * R / Cp - T / Rt / Cp
    where T is in K relative to room temperature

    We use units of 100K for T and 100A for I.

    Temp roundoff is at 0.1 K
    Current roundoff is at 0.6 A
  */
  const float Rm=0.15;      // motor resistance, ohms
  const float Rt=1.5;       // thermal resitance, K/W
#ifdef FAST_TEMP_TEST
  const float Cp=25.0;
#else
  const float Cp=491.0;     // heat capacity, J/K
#endif
  const float Dt=1.0/MOMENTS_PER_SEC/4.0;  // time interval, seconds
  const float Tunits=100.0; // temp units (1 = 100K)
  const float Iunits=100.0; // current units (1 = 100A)
#ifdef FAST_TEMP_TEST
  const float overheat_temp=40.0/Tunits;
#else
  const float overheat_temp=80.0/Tunits;    // insulation class A: 105 C
#endif

  const dsp_t isq_coeff =  DSPNUM(Iunits*Iunits * Rm / Cp / Tunits * Dt);
  const dsp_t temp_coeff = DSPNUM(1.0 / Rt / Cp * Dt);

  ASMCOMMENT("compute mtemp");
  bs.lmtemp = bs.lmtemp + dspmul(lmisq, isq_coeff) - dspmul(bs.lmtemp, temp_coeff);
  bs.rmtemp = bs.rmtemp + dspmul(rmisq, isq_coeff) - dspmul(bs.rmtemp, temp_coeff);
  ASMCOMMENT("done");

  dsp_t maxtemp=dspmax(bs.lmtemp, bs.rmtemp);

  int8_t tmdelta=0;
  if (bs.thermal_mode==0) {
    if (maxtemp>DSPNUM(0.6*overheat_temp)) tmdelta=1;
  }
  else if (bs.thermal_mode==1) {
    if (maxtemp>DSPNUM(0.8*overheat_temp)) tmdelta=1;
    if (maxtemp<DSPNUM(0.5*overheat_temp)) tmdelta=-1;
  }
  else if (bs.thermal_mode==2) {
    if (maxtemp>DSPNUM(1.0*overheat_temp)) tmdelta=1;
    if (maxtemp<DSPNUM(0.7*overheat_temp)) tmdelta=-1;
  }
  else if (bs.thermal_mode==3) {
    if (maxtemp<DSPNUM(0.9*overheat_temp)) tmdelta=-1;
  }

  if (tmdelta) {
    bs.thermal_mode += tmdelta;
    pkt_tx_start();
    pkt_tx_str_P(PSTR("!thermal mode "));
    pkt_tx_uint8('0'+bs.thermal_mode);
    pkt_tx_uint8(' ');
    pkt_tx_uint32_hex(bs.lmis);
    pkt_tx_uint8(' ');
    pkt_tx_uint32_hex(bs.rmis);
    pkt_tx_uint8(' ');
    pkt_tx_uint32_hex(bs.lmtemp);
    pkt_tx_uint8(' ');
    pkt_tx_uint32_hex(bs.rmtemp);
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
    int32_t last_enc1_raw_pos=bi.enc1_raw_pos;
    int32_t last_enc2_raw_pos=bi.enc2_raw_pos;

    cli();
    // interval = time in seconds since we last updated things.
    uint16_t cur_tcnt3 = TCNT3;
    
    // Since we're taking derivatives, do this as close as possible to the time sample
    ls7266_update_encoder(0, &bi.enc1_raw_pos);
    ls7266_update_encoder(1, &bi.enc2_raw_pos);
    ls7266_update_encoder(2, &bi.enc3_raw_pos);
    sei();

    uint16_t ticks = cur_tcnt3 - last_balance_tcnt3;
    last_balance_tcnt3 = cur_tcnt3;

    // TCNT3 runs at 14756000/64 = 230562 Hz. We want 5*DSP_ONE = 83886080, so multiply by 364.
    // Limit to 0.0005 to 0.005 seconds. Typical is 1 mS 
    bs.interval = dsplim((dsp_t)ticks * DSPNUM(MOMENTS_PER_SEC / (CLOCK_SPEED/64.0)),
                         DSPNUM(0.0025), DSPNUM(0.025));

    bs.inverse_interval = DSP_ONE/bs.interval;
    
    /*
      encoder to pos conversion:
      8192 counts/rev
      20:1 gear ratio
      20" diameter wheel

      This number isn't in 8.24 format. Note the prescale by 4 when doing the multiply.
      Be careful about integer overflow when changing this
    */
    ASMCOMMENT("compute velfac");
    dsp_t wheel_vel_fac = ((dsp_t)((20.0*0.0254*M_PI) / (8192.0*20.0) * DSP_ONE/4*DSP_ONE) / bs.interval)*4;
    
    // These should be in meters/moment
    bs.r_vel = RM_ENC_SIGN (bi.enc1_raw_pos - last_enc1_raw_pos) * wheel_vel_fac;
    bs.l_vel = LM_ENC_SIGN (bi.enc2_raw_pos - last_enc2_raw_pos) * wheel_vel_fac;
  }

  ASMCOMMENT("update realtime");
  {
    dsp_t new_realtime = bs.realtime + bs.interval;
    bs.tickmask = (new_realtime>>16) ^ (bs.realtime>>16);
    bs.realtime = new_realtime;
  }

  dsp_t old_ts_pos = bs.ts_pos;
  // Factor reflects 106 degrees of travel (106/360 * 8192==2412). It's important that this match ts_pos
  // for the calibration routine to work right
  bs.ts_pos = bi.enc3_raw_pos * DSPNUM(2.0/2412.0) + DSP_ONE;
  bs.ts_vel = (bs.ts_pos - old_ts_pos) * bs.inverse_interval;
  if (1) {
    dsp_t pos2=dspmul(bs.ts_pos, bs.ts_pos);
    bs.neutral_pitch = (bc.neutral_pitch_c0 +
                        dspmul(bc.neutral_pitch_c1, bs.ts_pos) +
                        dspmul(bc.neutral_pitch_c2, pos2));
    
    bs.neutral_pitch_rate = dspmul(bc.neutral_pitch_c1 + dspmul(bc.neutral_pitch_c2, bs.ts_pos)/2,
                                   bs.ts_vel);
  }
  
  ASMCOMMENT("compute yaw & vel");
  // Wheel spacing is 21.5 inches
  dsp_t wheel_yaw_fac = DSPNUM(1.0/(21.5*0.0254));
  bs.wheel_yaw_rate = dspmul(bs.r_vel - bs.l_vel, wheel_yaw_fac);
  bs.avg_vel = (bs.l_vel + bs.r_vel) / 2;

  /* Ramp up softstart over the first second */
  if (bs.mode==MODE_STARTUP) {
    bs.softstart = bs.softstart + bs.interval/4;
    if (bs.softstart>=DSPNUM(1.0)) {
      bs.softstart=DSPNUM(1.0);
      bs.mode=MODE_RUNNING;
      set1_powerhold();
    }
  }

  /*
    Stop if we're not getting current motion commands
  */
  ASMCOMMENT("update target_age");
  bs.target_age += bs.interval;
  if (bs.target_age > DSPNUM(50.0)) bs.target_age = DSPNUM(50.0);

  if (bs.target_age >= DSPNUM(1.0)) {
    bs.target_vel=0;
    bs.target_yaw_rate=0;
    // Don't clear neutral_pitch
    bs.neutral_pitch_rate=0;
  }

  ASMCOMMENT("convert bat_voltage");
  // convert 24v => 1
  bs.bat_voltage =  (dsp_t)adc_samples[ADCNO_BAT_VOLTAGE] * DSPNUM(4.34/65536.0);

  ASMCOMMENT("pitch gyro");
  // ADXRS300 rate gyro, with 100K resistor from SUMJ to RATEOUT
  // convert 1.7857 mv/deg/sec to radians/moment.
  bs.pitch_gyro = ((dsp_t)adc_samples[ADCNO_PITCH_RATE] - (dsp_t)adc_samples[ADCNO_PITCH_VREF])
    * DSPNUM((M_PI/180.0) / 1.7857e-3 / MOMENTS_PER_SEC * (5.0/65536.0));

  // convert to temp to 0=27 C, 1=127 C, -1=-73C. sensor gives 8.4 mv/degree
  bs.pitch_temp = ((dsp_t)adc_samples[ADCNO_PITCH_TEMP] - (dsp_t)adc_samples[ADCNO_PITCH_VREF]) * DSPNUM(1.0 / (8.4e-3 * 100) * (5.0/65536.0));

  ASMCOMMENT("current sensors");
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

  if (bs.tickmask & 0xffc0) { // 4x / moment
    update_motor_temps();
  }
  
  // ADXL322 accelerometer
  // Convert from 700 mv/G into Gs
  ASMCOMMENT("convert fwd_accel");
  bs.fwd_accel = (32768L - (dsp_t)adc_samples[ADCNO_Y_ACCEL]) * DSPNUM(5.0 / 0.7 / 65536.0);
  ASMCOMMENT("convert up_accel");
  bs.up_accel = ((dsp_t)adc_samples[ADCNO_X_ACCEL] - 32768L) * DSPNUM(5.0 / 0.7 / 65536.0);

  ASMCOMMENT("Mode stuff");
  if (bs.mode==MODE_INITIAL) return;

  if (bs.mode==MODE_TEST_MOTORS) {
    dsp_t pwm=DSPNUM(0.15);
    set_motors(pwm, pwm);
    return;
  }

  {
    dsp_t x=bs.pitch_temp;
    dsp_t y=bc.pitch_temp_corr_c0;
    
    y += dspmul(x, bc.pitch_temp_corr_c1);
    x = dspmul(x, bs.pitch_temp);
    y += dspmul(x, bc.pitch_temp_corr_c2);
    x = dspmul(x, bs.pitch_temp);
    y += dspmul(x, bc.pitch_temp_corr_c3);
    
    bs.pitch_gyro_temp_corr = y;
  }
  
  ASMCOMMENT("gyro fusion");
  if (!bs.pitch_inited) {
    bs.pitch = bs.fwd_accel;
    bs.pitch_inited=1;
  }
  dsp_t accel_rate = (bs.fwd_accel - bs.pitch)/32;
  bs.pitch_rate = bs.pitch_gyro - bs.pitch_gyro_temp_corr + accel_rate;
  bs.pitch += dspmul(bs.pitch_rate, bs.interval);


  if ((bal_tickno&7)==0 &&
      bi.pitch_mon.s1<32000 &&
      bi.pitch_mon.sy<DSPNUM(120) && bi.pitch_mon.sy>DSPNUM(-120) && 
      bi.pitch_mon.syy<DSPNUM(120) && bi.pitch_mon.syy>DSPNUM(-120) && 
      bi.temp_mon.s1<32000 &&
      bi.temp_mon.sy<DSPNUM(120) && bi.temp_mon.sy>DSPNUM(-120) && 
      bi.temp_mon.syy<DSPNUM(120) && bi.temp_mon.syy>DSPNUM(-120)) {

    bi.pitch_mon.sy += bs.pitch_gyro;
    bi.pitch_mon.syy += dspmul(bs.pitch_gyro, bs.pitch_gyro);
    bi.pitch_mon.s1 ++;
    
    bi.temp_mon.sy += bs.pitch_temp;
    bi.temp_mon.syy += dspmul(bs.pitch_temp, bs.pitch_temp);
    bi.temp_mon.s1 ++;
  }

  ASMCOMMENT("filters");
  bs.lpf_target_vel = filter2_run(&bc.target_vel_filter, &bf.target_vel_filter, bs.target_vel);
  bs.lpf_target_yaw_rate = filter2_run(&bc.target_yaw_rate_filter, &bf.target_yaw_rate_filter, bs.target_yaw_rate);

  // Don't update vel & yaw integral terms when we're being controlled.
  if (0 && (bs.lpf_target_vel > DSPNUM(0.01) || bs.lpf_target_vel < -DSPNUM(0.01))) {
    bs.kill_vel_integral=1000;
  }


  /*
    XXX This is busted because total_pos drifts with the rate gyro. What we should do is to integrate the wheel
    speed to get a wheel pos, and then compute total_pos by adding pitch*vp_height
    XXX this calculation should go after vp_vel and vp_pos.
    Actually, vp_pos is what we want.
   */
  ASMCOMMENT("update total_pos, total_yaw");
  {
    dsp_t d_pos = dspmul(bs.avg_vel, bs.interval);
    dsp_t d_yaw = dspmul(bs.wheel_yaw_rate, bs.interval);
    
    bs.total_pos += d_pos;
    bs.total_yaw += d_yaw;
    if (bs.kill_vel_integral) {
      bs.target_hold_pos += dspmul(bs.vp_vel, bs.interval);
      bs.kill_vel_integral--;
    } else {
      bs.target_hold_pos += dspmul(bs.lpf_target_vel, bs.interval);
    }

    bs.target_hold_yaw += dspmul(bs.target_yaw_rate, bs.interval);
  }
  
  /* If we've tipped past 0.6 radians (45 degrees) abandon hope.
     This happens, for example, when the wheels slip on wet grass or snow and it falls over.
     It's better to give up than keep spinning the wheels trying to right ourselves.
     We probably can't recover anyway.
  */
  ASMCOMMENT("check for tip & bat voltage");
  bs.effective_pitch = bs.pitch - bs.neutral_pitch;
  if (bs.mode != MODE_TIPPED && 
      bs.mode != MODE_PENDING &&
      bs.mode != MODE_EXPLORE_MOTORS && 
      (bs.effective_pitch > DSPNUM(0.6) || bs.effective_pitch < -DSPNUM(0.6))) {
    bs.at_limit |= (1<<LIMIT_PITCH);
  }

  bs.vp_pos = bs.total_pos + dspmul(bs.effective_pitch, bc.vp_height) + dspmul(bs.pitch, DSPNUM(0.254));
  
  ASMCOMMENT("compute vp_vel");
  bs.vp_vel = bs.avg_vel + dspmul(bs.pitch_rate-bs.neutral_pitch_rate, bc.vp_height) + dspmul(bs.pitch_rate, DSPNUM(0.254));
  
  ASMCOMMENT("compute vel_err");
  bs.vel_err = dsplim(bs.lpf_target_vel - bs.vp_vel,
                      -bc.vel_err_lim, bc.vel_err_lim);
  ASMCOMMENT("compute yaw_rate_err");
  bs.yaw_rate_err = dsplim(bs.lpf_target_yaw_rate - bs.wheel_yaw_rate, 
                           -bc.yaw_rate_err_lim, bc.yaw_rate_err_lim);

  if (bs.thermal_mode>=2) {
    dsp_t tmis=bs.rmis+bs.lmis;
    bs.target_hold_pos -= dspmul(dspmul(tmis, DSPNUM(0.2)), bs.interval);   // move back 0.2 m/s for every 100A current

    bs.target_hold_yaw = bs.total_yaw;
  }

  /* Only do the integral term once we're stable.
     Also, disable the integral terms if we get hot, hoping we'll drift to a place that needs less torque
  */
  ASMCOMMENT("integral terms");
  if (bs.mode==MODE_RUNNING) {

    bs.vel_err_i = bs.target_hold_pos - bs.vp_pos;
    if (bs.vel_err_i>=0) {
      bs.vel_err_i -= bc.vel_err_i_deadband;
      if (bs.vel_err_i<0) {
        bs.vel_err_i=0;
      }
      else if (bs.vel_err_i > bc.vel_err_i_lim) {
        bs.vel_err_i = bc.vel_err_i_lim;
        bs.target_hold_pos = bs.vp_pos + bs.vel_err_i;
      }
    }
    else {
      bs.vel_err_i += bc.vel_err_i_deadband;
      if (bs.vel_err_i>0) {
        bs.vel_err_i=0;
      }
      else if (bs.vel_err_i < -bc.vel_err_i_lim) {
        bs.vel_err_i = -bc.vel_err_i_lim;
        bs.target_hold_pos = bs.vp_pos + bs.vel_err_i;
      }
    }
    
    bs.yaw_rate_err_i = bs.target_hold_yaw - bs.total_yaw;
    if (bs.yaw_rate_err_i < -bc.yaw_rate_err_i_lim) {
      bs.yaw_rate_err_i = -bc.yaw_rate_err_i_lim;
      bs.target_hold_yaw = bs.total_yaw + bs.yaw_rate_err_i;
    }
    else if (bs.yaw_rate_err_i > bc.yaw_rate_err_i_lim) {
      bs.yaw_rate_err_i = bc.yaw_rate_err_i_lim;
      bs.target_hold_yaw = bs.total_yaw + bs.yaw_rate_err_i;
    }

  } else {
    bs.target_hold_pos = bs.vp_pos;
    bs.target_hold_yaw = bs.total_yaw;
    bs.vel_err_i = 0;
    bs.yaw_rate_err_i = 0;
  }

  /*
    Below this we can't count on enough voltage to keep ourselves or the PWM drives running.
  */
  if (bs.bat_voltage < DSPNUM(0.5)) {
    bs.at_limit |= (1<<LIMIT_BAT_VOLTAGE);
  }

  ASMCOMMENT("compute target pitch");
  bs.target_pitch = filter2_run(&bc.target_pitch_filter, &bf.target_pitch_filter,
                                dsplim(dspmul(bc.vel_coeff, bs.vel_err) + dspmul(bc.vel_i_coeff, bs.vel_err_i),
                                       -bc.target_pitch_lim, bc.target_pitch_lim));
  bs.target_pitch_rate = filter2_deriv(&bf.target_pitch_filter);
  
  bs.pitch_err = bs.target_pitch - bs.effective_pitch;
  bs.pitch_rate_err = bs.target_pitch_rate + bs.neutral_pitch_rate - bs.pitch_rate;

  if (0 &&  // Disable wheelie protection
      bs.mode==MODE_RUNNING &&
      bs.target_pitch < DSPNUM(-0.1) && 
      bs.vel_err < DSPNUM(-0.2) &&
      bs.pitch < DSPNUM(-0.4) && 
      bs.avg_vel > DSPNUM(0.2) // 2.25 mph
      ) {
    bs.at_limit |= (1<<LIMIT_WHEELIE);
  }


  if (bs.mode==MODE_RUNNING) {
    bs.pitch_err_i = dsplim(bs.pitch_err_i + dspmul(bs.pitch_err, bs.interval),
                            -bc.pitch_err_i_lim, bc.pitch_err_i_lim);
  } else {
    bs.pitch_err_i=0;
  }
  

  ASMCOMMENT("compute fwd_cmd, yaw_cmd");
  bs.fwd_cmd = filter2_run(&bc.fwd_cmd_filter, &bf.fwd_cmd_filter,
                           - dspmul(bc.pitch_coeff, bs.pitch_err)
                           - dspmul(bc.pitch_i_coeff, bs.pitch_err_i)
                           - dspmul(bc.pitch_rate_coeff, bs.pitch_rate_err));

  bs.yaw_cmd = filter2_run(&bc.yaw_cmd_filter, &bf.yaw_cmd_filter,
                           + dspmul(bc.yaw_rate_coeff, bs.yaw_rate_err) 
                           + dspmul(bc.yaw_rate_i_coeff, bs.yaw_rate_err_i));

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

  ASMCOMMENT("PENDING->STARTUP");
  if (bs.mode==MODE_PENDING && 
      bs.pitch_err > DSPNUM(-0.3) && bs.pitch_err < DSPNUM(0.3) &&
      bs.fwd_cmd > DSPNUM(-0.1) && bs.fwd_cmd < DSPNUM(0.1)) {
    bs.softstart=0;
    bs.mode=MODE_STARTUP;
  }
  
  ASMCOMMENT("softstart");
  if (bs.mode==MODE_STARTUP) {
    bs.fwd_cmd = dspmul(bs.fwd_cmd, bs.softstart);
    bs.yaw_cmd = dspmul(bs.yaw_cmd, bs.softstart);
  }

  // Compute batfac=1/bs.bat_voltage
  ASMCOMMENT("compute batfac, pwms");

  dsp_t batfac;
  {
    dsp_t bvdiff = bs.bat_voltage-bi.batfac_cache.input;
    if (0 && bvdiff<DSPNUM(0.05) && bvdiff>-DSPNUM(0.05)) {
      batfac = bi.batfac_cache.output - bvdiff;
    } else {
      batfac = (0x40000000L / (bs.bat_voltage>>8)) << 10;
      bi.batfac_cache.input = bs.bat_voltage;
      bi.batfac_cache.output = batfac;
      bi.batfac_cache.misses++;
    }
  }

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

  ASMCOMMENT("set motors");
  if (bs.mode==MODE_STARTUP || bs.mode==MODE_RUNNING) {
#if 0 // testing
    set_motor_idle();
#else
    set_motors(bs.l_pwm, bs.r_pwm);
#endif
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
  /* Send current bal_state */
  if (cmd=='C') {
    pkt_tx_start();
    pkt_tx_uint8('c');
    pkt_tx_uint16(bal_tickno);
    for (uint8_t i=0; i<sizeof(bs); i++) {
      pkt_tx_uint8(((uint8_t *)&bs)[i]);
    }
    pkt_tx_end();
  }

  /* Update */
  else if (cmd=='U') {
    pkt_tx_start();
    pkt_tx_uint8('u');
    pkt_tx_uint8(bs.mode);
    pkt_tx_dsp_8dot8(bs.total_pos);
    pkt_tx_dsp_8dot8(bs.total_yaw);
    pkt_tx_dsp_4dot12(bs.vp_vel);
    pkt_tx_dsp_4dot12(bs.avg_vel);
    pkt_tx_dsp_4dot12(bs.wheel_yaw_rate);
    pkt_tx_dsp_4dot12(bs.pitch);
    pkt_tx_dsp_4dot12(bs.pitch_rate);
    pkt_tx_dsp_4dot12(bs.fwd_cmd);
    pkt_tx_dsp_4dot12(bs.yaw_cmd);
    pkt_tx_dsp_4dot12(bs.vel_err);
    pkt_tx_dsp_4dot12(bs.vel_err_i);
    pkt_tx_dsp_4dot12(bs.yaw_rate_err);
    pkt_tx_dsp_4dot12(bs.yaw_rate_err_i);
    pkt_tx_dsp_4dot12(bs.pitch_err);
    pkt_tx_dsp_4dot12(bs.pitch_err_i);
    pkt_tx_dsp_4dot12(bs.fwd_accel);
    pkt_tx_dsp_4dot12(bs.target_pitch);
    pkt_tx_dsp_4dot12(bs.target_pitch_rate);
    pkt_tx_dsp_4dot12(bs.batis);
    pkt_tx_dsp_4dot12(bs.lmis);
    pkt_tx_dsp_4dot12(bs.rmis);
    pkt_tx_dsp_4dot12(bs.neutral_pitch);
    pkt_tx_dsp_4dot12(bs.neutral_pitch_rate);
    pkt_tx_end();
  }


  /* Send current adcs */
  else if (cmd=='A') {
    pkt_tx_start();
    pkt_tx_uint8('a');
    pkt_tx_uint8(ADCNO_END);
    for (uint8_t i=0; i<ADCNO_END; i++) {
      pkt_tx_uint16(adc_samples[i]);
    }
    pkt_tx_end();

  }

  // zero encoders
  else if (cmd=='Z') {
    ls7266_zero_counter(2);
  }

  else if (cmd=='P') {
    uint8_t persist=*((uint8_t *)pkt); pkt+=sizeof(uint8_t);
    if (persist) {
      set1_powerhold();
    } else {
      set0_powerhold();
    }
  }

  else if (cmd=='K') {
    set0_powerhold();
    bs.mode=MODE_TIPPED;
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

  else if (cmd=='G') {
    pkt_tx_start();
    pkt_tx_uint8('g');

    pkt_tx_dsp_8dot24(bi.pitch_mon.sy);
    pkt_tx_dsp_8dot24(bi.pitch_mon.syy);
    pkt_tx_uint16(bi.pitch_mon.s1);

    pkt_tx_dsp_8dot24(bi.temp_mon.sy);
    pkt_tx_dsp_8dot24(bi.temp_mon.syy);
    pkt_tx_uint16(bi.temp_mon.s1);

    pkt_tx_end();

    memset(&bi.pitch_mon, 0, sizeof(bi.pitch_mon));
    memset(&bi.temp_mon, 0, sizeof(bi.temp_mon));
  }

  /* Accept drive command */
  else if (cmd=='D') {
    bs.target_vel = *((dsp_t *)pkt); pkt+=sizeof(dsp_t);
    bs.target_yaw_rate = *((dsp_t *)pkt); pkt+=sizeof(dsp_t);
    bs.external_pitch_adj = *((dsp_t *)pkt); pkt+=sizeof(dsp_t);
    bs.target_age = 0;
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

#define VERSION "segbase 2.0"
uint32_t beacon_count;

int main( void )
{
  io_init();
  ls7266_init();
  adc_init();
  timer_init();
  uart_init_tx();
  uart_init_rx();

  sei();

  PKT_ANNOUNCE_VERSION(VERSION);

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
