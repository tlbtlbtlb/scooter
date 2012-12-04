/*
 *
 * Firmware for self-balancing vehicles. It handles 2-wheeled
 * balancing robots, 2-wheeled scooters, and unicycles. See README.
 *
 * COPYRIGHT
 *  
 */

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/delay.h>
#include <avr/eeprom.h>
#include <math.h>

#include "iobits.h"
#include "uart.h"
#include "string.h"
#include "adc.h"
#include "balconf.h"
#include "hwdefs.h"

enum {
  MODE_INITIAL,
  MODE_RUNNING,
  MODE_TIPPED,
  MODE_HWADJ,
  MODE_HWADJ_DONE,
  MODE_TEST_MOTORS,
};


void barf();

void
io_init()
{
  /*
    All the IO pins which control the PWM have pull up/down resistors to make their high-Z
    state safe. First we set the correct (neutral) levels, then we turn them into
    outputs.
   */
#ifdef DO_STEERING
  set0_r_pwm_oc();
  set1_r_pwm_disable();
  set0_r_pwm_rev();
  set0_r_pwm_fwd();
  setout_r_pwm_oc();
  setout_r_pwm_disable();
  setout_r_pwm_rev();
  setout_r_pwm_fwd();
#endif

  set0_l_pwm_oc();
  set1_l_pwm_disable();
  set0_l_pwm_rev();
  set0_l_pwm_fwd();
  setout_l_pwm_oc();
  setout_l_pwm_disable();
  setout_l_pwm_rev();
  setout_l_pwm_fwd();

  set0_accel_st();
  setout_accel_st();

  set0_led1();
  set0_led2();
  setout_led1();
  setout_led2();
#ifdef __BAL4__
  set0_led3();
  setout_led3();
#endif

  set1_beeper();
  setout_beeper();

  set_extramux(0);
  setout_extramux();
  
#if defined(DO_ENCODERS)
  // Encoders, the HCTL-2016
  set1_enc_oe1_not();
  set1_enc_oe2_not();
  set1_enc_rst_not();
  set0_enc_sel1();
  set0_enc_sel2();
  setout_enc_oe1_not();
  setout_enc_oe2_not();
  setout_enc_rst_not();
  setout_enc_sel1();
  setout_enc_sel2();
  setin_enc_data();
#endif

}


/* Convert from timer 3 counts to seconds */
float timer3_seconds_conv;

/* Do one iteration of a low-pass filter. *state will track input, but only slowly
   tc is the time constant and interval is the length of 1 time step 
*/

void lpf_update(float *state, float tc, float interval, float input)
{
  float frac=interval/tc;
  if (frac>1.0) frac=1.0;
  *state = input*frac + *state * (1.0-frac);
}

/* Fabs seems to be declared in a header, but not defined in the library?
   Oh, well, write my own 
*/

#define fabs my_fabs
float fabs(float a)
{
  if (a>=0.0) {
    return a;
  } else {
    return -a;
  }
}

float fmax(float a, float b)
{
  if (a>=b) {
    return a;
  } else {
    return b;
  }
}

float fmin(float a, float b)
{
  if (a<=b) {
    return a;
  } else {
    return b;
  }
}

float flim(float x, float lo, float hi)
{
  if (x>hi) return hi;
  if (x<lo) return lo;
  return x;
}


/* A basic 2-input Kalman filter for a rate gyro stabilized with a
   2-axis accelerometer.

   In a nutshell...
   
   We have two inputs, a measured angular rotation rate (degrees per second) and a
   forward-backward acceleration. We integrate the angular rate to get the angle. Except
   that the angular rate has a bias on it, simulating an apparent rotation all the time at
   a small rate that varies with temperature. So we have to correct it with the
   accelerometer. The accelerometer is a tiny mass on a spring that moves forward-backward
   in the frame of reference of the scooter. At rest, it will read positive if the scooter
   is tipped forward and negative if tipped backwards, since gravity pulls it forwards or
   backwards. However, it also moves a lot due to the scooter itself acclerating. So we
   use the accelerometer as a long-term correction and the rate gyro for short-term
   changes.

   This isn't really a Kalman filter, since that particular brand of filter dynamically
   estimates the noise on each signal. I initially did that but it didn't work well, since
   the noise is not independent, additive white Gaussian noise. Instead, how much we trust
   the acclerometer depends on how much we're accelerating.

   This isn't perfect; the scooter can wobble slowly by a couple degrees over several
   seconds, and especially after a sharp maneuver. But it's barely noticeable riding it;
   you get used to adjusting for it. 
*/
void gyro_sample_rate(struct gyro_filter *it, float in_rate, float interval)
{
  if (!it->rate_inited) {
    it->rate_bias = in_rate;
    it->rate_inited=1;
  }
  
  it->rate = in_rate - it->rate_bias;
  it->angle += it->rate * interval;

#ifdef __UNICYCLE2__
  it->rate_bias += it->rate * 0.1 * interval;
#else
  it->rate_bias += it->rate * 0.3 * interval;
#endif
}

void gyro_sample_angle(struct gyro_filter *it, float in_angle, float interval)
{
  float angle_err;

  if (!it->angle_inited) {
    it->angle = in_angle;
    it->angle_inited=1;
  }
  
  angle_err = in_angle - it->angle;

#ifdef __UNICYCLE2__
  it->angle += angle_err * 3.0 * interval / (2.0 + it->angle_noise);
#else
  it->angle += angle_err * interval / (2.0 + it->angle_noise);
#endif
}

/* 
   Set the PWM values for the motors. +1.0 is full forward, -1.0 is full reverse.
*/

void set_motors(float left
#ifdef DO_STEERING
                , float right
#endif
                )
{
  uint8_t debuglog=0;
  {
    static uint8_t dlc;
    dlc++;
    if (dlc==0) debuglog=1;
  }
  left=flim(L_MOTOR_POLARITY left, -1.0, 1.0);
  int16_t lefti = (int16_t)(left*(OCR1_MAX+0.9));
  if (lefti<-(OCR1_MAX-3)) lefti=-(OCR1_MAX-3);
  if (lefti>OCR1_MAX-3) lefti=OCR1_MAX-3;

#ifdef DO_STEERING
  right=flim(R_MOTOR_POLARITY right, -1.0, 1.0);
  int16_t righti = (int16_t)(right*(OCR1_MAX+0.9));
  if (righti<-(OCR1_MAX-3)) righti=-(OCR1_MAX-3);
  if (righti>OCR1_MAX-3) righti=OCR1_MAX-3;
#endif

  /* 
    It's OK to update these asynchronously with the PWM clock, because OCRn[AB] is double buffered 
    and only changes when the counter reaches TOP 
  */

  cli();
  // The OSMC1 port on the BAL2 board
  if (lefti<0) {
    ASMCOMMENT("left reverse");
    /* For reverse, set PWM_FWD=0, PWM_DISABLE=0, PWM_REV=1.
       The order here is carefully chosen to avoid glitches.
       Note also that we have interrupts disabled so there's no chance of dead time.
    */
    set0_l_pwm_fwd();
    set0_l_pwm_disable();
    set1_l_pwm_rev();
    set_l_pwm(-lefti);
    if (debuglog) {
      pkt_tx_start();
      pkt_tx_str_P(PSTR("!lr"));
      pkt_tx_uint16_hex(-lefti);
      pkt_tx_end();
    }
  } else {
    ASMCOMMENT("left forward");
    // For forward, set PWM_FWD=1, PWM_DISABLE=0, PWM_REV=0
    set0_l_pwm_rev();
    set0_l_pwm_disable();
    set1_l_pwm_fwd();
    set_l_pwm(lefti);
    if (debuglog) {
      pkt_tx_start();
      pkt_tx_str_P(PSTR("!lf"));
      pkt_tx_uint16_hex(lefti);
      pkt_tx_end();
    }
  }

#ifdef DO_STEERING
  // The OSMC2 port on the BAL2 board
  if (righti<0) {
    ASMCOMMENT("right reverse");
    set0_r_pwm_fwd();
    set0_r_pwm_disable();
    set1_r_pwm_rev();
    set_r_pwm(OCR1_MAX+righti);
    if (debuglog) {
      pkt_tx_start();
      pkt_tx_str_P(PSTR("!rr"));
      pkt_tx_uint16_hex(OCR1_MAX+righti);
      pkt_tx_end();
    }
  } else {
    set0_r_pwm_rev();
    set0_r_pwm_disable();
    set1_r_pwm_fwd();
    set_r_pwm(OCR1_MAX-righti);
    if (debuglog) {
      pkt_tx_start();
      pkt_tx_str_P(PSTR("!rf"));
      pkt_tx_uint16_hex(OCR1_MAX-righti);
      pkt_tx_end();
    }
  }
#endif

  sei();
}

/*
  Different from setting the motors to zero speed, this disables both legs of the H bridge
  so the motor can freewheel. This is a better thing to do in an emergency than locking
  the wheel by calling set_motors(0.0, 0.0). 
*/

void set_motor_idle()
{

#ifdef DO_STEERING
  set_r_pwm(0);
  set1_r_pwm_disable();
  set0_r_pwm_rev();
  set0_r_pwm_fwd();
#endif

  set_l_pwm(0);
  set1_l_pwm_disable();
  set0_l_pwm_rev();
  set0_l_pwm_fwd();
}

void
accum_88_32(uint8_t l, uint8_t h, uint32_t *out)
{
  uint8_t *outb = (uint8_t *)out;
  
  outb[0] = l;
  uint8_t oldh = outb[1];
  outb[1] = h;

  uint8_t dir=h-oldh;

  if ((dir&0x80) && !(oldh&0x80) && (h&0x80)) {
    outb[3]--;
    if (outb[3]==0xff) outb[4]--;
  }
  else if (!(dir&0x80) && (oldh&0x80) && !(h&0x80)) {
    outb[3]++;
    if (outb[3]==0) outb[4]++;
  }

}

#if defined(DO_ENCODERS)
void
read_encoders(uint32_t *enc1_r, uint32_t *enc2_r)
{
  set0_enc_sel1();
  set0_enc_oe1_not();
  _delay_loop_1(10);
  uint8_t enc1h = getpin_enc_data();
  set1_enc_oe1_not();
  set1_enc_sel1();
  set0_enc_oe1_not();
  _delay_loop_1(10);
  uint8_t enc1l = getpin_enc_data();
  set1_enc_oe1_not();

  accum_88_32(enc1l, enc1h, enc1_r);

  set0_enc_sel2();
  set0_enc_oe2_not();
  _delay_loop_1(10);
  uint8_t enc2h = getpin_enc_data();
  set1_enc_oe2_not();
  set1_enc_sel2();
  set0_enc_oe2_not();
  _delay_loop_1(10);
  uint8_t enc2l = getpin_enc_data();
  set1_enc_oe2_not();

  accum_88_32(enc2l, enc2h, enc2_r);
}
#endif

/* 
   All the state and config variables are defined in balconf.h, so I can get at them externally.
*/
struct bal_config bc;
struct bal_state bs;
struct bal_hwadj bh;

/*
  Constants to convert integer ADC samples to numbers between 0 and 1, or -1 and 1.
  What the ADC system actually gives us is the sum of ADC_HIST (8) consecutive samples.
*/
static float sample_conv_bipolar = 1.0/512.0/(float)ADC_HIST;
static float sample_conv_unipolar = 1.0/1024.0/(float)ADC_HIST;

void
update_hwadj(float lpf)
{
#ifdef DO_STEERING
  bh.steering_bias -= lpf*bs.steering;
  bh.yaw_bias -= lpf*bs.yaw_rate;
#endif
  bh.fwd_accel_bias -= lpf*bs.fwd_accel;
#ifdef __BAL3__
  bh.right_accel_bias -= lpf*bs.right_accel;
#endif

  bh.bat_voltage_mult += lpf*(1.0/bs.bat_voltage - 1.0);
}

int bal_hwadj_validate(struct bal_hwadj *h)
{
#define R(X,L,H) if (h->X < L || h->X > H) return 0;
#ifdef DO_STEERING
  R(steering_bias, -2.0, 2.0);
  R(yaw_bias, -2.0, 2.0);
#endif
  R(fwd_accel_bias, -5.0, 5.0);
  R(right_accel_bias, -5.0, 5.0);
  R(bat_voltage_mult, 1.0, 5.0);
#undef R
  return 1;
}

int bal_config_validate(struct bal_config *c)
{
#define R(X,L,H) if (c->X < L || c->X > H) return 0;

  R(p_gain, 0.0, 100.0);
  R(d_gain, 0.0, 30.0);
  R(i_gain, 0.0, 100.0);
  R(hard_speed_lim, 0.01, 5.0);
  R(fwd_speed_lim, 0.0, 2.0);
  R(rev_speed_lim, 0.0, 2.0);
#undef R
  return 1;
}

int bal_drive_validate(struct bal_drive *c)
{
#define R(X,L,H) if (c->X < L || c->X > H) return 0;

  R(speed_targ, -0.5, 0.5);
  R(steering_targ, -1.0, 1.0);

#undef R
  return 1;
}


/*
  Reasonable parameter values. Tuners, this is what you're looking for.
  
  Actually, you should use the serial port interface and GUI application to adjust these
  on-line, then copy the values back here.

  Read the comments in balconf.h for intent, then look at the code to see what they
  actually do.
*/
void balance_configure(void)
{
#if defined(__SEGWELL2__)
  bc.p_gain = 6.0;
  bc.d_gain = 0.60;
  bc.i_gain = 2.40;
  bc.motor_torque_factor = 0.32;
#elif defined(__JORDANWAY__)
  bc.p_gain = 6.0;
  bc.d_gain = 0.45;
  bc.i_gain = 3.90;
  bc.motor_torque_factor = 0.55;
#elif defined(__ZOOMBOARD__)
  bc.p_gain = 2.5;
  bc.d_gain = 0.13;
  bc.i_gain = 3.0;
  bc.motor_torque_factor = 1.0;
#elif defined(__UNICYCLE2__)
  bc.p_gain = 3.2;
  bc.d_gain = 0.10;
  bc.i_gain = 0.0;
  bc.motor_torque_factor = 0.70;
  bc.fwd_accel_coupling = 0.6;
#else
#error "No HW"
#endif

#ifdef DO_STEERING
  bc.yaw_steer_gain = 0.05;
  bc.yaw_steer_speed_thresh = 0.1;
#endif

#if defined(__SEGWELL2__)
  bc.hard_speed_lim = 0.99;
  bc.fwd_speed_lim = 0.75;
  bc.rev_speed_lim = 0.60;
  bc.beep_speed_lim = 0.88;
  bc.fwd_accel_coupling = 0.3;
  bc.rev_stick_shake_speed_lim = 0.40;
#elif defined(__JORDANWAY__)
  bc.hard_speed_lim = 0.99;
  bc.fwd_speed_lim = 0.75;
  bc.rev_speed_lim = 0.60;
  bc.beep_speed_lim = 0.88;
  bc.rev_stick_shake_speed_lim = 0.40;
#elif defined(__ZOOMBOARD__)
  bc.hard_speed_lim = 0.99;
  bc.fwd_speed_lim = 0.70;
  bc.rev_speed_lim = 0.60;
  bc.beep_speed_lim = 0.80;
  bc.rev_stick_shake_speed_lim = 0.40;
#elif defined(__UNICYCLE2__)
  bc.hard_speed_lim = 0.99;
  bc.fwd_speed_lim = 0.75;
  bc.rev_speed_lim = 0.75;
  bc.beep_speed_lim = 0.88;
  bc.rev_stick_shake_speed_lim = 10.00;
#else
#error "No HW"
#endif

#ifdef DO_STEERING
  bc.steer_rate_num = 0.100;
  bc.steer_rate_den = 0.200;
  bc.yaw_target_mult = 7.0;
#endif

  bc.balance_torque_lim = 1.0;

  bc.fuse_alarm_thresh = 0.35; // 35 amps. We have 40 amp fuses

#if defined(__BAL3__) || defined(__BAL4__)
  bc.crossover_boost = 0.01;
#else
  bc.crossover_boost = 0.015;
#endif

  eeprom_read_block(&bh, 0, sizeof(bh));
  if (1 || (((uint8_t *)&bh)[0]==0xff && ((uint8_t *)&bh)[1]==0xff && ((uint8_t *)&bh)[2]==0xff  && ((uint8_t *)&bh)[3]==0xff) ||
      !bal_hwadj_validate(&bh)) {

    memset(&bh, 0, sizeof(bh));

#if defined(__SEGWELL2__)
    bh.steering_bias = +0.250;
    bh.yaw_bias = -0.035;
    bh.fwd_accel_bias = 1.65;
    bh.right_accel_bias = 0.0;
    bh.bat_voltage_mult = 1.5625;
#elif defined(__JORDANWAY__)
    bh.steering_bias = -0.078;
    bh.yaw_bias = +0.0;
    bh.fwd_accel_bias = -1.07;    // more positive means more backwards
    bh.right_accel_bias = 0.0;
    bh.bat_voltage_mult = 2.22;
#elif defined(__ZOOMBOARD__)
    bh.fwd_accel_bias = -0.3;
    bh.right_accel_bias = 0.0;
    bh.bat_voltage_mult = 2.0;
#elif defined(__UNICYCLE2__)
    bh.fwd_accel_bias = 0.05;
    bh.right_accel_bias = 0.0;
    bh.bat_voltage_mult = 3.2;  // correct for 33v nominal battery voltage
#else
#error "No HW"
#endif
  }
}

static uint16_t last_balance_tcnt3;
static uint8_t last_balance_s0;

void balance_init(void)
{
  balance_configure();
  if (!bal_config_validate(&bc)) {
    pkt_tx_start();
    pkt_tx_str_P(PSTR("!balance_validate fails\n"));
    pkt_tx_end();
  }

  last_balance_tcnt3=TCNT3;

  bs.mode = MODE_INITIAL;
  bs.balance_torque=0.0;
#ifdef DO_DRIVE
  bs.drive_age = 1000.0;
#endif
#ifdef DO_BAT
  bs.bat_speed_lim = 0.5;
  bs.bat.fullloadvoltage = 0.7;
#endif

  set_motor_idle();
  set0_accel_st();
}

void do_crossover(float *x, float boost, float factor)
{
  if (*x>=0.0) {
    *x += fmin(boost, *x * factor);
  } else {
    *x -= fmin(boost, - *x * factor);
  }
}


/*
  Speed : 

  Motors: 24 volts is 240 RPM. With a 20" diameter wheel, that's 14.3 mph

  bat_voltage is volts/50

  MotorVoltage = bat_voltage*50 * PWM

  MotorVoltage = 50/1.5625 * cur_speed_est
  MotorVoltage = 32.0 * cur_speed_est

  This is the speed at which it starts tilting, but it can go 0.08
  beyond before integral limiting kicks in.

  speed_lim=0.75 => 24.0 volts => 14.3 mph
  speed_lim=0.7 => 13.3 mph (14.9 at integral limit)
  speed_lim=0.6 => 11.5 mph (13.0 at integral limit)
  speed_lim=0.4 => 7.6 mph


  Assume CG is 3 feet above axle. So a angrate of 1 (radians/sec) corresponds to a speed difference of
  2.04 mph, which corresponds to a cmd change of 0.107

*/

#ifdef DO_MISSION_LOG
bal_mission_log_ent mission_accum;
float time_since_mission_save;
#endif

uint16_t samples[ADCNO_END];
uint8_t last_samplecount[ADCNO_END];
float time_since_bat_update;

uint8_t bal_tickno;            // Counter used to time stick shake oscillator and other things

void balance(void)
{

  bal_tickno++;
  {
#if defined(DO_ENCODERS)
    uint32_t lastenc1=bs.enc1_pos;
    uint32_t lastenc2=bs.enc2_pos;
#endif

    // interval = time in seconds since we last updated things.
    uint16_t cur_tcnt3 = TCNT3;
    
#if defined(DO_ENCODERS)
    // Since we're taking derivatives, do this as close as possible to the time sample
    read_encoders(&bs.enc1_pos, &bs.enc2_pos);
#endif

    uint16_t ticks = cur_tcnt3 - last_balance_tcnt3;
    last_balance_tcnt3 = cur_tcnt3;

    // It's normally 3 mS. In case of a hiccup, we don't want to make too big a correction
    bs.interval = fmax(0.001, fmin(0.010, timer3_seconds_conv * (float)ticks));


#if defined(DO_ENCODERS)
    bs.enc1_vel = (bs.enc1_pos - lastenc1) / bs.interval;
    bs.enc2_vel = (bs.enc2_pos - lastenc2) / bs.interval;
#endif
  }

  /* 
     Accumulate real time. This will lose precision after about 4 hours of operation. It's
     currently only used for timestamping log packets, but if we want to log for a long
     time maybe we should use an integer counter or take it modulo 1000 or something.
  */
  bs.realtime += bs.interval; 

  /*
    Convert the ADC inputs to relevant variables. If you wire up your
    controller differently, these will have to change.
  */

#if defined(__BAL2__)
  bs.bat_voltage = samples[ADCNO_BAT_VOLTAGE]*sample_conv_unipolar * bh.bat_voltage_mult;
  bs.pitch_rate = (samples[ADCNO_PITCH_RATE]*sample_conv_bipolar - 1.0) * -2.1817; // convert to 20 mv/deg/sec to radians/sec
  bs.steering = (samples[ADCNO_STEERING]*sample_conv_bipolar - 1.0) * -3.75 + bh.steering_bias;
  bs.yaw_rate = (samples[ADCNO_YAW_RATE]*sample_conv_bipolar - 1.0) * 2.1817 + bh.yaw_bias;  // convert to radians/sec
  bs.knob1 = samples[ADCNO_KNOB1]*sample_conv_unipolar;
  bs.lmis = (samples[ADCNO_LMIS]*sample_conv_bipolar - 1.0) * L_MOTOR_POLARITY(-1.875); // Units are 100A
  bs.rmis = (samples[ADCNO_RMIS]*sample_conv_bipolar - 1.0) * R_MOTOR_POLARITY(-1.875);
  bs.b1is = (samples[ADCNO_B1IS]*sample_conv_bipolar - 1.0) * 1.875;
  bs.b2is = (samples[ADCNO_B2IS]*sample_conv_bipolar - 1.0) * 1.875;
  bs.batis = bs.b1is + bs.b2is;
  bs.fwd_accel = ((samples[ADCNO_FWD_ACCEL]*sample_conv_bipolar - 1.0) * -10.0 + bh.fwd_accel_bias);  // convert from 250 mv/G

#elif defined(__BAL3__)
  bs.bat_voltage = samples[ADCNO_BAT_VOLTAGE]*sample_conv_unipolar * bh.bat_voltage_mult;
  bs.pitch_rate = (samples[ADCNO_PITCH_RATE]*sample_conv_bipolar - 1.0) * -2.1817; // convert to 20 mv/deg/sec to radians/sec
#ifdef DO_STEERING
#ifdef __JORDANWAY__
  bs.steering = (samples[ADCNO_STEERING]*sample_conv_bipolar - 1.0) * +1.5 + bh.steering_bias;
#else
  bs.steering = (samples[ADCNO_STEERING]*sample_conv_bipolar - 1.0) * -1.5 + bh.steering_bias;
#endif
#ifdef __JORDANWAY__
  bs.yaw_rate = (samples[ADCNO_YAW_RATE]*sample_conv_bipolar - 1.0) * -2.1817 + bh.yaw_bias;  // convert to radians/sec
#else
  bs.yaw_rate = (samples[ADCNO_YAW_RATE]*sample_conv_bipolar - 1.0) * 2.1817 + bh.yaw_bias;  // convert to radians/sec
#endif
#endif
  bs.knob1 = samples[ADCNO_KNOB1]*sample_conv_unipolar;
  bs.knob2 = samples[ADCNO_KNOB2]*sample_conv_unipolar;
  bs.knob3 = samples[ADCNO_KNOB3]*sample_conv_unipolar;
  bs.lmis = (samples[ADCNO_LMIS]*sample_conv_bipolar - 1.0) * 0.625; // Units are 100A
  bs.rmis = (samples[ADCNO_RMIS]*sample_conv_bipolar - 1.0) * 0.625;
  bs.batis = (samples[ADCNO_BATIS]*sample_conv_bipolar - 1.0) * 0.625;
  bs.roll_rate = (samples[ADCNO_RIGHT_ACCEL]*sample_conv_bipolar - 1.0) * 2.1817;

  // ADXL0311 accelerometer, with a gain of 3.
  // Convert from 312*3 mv/G into Gs
  bs.fwd_accel = (samples[ADCNO_FWD_ACCEL]*sample_conv_bipolar - 1.0) * 2.67 + bh.fwd_accel_bias;  
  bs.right_accel = (samples[ADCNO_RIGHT_ACCEL]*sample_conv_bipolar - 1.0) * 2.67 + bh.right_accel_bias;

  // LM61 temp sensors report 600 mv + 10 mv/C
  // Convert so 0=0C, 1=1C
  if ((bal_tickno & 0x0f)==0) {
    bs.pwmtempa = (samples[ADCNO_PWMTEMPA]*sample_conv_unipolar - 0.120) * 5.0;
    bs.pwmtempb = (samples[ADCNO_PWMTEMPB]*sample_conv_unipolar - 0.120) * 5.0;
    bs.pwmtempc = (samples[ADCNO_PWMTEMPC]*sample_conv_unipolar - 0.120) * 5.0;
    bs.pwmtempd = (samples[ADCNO_PWMTEMPD]*sample_conv_unipolar - 0.120) * 5.0;
  }

#elif defined(__BAL4__)
  bs.bat_voltage = samples[ADCNO_BAT_VOLTAGE]*sample_conv_unipolar * bh.bat_voltage_mult;
  bs.pitch_rate = (samples[ADCNO_PITCH_RATE]*sample_conv_bipolar - 1.0) * 3.491; // convert to 12.5 mv/deg/sec to radians/sec
  bs.knob1 = samples[ADCNO_KNOB1]*sample_conv_unipolar;
  bs.knob2 = samples[ADCNO_KNOB2]*sample_conv_unipolar;
  bs.knob3 = samples[ADCNO_KNOB3]*sample_conv_unipolar;
  bs.lmis = (samples[ADCNO_LMIS]*sample_conv_bipolar - 1.0) * 0.625; // Units are 100A
  bs.batis = (samples[ADCNO_BATIS]*sample_conv_bipolar - 1.0) * 0.625;

  // ADXL0311 accelerometer, with a gain of 3.
  // Convert from 312*3 mv/G into Gs
  bs.fwd_accel = (samples[ADCNO_FWD_ACCEL]*sample_conv_bipolar - 1.0) * 2.67 + bh.fwd_accel_bias;  

#else
#error "No hw defined"
#endif

  if (bs.mode==MODE_INITIAL) {
    if (0) {
    }
    else if (0 && bs.knob2 > 0.9) {
      bs.mode=MODE_HWADJ;
      update_hwadj(1.0);
    }
    else if (0 && bs.knob2 < 0.1) {
      bs.mode=MODE_TEST_MOTORS;
    }
    else {
      if (bs.fwd_accel < 0.2 && bs.fwd_accel > -0.2) {
        bs.mode=MODE_RUNNING;
      } else {
        bs.mode=MODE_TIPPED;
        bs.mode_reason=1;
      }
    }
  }

  if (bs.mode==MODE_TEST_MOTORS) {

    bs.left_pwm = fmin(0.3, bs.realtime / 20.0);
#ifdef DO_STEERING
    bs.right_pwm = bs.left_pwm*0.5;
    set_motors(bs.left_pwm, bs.right_pwm);
#else
    set_motors(bs.left_pwm);
#endif

    return;
  }
  else if (bs.mode==MODE_HWADJ_DONE) {
    set1_beeper();
    return;
  }
  else if (bs.mode==MODE_HWADJ) {
    update_hwadj(0.01);
    if (bs.realtime > 2.0) {
      set0_beeper();
      cli();
      eeprom_write_block(&bh, 0, sizeof(bh));
      sei(); // Ugh. eeprom_write does cli but no corresponding sei.
      bs.mode=MODE_HWADJ_DONE;
    }
    return;
  }
  
  bs.fwd_accel_net = bs.fwd_accel + bs.balance_torque*bc.fwd_accel_coupling;

  gyro_sample_rate(&bs.pitch_filter, bs.pitch_rate, bs.interval);
  gyro_sample_angle(&bs.pitch_filter, bs.fwd_accel_net, bs.interval);

  bs.gain_reduction=1.0;

#ifdef DO_DRIVE
  if (bs.drive_age < 1.0) {
    bs.drive_age += bs.interval;

    lpf_update(&bs.drive_tilt, 0.3, bs.interval, 2.0 * (bs.drive.speed_targ - bs.cur_speed_est) + 0.1);
    lpf_update(&bs.drive_steer, 0.3, bs.interval, bs.drive.steering_targ); // wrong
    bs.gain_reduction = 0.2;
  }
  else if (bs.drive_age < 2.0) {
    bs.drive_age += bs.interval;

    lpf_update(&bs.drive_steer, 0.3, bs.interval, 0.0);
    lpf_update(&bs.drive_tilt, 0.3, bs.interval, 0.0);

    bs.gain_reduction = 0.4;
  } 
  else {
    bs.drive_tilt=0.0;
    bs.drive_steer=0.0;
  }
#endif


  lpf_update(&bs.lpf_angle, 0.04, bs.interval, bs.pitch_filter.angle
#ifdef DO_DRIVE
             + bs.drive_tilt
#endif
             );
#ifdef __BAL4__
  lpf_update(&bs.lpf_angrate, 0.050, bs.interval, bs.pitch_filter.rate);
#else
  lpf_update(&bs.lpf_angrate, 0.015, bs.interval, bs.pitch_filter.rate);
#endif
#ifdef DO_STEERING
  lpf_update(&bs.lpf_steering, 0.1, bs.interval, bs.steering 
#ifdef DO_DRIVE
             + bs.drive_steer
#endif
             );

  lpf_update(&bs.lpf_yaw_rate, 0.05, bs.interval, bs.yaw_rate);

  /* Since we can dial down the speed limit, we don't want this changing too fast */
  lpf_update(&bs.lpf_knob1, 0.5, bs.interval, bs.knob1);

#endif

  bs.cur_cg_speed_est = bs.cur_speed_est;

  bs.overspeed=0.0;

#ifdef DO_BAT
  float fsl = fmin(bc.fwd_speed_lim, bs.bat_speed_lim);
  float rsl = fmin(bc.rev_speed_lim, bs.bat_speed_lim);
#else
  float fsl = bc.fwd_speed_lim;
  float rsl = bc.rev_speed_lim;
#endif

#if !defined(__UNICYCLE2__)
  // Adjust speed limits down to 25% of maximum with knob
  fsl *= (0.5 + 0.5*bs.lpf_knob1);
  rsl *= (0.5 + 0.5*bs.lpf_knob1);
#endif

  /* Check if we're over speed and arrange to tilt back. This is a bit convoluted. */
  if (bs.cur_cg_speed_est > fsl) {
    bs.overspeed = (bs.cur_cg_speed_est - fsl); // positive
    bs.overspeed_integral += 2.0 * bs.interval * fmax(0.0, bs.overspeed-0.08);
  }
  else if (bs.cur_cg_speed_est < -rsl) {
    bs.overspeed = (bs.cur_cg_speed_est + rsl); // negative
    bs.overspeed_integral += 2.0 * bs.interval * fmin(0.0, bs.overspeed+0.08);
  }
  else if (bs.overspeed_integral>0.0) {
    float not_overspeed = fmin(0.0, (bs.cur_cg_speed_est - fsl)); // should be negative
    bs.overspeed_integral += 4.0 * bs.interval * fmin(-0.02, not_overspeed);
    if (bs.overspeed_integral<0.0) bs.overspeed_integral=0.0;
  }
  else if (bs.overspeed_integral<0.0) {
    float not_overspeed = fmax(0.0, (bs.cur_cg_speed_est + rsl)); // should be positive
    bs.overspeed_integral += 4.0 * bs.interval * fmax(0.02, not_overspeed);
    if (bs.overspeed_integral>0.0) bs.overspeed_integral=0.0;
  }
  bs.overspeed_integral=flim(bs.overspeed_integral, -0.5, 0.5);

  bs.stick_shake=0.0;
  if (bs.cur_cg_speed_est < -bc.rev_stick_shake_speed_lim*(0.5+0.5*bs.lpf_knob1)) {
    bs.stick_shake=0.03;
  }

  /* Add the tilt to the measured angle */
  bs.lpf_angle_corrected = bs.lpf_angle + flim(0.9*bs.overspeed + bs.overspeed_integral, -0.4, 0.4);

  /* Only do the integral term once we're stable */
  if (bs.softstart==1.0) {
    bs.lpf_angintegral = flim(bs.lpf_angintegral + bs.lpf_angle_corrected*bs.interval * 4.0, -0.1, 0.1);
  }

  /* If we've tipped past 0.6 radians (45 degrees) abandon hope.
     This happens, for example, when the wheels slip on wet grass or snow and it falls over.
     It's better to give up than keep spinning the wheels trying to right ourselves.
     We probably can't recover anyway.
  */
  if (bs.mode!=MODE_TIPPED) {
    if (bs.lpf_angle > 0.6) {
      bs.mode=MODE_TIPPED;
      bs.mode_reason=2;
    }
    else if (bs.lpf_angle < -0.6) {
      bs.mode=MODE_TIPPED;
      bs.mode_reason=3;
    }
  }

  /*
    Below this we can't count on enough voltage to keep ourselves or the PWM drives running.
  */
  if (bs.bat_voltage < 0.35) {
    bs.mode=MODE_TIPPED;
    bs.mode_reason=4;
  }

  if (bs.mode!=MODE_RUNNING) {
    // too tippy to try to recover.
    set_motor_idle();
    return;
  }

  /*
    The actual feedback loop between tilt angle and motor torque
   */

  float bat_factor = fmax(0.45, bs.bat_voltage);

  bs.balance_torque = (bc.p_gain*bs.lpf_angle_corrected + bc.d_gain*bs.lpf_angrate + bc.i_gain*bs.lpf_angintegral) * bs.softstart * bs.gain_reduction;
  

  /*
    Limit balance_torque to the amount we can actually produce, based on the PWM limit values.
    Avoiding having enormous balance_torques during PWM saturation helps keep cur_speed_est reasonable.
  */
  int8_t at_limit=0;
  if (bs.balance_torque>=0.0) {
    bs.bt_lim = fmin((bc.hard_speed_lim * bat_factor - bs.cur_speed_est) / bc.motor_torque_factor, bc.balance_torque_lim);
    if (bs.balance_torque > bs.bt_lim) {
      if (bs.bt_lim<0.0) {
        bs.balance_torque=0.0;
      } else {
        bs.balance_torque = bs.bt_lim;
      }
      at_limit=1;
    }
  } else {
    bs.bt_lim = fmax((-bc.hard_speed_lim * bat_factor - bs.cur_speed_est) / bc.motor_torque_factor, -bc.balance_torque_lim);
    if (bs.balance_torque < bs.bt_lim) {
      if (bs.bt_lim>=0.0) {
        bs.balance_torque=0.0;
      } else {
        bs.balance_torque = bs.bt_lim;
      }
      at_limit=1;
    }
  }

  /*
    Calculate the PWM value, roughly = current speed + k * desired torque / bat_voltage.
  */
  bs.cmd = (bs.cur_speed_est + bc.motor_torque_factor*bs.balance_torque) / bat_factor;

  /*
    Assume that torque makes us accelerate. Not very accurate, but the feedback is tight enough
    to compensate.
  */
  bs.cur_speed_est = bs.cur_speed_est + 0.5*bs.interval*bs.balance_torque;

  /*
    Avoid trusting the accelerometer when we're accelerating hard.
   */
  bs.pitch_filter.angle_noise = fmax(bs.pitch_filter.angle_noise, fabs(bs.balance_torque)*10.0);
  lpf_update(&bs.pitch_filter.angle_noise, 0.5, bs.interval, 0.0);
  
  /*
    Annoy the operator by adding a wobble.
   */
  if (bs.stick_shake!=0.0) {
    if (bal_tickno&8) {
      bs.cmd += bs.stick_shake;
    } else {
      bs.cmd -= bs.stick_shake;
    }
  }

  if (at_limit) {
    bs.full_speed_time += bs.interval;
    if (bs.full_speed_time > 0.5) {
      bs.mode=MODE_TIPPED;
      bs.mode_reason=5;
    }
  } else {
    bs.full_speed_time = 0.0;
  }

#ifdef CATCH_GLITCHES
  if (bs.cmd > 0.3 || bs.cmd < -0.3) {
    barf();
  }
#endif

  /* Ramp up softstart over the first second */
  if (bs.pitch_filter.angle_inited && bs.pitch_filter.rate_inited) {
    bs.softstart = fmin(1.0, bs.softstart + 1.0*bs.interval);
    bs.start_protection = fmin(1.0, bs.start_protection + 0.25*bs.interval);
  }


#ifdef DO_STEERING

  /* Maximum steering rate calculation. At zero speed you want to be able to spin like a top
     but at high speed it needs to be gentle. We do an asymptotic slope */
  bs.steer_rate = bc.steer_rate_num / (bc.steer_rate_den * (2.0 - bs.lpf_knob1) + bs.cur_speed_est*bs.cur_speed_est) * bs.softstart;


  /* 
     Set yaw gain. Gain is constant except at low speed where it drops off. I like to drop it off because
     you don't need it for stability, and it lets you twist the scooter with your feet. 
  */
  float yaw_steer_gain_adj = fmin(1.0, fabs(bs.cur_speed_est) / bc.yaw_steer_speed_thresh) * bc.yaw_steer_gain;

  float desired_steer = bs.steer_rate*bs.lpf_steering;

  // Make sure to saturate at the same values as the gyro saturates.
  bs.target_yaw = flim(desired_steer * bc.yaw_target_mult, -2.1817, 2.1817);
  
  // Differential steering
  bs.steer_diff = desired_steer + yaw_steer_gain_adj * (bs.target_yaw - bs.lpf_yaw_rate);
  
  bs.left_pwm =  flim(bs.cmd - bs.steer_diff, -bs.start_protection, bs.start_protection);
  bs.right_pwm = flim(bs.cmd + bs.steer_diff, -bs.start_protection, bs.start_protection);
#else
  bs.left_pwm = bs.cmd;
#endif

  /* Boost the PWM signal around 0, to overcome motor stiction */
  do_crossover(&bs.left_pwm, bc.crossover_boost, 2.0);
#ifdef DO_STEERING
  do_crossover(&bs.right_pwm, bc.crossover_boost, 2.0);
#endif

#ifdef DO_STEERING
  set_motors(+bs.left_pwm, +bs.right_pwm);
#else
  set_motors(+bs.left_pwm);
#endif

#if defined(__BAL2__) || defined(__BAL3__) || defined(__BAL4__)
  /* If we're hitting the speed limit on any wheel, beep */
  if (bs.left_pwm > bc.beep_speed_lim || 
      bs.left_pwm < -bc.beep_speed_lim ||
      at_limit ||
#ifdef DO_STEERING
      bs.right_pwm > bc.beep_speed_lim || bs.right_pwm < -bc.beep_speed_lim ||
      bs.steering > 1.0 || bs.steering < -1.0 ||
#endif
#ifdef DO_BAT
      (bs.bat_voltage < 0.5 && (bal_tickno & 0x38) == 0x00) ||
      (bs.bat_speed_lim < 0.25 && (bal_tickno & 0x38) == 0x20) ||
#endif
#if defined(__BAL2__)
      ((fabs(bs.b1is) > bc.fuse_alarm_thresh || fabs(bs.b2is) > bc.fuse_alarm_thresh) && (bal_tickno & 0x0c) == 0x08) ||
#endif
#if defined(__BAL3__) || defined(__BAL4__)
      ((fabs(bs.batis) > bc.fuse_alarm_thresh) && (bal_tickno & 0x0c) == 0x08) ||
#endif
#if defined(__BAL3__) && !defined(__JORDANWAY__)
      ((bs.pwmtempa > 0.7 || bs.pwmtempb > 0.7 || bs.pwmtempc > 0.7 || bs.pwmtempd > 0.7) && (bal_tickno&0x40)==0x00) ||
#endif
      (1 && bs.realtime < 0.2) ||
       0
      ) {
    set0_beeper();
  } else {
    set1_beeper();
  }
#endif

#ifdef DO_BAT

  float weight = bs.interval;
  bs.bat.s1 += weight;
  bs.bat.si += bs.batis * weight;
  bs.bat.sii += bs.batis * bs.batis * weight;
  bs.bat.sv += bs.bat_voltage * weight;
  bs.bat.svv += bs.bat_voltage * bs.bat_voltage * weight;
  bs.bat.siv += bs.bat_voltage * bs.batis * weight;

  time_since_bat_update += bs.interval;

  /* Every second, update bat_speed_lim by estimating the battery resistance */

  if (time_since_bat_update >= 0.1) {
    bs.bat.delta = bs.bat.s1 * bs.bat.sii - bs.bat.si * bs.bat.si;
    float idelta = (bs.bat.delta<=0.0) ? 10.0 : 1.0/bs.bat.delta;
    bs.bat.resistance = (bs.bat.s1 * bs.bat.siv - bs.bat.sv * bs.bat.si) * idelta; // normally negative
    bs.bat.ocvoltage = (bs.bat.sii * bs.bat.sv - bs.bat.si * bs.bat.siv) * idelta;
  
    bs.bat.sigma_ocvoltage = sqrt(bs.bat.sii * idelta);
    bs.bat.sigma_resistance = sqrt(bs.bat.s1 * idelta);

    
    float max_current = 0.4; // The maximum current I want to be able to draw (0.4=40 amps)
    float max_bt = 0.2;

    bs.bat.fullloadvoltage = fmin(0.8, bs.bat.ocvoltage) + max_current * fmin(-0.15, bs.bat.resistance);

    // compare to the way cmd is generated from cur_speed_est and
    // balance_torque. This allows balance_torque up to 0.3
    // Maybe I should weight this according to delta, since that reflects how much variation
    // in current there was.

    if (bs.bat.delta>2.0) {
      lpf_update(&bs.bat_speed_lim,
                 40.0,
                 time_since_bat_update * bs.bat.delta / 10.0,
                 fmax(0.15, bs.bat.fullloadvoltage * 1.5626 - bc.motor_torque_factor * max_bt));
    }
    time_since_bat_update = 0.0;

    bs.bat.s1 *= 0.99;
    bs.bat.si *= 0.99;
    bs.bat.sii *= 0.99;
    bs.bat.sv *= 0.99;
    bs.bat.svv *= 0.99;
    bs.bat.siv *= 0.99;
                 
  }
#endif

#ifdef DO_MISSION_LOG
  lpf_update(&mission_accum.bat_voltage, 0.5, bs.interval, bs.bat_voltage);
  lpf_update(&mission_accum.bat_current, 0.5, bs.interval, bs.batis);
  lpf_update(&mission_accum.bat_resistance, 0.5, bs.interval, bs.bat.resistance);
  lpf_update(&mission_accum.bat_ocvoltage, 0.5, bs.interval, bs.bat.ocvoltage);
  lpf_update(&mission_accum.cur_speed_est, 0.5, bs.interval, bs.cur_speed_est);

  time_since_mission_save += bs.interval;
  if (time_since_mission_save >= 1.0) {
    mission_accum.realtime = bs.realtime;
    
    eeprom_write_bg(&mission_accum, sizeof(mission_accum),
                    mission_index*sizeof(mission_accum));
    mission_index = (mission_index+1) % N_MISSION_ENTRIES;
  }
#endif

}

void
timer_init(void)
{
  /*
    Timer 1:

    PWM mode is "PWM, Phase Correct, 9-bit"
    14 MHz / 1 / 512 / 2 gives 14 kHz, just inaudible

  */

  TCCR1A = 0 |
    (1<<COM1A1) | (1<<COM1A0) | // clear on match up, set on match down
    (1<<COM1B1) | (0<<COM1B0) | // set on match up, clear on match down
#if OCR1_MAX==1023
    (1<<WGM11) | (1<<WGM10);
#elif OCR1_MAX==511
    (1<<WGM11);
#elif OCR1_MAX==255
    (1<<WGM10);
#else
#error "Unknown OCR1_MAX"
#endif

  TCCR1B = 0 |
    (1<<CS10); // prescaler divide by 1


  /*
    Timer 3:
    
    Frequency is about 14 kHz, so it overflows every 4.5 seconds
  */

  TCCR3A = 0;
  TCCR3B = 0 |
    (1<<CS32) | (0<<CS31) | (1<<CS30); // prescale clk/1024
  timer3_seconds_conv = 1.0/((float)CLOCK_SPEED/1024);

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

  uint8_t cmd=pkt[0];
  /* Send current bal_state */
  if (cmd=='C' && pktlen==1) {
    pkt_tx_start();
    pkt_tx_uint8('c');
    for (uint8_t i=0; i<sizeof(bs); i++) {
      pkt_tx_uint8(((uint8_t *)&bs)[i]);
    }
    pkt_tx_end();
  }
  /* Send current bal_config */
  else if (cmd=='R' && pktlen==1) {
    pkt_tx_start();
    pkt_tx_uint8('r');
    for (uint8_t i=0; i<sizeof(bc); i++) {
      pkt_tx_uint8(((uint8_t *)&bc)[i]);
    }
    pkt_tx_end();
  }
  /* Send current bal_hwadj */
  else if (cmd=='H' && pktlen==1) {
    pkt_tx_start();
    pkt_tx_uint8('h');
    for (uint8_t i=0; i<sizeof(bh); i++) {
      pkt_tx_uint8(((uint8_t *)&bh)[i]);
    }
    pkt_tx_end();
  }

  /* Accept new bal_hwadj */
  else if (cmd=='A' && pktlen==1+sizeof(struct bal_hwadj)) {
    struct bal_hwadj *newbh = (struct bal_hwadj *)(pkt+1);
    pkt_tx_start();
    pkt_tx_uint8('a');
    if (bal_hwadj_validate(newbh)) {
      bh = *newbh;
      pkt_tx_uint8(1);
    } else {
      pkt_tx_uint8(0);
    }
    pkt_tx_end();
  }

  /* Accept new bal_config */
  else if (cmd=='W' && pktlen==1+sizeof(struct bal_config)) {
    struct bal_config *newbc = (struct bal_config *)(pkt+1);
    pkt_tx_start();
    pkt_tx_uint8('w');
    if (bal_config_validate(newbc)) {
      bc = *newbc;
      pkt_tx_uint8(1);
    } else {
      pkt_tx_uint8(0);
    }
    pkt_tx_end();
  }

#ifdef DO_DRIVE
  /* Accept drive command */
  else if (cmd=='D' && pktlen==1+sizeof(struct bal_drive)) {
    struct bal_drive *newbd = (struct bal_drive *)(pkt+1);
    pkt_tx_start();
    pkt_tx_uint8('d');
    if (bal_drive_validate(newbd)) {
      bs.drive = *newbd;
      bs.drive_age = 0.0;
      pkt_tx_uint8(1);
    } else {
      pkt_tx_uint8(0);
    }
    pkt_tx_end();
  }
#endif

  /* Reboot request, handy for uploading new firmware without hitting the big red button */
  else if (cmd=='Q' && pktlen==6) {
    // Only 'l33t people can reboot it. 
    // (This is mainly just so line noise on the serial port doesn't hose me)
    if (pkt[1] != '3' || pkt[2] != 'b' || pkt[3] != '0' || pkt[4] != '0' || pkt[5] != 't') return;
      
    set_motor_idle();

    // I could be more careful and turn off all the interrupt sources, but this should work too.
    cli();
    void (*funcptr)();
    funcptr = (void (*)()) 0x1f800L; // XXX fragile, specific to ATMega128
    funcptr();
  }

  else {
    // discard cmd
  }
}

void barf()
{
  adc_disable();
  set_motor_idle();
  while (1) {
    if (uart_tx_empty()) {
      dump_adc_log();
    }
  }
}

int8_t led_counter;
void update_leds()
{
#ifdef __BAL4__
  led_counter=(led_counter+1)&15;
  
  float angle_thresh=(led_counter+1) * 0.03125;
  
  set0_led1();
  set0_led2();
  set0_led3();

  if (bs.lpf_angle > angle_thresh) set1_led1();
  else if (bs.lpf_angle < -angle_thresh) set1_led2();

  //if (led_counter==0) set1_led3();
#endif
}

/* Sending data just when the Free2Move serial port dongle is powered on seems to hose it,
   so we give the first beacon after 2 seconds */
float timetonext_beacon=2.0;

#define VERSION "bal 2.0"

int main( void )
{
  timer_init();
  io_init();
  uart_init_tx();
  uart_init_rx();

  sei();

  PKT_ANNOUNCE_VERSION(VERSION);

  adc_init();
  set1_led2();
  adc_stabilize();
  set0_led2();
  balance_init();

  set0_led1();
  set0_led2();

  while (1) {


    if (uart_tx_empty()) {
      handle_rx();
    }

    //dump_adc_log();

    if (adc_collect_samples(samples, &last_balance_s0, ADCNO_BAT_VOLTAGE)) {
      /* The brightness of LED2 gives us an idea how much of the time we're calculating
         as opposed to waiting for new ADC samples */
      update_leds();
      balance();

      timetonext_beacon -= bs.interval;
    }

    /* Send periodic beacons to inform balctl that we're here */
    if (timetonext_beacon <= 0.0 && uart_tx_empty()) {
      timetonext_beacon = 0.5;
      PKT_ANNOUNCE_VERSION(VERSION);
      err_tx_start();
      err_tx_comm();
      err_tx_end();
    }
  }
}
