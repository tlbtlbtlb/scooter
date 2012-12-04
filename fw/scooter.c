/*
 *
 * Code for a wheeled balancing robot. If you stand on it, it can be a scooter too.
 *
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
 */


#include "debug.h"
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <math.h>

#include "timer.h"
#include "uart.h"
#include "string.h"
#include "adc.h"

float timer1_seconds_conv;

/*
  Do one iteration of a low-pass filter. *state will track input, but only slowly
  tc is the time constant and interval is the length of 1 time step
 */
void lpf_update(float *state, float tc, float interval, float input)
{
  float frac=interval/tc;
  if (frac>1.0) frac=1.0;
  *state = input*frac + *state * (1.0-frac);
}

/* Math utilities */
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

/*
  A basic 2-input Kalman filter for a rate gyro stabilized with a
  2-axis accelerometer.

  In a nutshell...
  
  We have two inputs, a measured angular rotation rate (degrees per
  second) and a forward-backward acceleration. We integrate the
  angular rate to get the angle. Except that the angular rate has a
  bias on it, simulating an apparent rotation all the time at a small
  rate that varies with temperature. So we have to correct it with the
  accelerometer. The accelerometer is a tiny mass on a spring that
  moves forward-backward in the frame of reference of the scooter. At
  rest, it will read positive if the scooter is tipped forward and
  negative if tipped backwards, since gravity pulls it forwards or
  backwards. However, it also moves a lot due to the scooter itself
  acclerating. So we use the accelerometer as a long-term correction
  and the rate gyro for short-term changes.

  This isn't really a Kalman filter, since that particular brand of
  filter dynamically estimates the noise on each signal. I initially
  did that but it didn't work well, since the noise is not
  independent, additive white Gaussian noise. Instead, how much we
  trust the acclerometer depends on how much we're accelerating.

  This isn't perfect; the scooter can wobble slowly by a couple
  degrees over several seconds, and especially after a sharp maneuver.
  But it's barely noticeable riding it; you get used to adjusting for
  it.
 */

struct gyro_filter {
  float angle;
  float rate_bias;
  float rate;
  float angle_noise;

  int rate_inited;
  int angle_inited;
};

void gyro_sample_rate(struct gyro_filter *it, float in_rate, float interval)
{
  if (!it->rate_inited) {
    it->rate_bias = in_rate;
    it->rate_inited=1;
  }
  
  it->rate = in_rate - it->rate_bias;
  it->angle += it->rate * interval;

  it->rate_bias += it->rate * 0.3 * interval;

  /* for debugging */
  LOG_uint8_t('r');
  LOG_float(in_rate);
  LOG_float(it->angle);
  LOG_float(it->rate);
}

void gyro_sample_angle(struct gyro_filter *it, float in_y, float in_z, float interval)
{
  float angle_err;
  float in_angle=in_y*2.0;

  if (!it->angle_inited) {
    it->angle = in_angle;
    it->angle_inited=1;
  }
  
  angle_err = in_angle - it->angle;

  it->angle += angle_err * interval / (2.0 + it->angle_noise);
  
  /* for debugging */
  LOG_uint8_t('a');
  LOG_float(in_y);
  LOG_float(in_z);
  LOG_float(in_angle);
  LOG_float(it->rate_bias);
}

struct gyro_filter pitch_filter;

/* Send a command to the RoboteQ controller out the serial port. It's
   a goofy protocol where you say [AaBb][2 hex digits], where a/b
   denotes the left or right channel, and capital means forward and
   lowercase means reverse.
*/
void set_motors(float left, float right)
{
  int16_t lefti = (int16_t)(left*127.9);
  int16_t righti = (int16_t)(right*127.9);

  if (lefti<-127) lefti=-127;
  if (lefti>127) lefti=127;
  if (righti<-127) righti=-127;
  if (righti>127) righti=127;

  /* for debugging */
  LOG_uint8_t('m');
  LOG_float(left);
  LOG_float(right);
  LOG_int16_t(lefti);
  LOG_int16_t(righti);

#ifdef DEBUG
  return;
#endif

  /* putc writes bytes out the serial port */
  putc('!');
  putc(righti>=0 ? 'B' : 'b');
  put_uint8_t(righti<0 ? -righti : righti);
  putc(13);

  /* It's the A motor (channel 1, right side of scooter) that sometimes locks.
     I accuse the RoboteQ driver of having a bug.
     [Cosma confirms this; the latest version has a fix] */
  putc('!');
  putc(lefti>=0 ? 'A' : 'a');
  put_uint8_t(lefti<0 ? -lefti : lefti);
  putc(13);

}

static uint16_t last_balance_timer1;
static uint8_t balance_counter;

static float lpf_angle_presc;
static float lpf_angrate;
static float lpf_steering;
static float lpf_angintegral;
static int tipped=0;
static float softstart;
static float overspeed;
static float overspeed_integral;
static float balance_torque;
static float cur_speed_est;

/*
  These parameters will have to change for different hardware, electronics, 
  weight, batteries, etc.
*/
static float p_gain = 5.0;
static float d_gain = 0.4;
static float i_gain = 3.0;
/* speed_lim is the speed at which it starts tilting back to slow it down */
static float speed_lim=0.6;
/* hard_speed_lim is the maximum wheel speed that will ever be set */
static float hard_speed_lim=1.0;

/* 
   On TLB's scooter, with a 42 volt battery and NPC-T62 wheel motors and
   14" tires, the motor control PWM maps to speed as follows:
   1.0 is 15 mph
   0.5 is 7.5 mph
   0.4 is 6 mph
*/

void balance(void)
{
  float cmd, left, right, steer_rate, sample_conv, interval, lpf_angle;
  uint16_t samples[8];
  uint8_t samplecount;
  float steer_knob, pitch_rate, az, ay;

  /*
    If we can't get the next command out right now, don't bother.
    It's better to wait and work from the newest data than to build up a queue
    of commands waiting to be transmitted
   */
  if (!uart_send_empty()) return;

  /*
    Collect at least 25 samples. This gives around 100/sec total.
   */
  samplecount = adc_collect_samples(samples, 25);
  if (!samplecount) return;
  /* Divide total by number of samples */
  sample_conv = (1.0/512.0) / (float)samplecount;
  
  /*
    Convert the ADC inputs to relevant variables. If you wire up your
    controller differently, these will have to change.
   */
  steer_knob =   samples[0]*sample_conv - 1.0;
  // gain_knob  =   samples[1]*sample_conv - 1.0;
  pitch_rate =   samples[2]*sample_conv - 1.0;
  // roll_rate = samples[3]*sample_conv - 1.0;
  az =           samples[4]*sample_conv - 1.0;
  ay =           samples[5]*sample_conv - 1.0;
  // ax =        samples[7]*sample_conv - 1.0;
  // yaw_rate =  samples[8]*sample_conv - 1.0;

  /*
    interval = time in seconds since we last updated things.
  */
  {
    uint16_t new_timer1=TCNT1;
    uint16_t ticks = new_timer1 - last_balance_timer1;
    last_balance_timer1=new_timer1;
    interval = timer1_seconds_conv * (float)ticks;
  }

  /* Do all the real work ... */
  gyro_sample_rate(&pitch_filter, pitch_rate*4.0, interval);
  gyro_sample_angle(&pitch_filter, az-0.3*balance_torque, ay, interval);

  lpf_update(&lpf_angle_presc, 0.05, interval, -0.03 - pitch_filter.angle);
  lpf_update(&lpf_angrate, 0.04, interval, 0.0 - pitch_filter.rate);
  lpf_update(&lpf_steering, 0.1, interval, steer_knob);

  overspeed=0.0;

  /* Check if we're over speed and arrange to tilt back. This is a bit convoluted. */
  if (cur_speed_est > speed_lim) {
    overspeed = (cur_speed_est - speed_lim);
    overspeed_integral += 0.7 * interval * fmax(0.05, overspeed);
  }
  else if (cur_speed_est < -speed_lim) {
    overspeed = (cur_speed_est + speed_lim);
    overspeed_integral += 0.7 * interval * fmax(0.05, overspeed);
  }
  else if (overspeed_integral>0.0) {
    overspeed_integral -= 0.02*interval;
    if (overspeed_integral<0.0) overspeed_integral=0.0;
  }
  else if (overspeed_integral<0.0) {
    overspeed_integral += 0.02*interval;
    if (overspeed_integral>0.0) overspeed_integral=0.0;
  }
  
  /* Add the tilt to the measured angle */
  lpf_angle = lpf_angle_presc + 0.4*overspeed + overspeed_integral;

  if (softstart==1.0) {
    lpf_angintegral = flim(lpf_angintegral + lpf_angle*interval, -0.08, 0.08);
  }

  /* for debugging */
  LOG_uint8_t('b');
  LOG_uint16_t(TCNT1);
  LOG_float(interval);
  LOG_float(lpf_angle);
  LOG_float(lpf_angrate);

  /* If it has tipped over, give up rather than spinning the wheels in the air */
  if (lpf_angle > 0.8 || lpf_angle < -0.8) tipped=1;
  
  if (tipped) {
    // too tippy to try to recover.
    set_motors(0.0, 0.0);
    return;
  }

  /* For testing */
  //gainmul = exp(1.5*gain_knob);
  //gainmul=0.9;

  /* The PID loop */
  balance_torque = (p_gain*lpf_angle + d_gain*lpf_angrate + i_gain*lpf_angintegral) * softstart;

  /* Assume larger noise when balance torque is large */
  pitch_filter.angle_noise = fmin(pitch_filter.angle_noise, fabs(balance_torque)*30.0);
  lpf_update(&pitch_filter.angle_noise, 0.5, interval, 0.0);
  
  /* The motor command is the current speed + the balance torque. So the balance torque is really
     a function of the resistance of the motor (and also the batteries) */
  cmd = cur_speed_est + (1.0+0.2*fabs(cur_speed_est))*balance_torque;
  /* Estimate the current speed */
  cur_speed_est += 1.3*interval*balance_torque;

  /* Don't exceed hard_speed_lim ever, since the PWM can't do more than 100% anyway.
     I gotta get one of those amplifiers that goes up to 11 */
  cmd=flim(cmd, -hard_speed_lim, hard_speed_lim);

  /* Allow fast steering at low speed, and slow steering at high speed. So you can spin around
     quickly in place, but touching the steering control doesn't make you veer off sideways */
  steer_rate = 0.080 / (0.3 + fabs(cur_speed_est)) * softstart;
  //steer_rate=0.0;

  /* softstart increases from 0 to 1 during the first 3 seconds after power-on, so it doesn't
     start with a jerk */
  if (pitch_filter.angle_inited && pitch_filter.rate_inited) {
    softstart = fmin(1.0, softstart + 0.3*interval);
  }

  /* Add the speed plus differential steering */
  left =  +(cmd + steer_rate*lpf_steering);
  right = +(cmd - steer_rate*lpf_steering);

  /* And send the command out the serial port */
  set_motors(left, right);
}

/* Called at power-up to read a few values from the ADCs before we
   start trusting them */
void stabilize_analog()
{
  uint16_t samples[8];
  int i;
  uint8_t samplecount;

  for (i=0; i<10; ) {
    samplecount = adc_collect_samples(samples, 1);
    if (samplecount) i++;
  }
}    

void
timer_init(void)
{
  /* Timer 1 at system clock / 64. We use this for general
     timekeeping, but no interrupts  */
  
  TCCR1A = 0x00;
  TCCR1B = 0x03;
  
  timer1_seconds_conv = 64.0 / 16e6; // assume 16 MHz
}

int main( void )
{
  int loopcnt;

  timer_init();
#ifdef DEBUG
  uart_init_for_debug();
#else
  uart_init_for_roboteq();
#endif

  sei();

#ifdef DEBUG
  puts("!scooter 12\n");
#endif

  /*
    The default mode for the RoboteQ is to only listen to the serial port if you send a bunch
    of CRs to it. But I changed mine to listen always.
  */
  if (0) {
    uint16_t last_tcnt1 = TCNT1;
    uint16_t interval_ticks = (uint16_t)(0.05 / timer1_seconds_conv);
    for (loopcnt=0; loopcnt<30; loopcnt++) {
      while ((TCNT1 - last_tcnt1) < interval_ticks) {}
      last_tcnt1=TCNT1;
      putc(13);
    }
  }

  adc_init();
  stabilize_analog();

  last_balance_timer1=TCNT1;
  while (1) {
    balance();
  }
}
