/*
 *
 * Firmware for a self-balancing unicycle. 
 *
 * (c) 2004 Trevor Blackwell <tlb@tlb.org>
 *  
 */

#include "debug.h"
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <math.h>

#include "uart.h"
#include "string.h"
#include "adc.h"

//#define WOBBLE_TEST 1

/*
  IO:

    PB0        Onboard LED
    PB5/OC1A   ALI -> OSMC pin 6
    PB6/OC1B   BLI -> OSMC pin 8

    PC1        Disable -> OSMC pin 4
    PC2        BHI -> OSMC pin 7
    PC3        AHI -> OSMC pin 5

    PF0/ADC0   Vbatt/10 -> OSMC pin 3
    PF1/ADC1   pitch rate gyro -> Z board pin ?
    PF2/ADC2   Z accel -> Z board pin ?

    PD3/ADC3   neutral pitch adjust

 */


float timer0_seconds_conv;

void extend_count(uint8_t cnt8, uint16_t *cnt16)
{
  uint8_t cnt16l=(uint8_t) *cnt16;
  uint8_t delta=(uint8_t) (cnt8 - cnt16l);
  *cnt16 += delta;
}

void lpf_update(float *state, float tc, float interval, float input)
{
  float frac=interval/tc;
  if (frac>1.0) frac=1.0;
  *state = input*frac + *state * (1.0-frac);
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

/*
  A basic 2-input Kalman filter for a rate gyro stabilized with a 2-axis accelerometer.
  It's really wrong to use the angle; I should handle sin and cosine terms separately.
 */
struct gyro_filter {
  float angle;
  float rate_bias;
  float rate;
  float angle_noise;

  uint8_t rate_inited;
  uint8_t angle_inited;
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

  uart_tx_uint8('r');
  uart_tx_float(in_rate);
  uart_tx_float(it->angle);
  uart_tx_float(it->rate);
}

void gyro_sample_angle(struct gyro_filter *it, float in_y, float interval)
{
  float angle_err;
  float in_angle=in_y*2.0;

  if (!it->angle_inited) {
    it->angle = in_angle;
    it->angle_inited=1;
  }
  
  angle_err = in_angle - it->angle;

  it->angle += angle_err * interval / (2.0 + it->angle_noise);
  
  uart_tx_uint8('a');
  uart_tx_float(in_y);
  uart_tx_float(in_angle);
  uart_tx_float(it->rate_bias);
}

struct gyro_filter pitch_filter;

#define OCR1_MAX 1023

void set_motor(float level)
{
  level=flim(level, -0.9, 0.9);
  int16_t leveli = (int16_t)(level*(OCR1_MAX+0.9));

  if (leveli<-(OCR1_MAX-2)) leveli=-(OCR1_MAX-2);
  if (leveli>OCR1_MAX-2) leveli=OCR1_MAX-2;

  uart_tx_uint8('m');
  uart_tx_float(level);
  uart_tx_int16(leveli);

  cli();
  PORTC |= 0x0c;     // AHI=1, BHI=1
  PORTC &= ~0x02;    // Disable=0
  if (leveli<0) {
    OCR1A = -leveli;   // ALI is PWM
    OCR1B = 0;         // BLI = 0
  }
  else {
    OCR1A = 0;         // ALI = 0
    OCR1B = leveli;    // BLI is PWM
  }
  sei();
}

void set_motor_idle()
{
  OCR1A=0;
  OCR1B=0;
  PORTC &= ~0x0c; // AHI=0, BHI=0
  PORTC |= 0x02;  // disable=1
}

static uint16_t tcnt0_ext;

static uint16_t last_balance_tcnt0_ext;

static uint8_t tipped;
static uint8_t erect=1;
static float lpf_angle;
static float lpf_angrate;
static float lpf_angintegral;
static float softstart;
static float overspeed;
static float overspeed_integral;
static float balance_torque;
static float cur_speed_est;
//static float wobble_time;

static float p_gain = 5.0/0.5;
static float d_gain = 0.4/0.5;
static float i_gain = 3.0/1.0;
static float hard_speed_lim=0.8;
//static float speed_lim=0.3;

// This is the thing I'd most like to adjust with a pot 
static float pitch_bias=-0.10;

static float sample_conv = 1.0/512.0/(float)ADC_HIST;

// speed: 1.0 is 15 mph
//        0.4 is 6 mph
//        0.5 is 7.5 mph

void balance_init(void)
{
  last_balance_tcnt0_ext=tcnt0_ext;
  balance_torque=0.0;

  DDRB|=0x60;
  DDRC|=0x0e;
}

static uint8_t last_balance_s0;

void balance(void)
{
  uint16_t samples[8];
  if (!adc_collect_samples(samples, &last_balance_s0)) return;
  PORTB |= 0x01;

  float interval;
  {
    uint16_t ticks = tcnt0_ext - last_balance_tcnt0_ext;
    last_balance_tcnt0_ext=tcnt0_ext;
    interval = timer0_seconds_conv * (float)ticks;
  }

  float batt_voltage = samples[0]*sample_conv * 0.5;
  float pitch_rate =   samples[1]*sample_conv - 1.0;
  float ay =           samples[2]*sample_conv - 1.0;
  //float pitchadj =     samples[3]*sample_conv - 1.0;

  uart_tx_uint8('c');
  uart_tx_float(batt_voltage);
  uart_tx_float(pitch_rate);
  uart_tx_float(ay);

  gyro_sample_rate(&pitch_filter, pitch_rate*4.0, interval);
  gyro_sample_angle(&pitch_filter, -(ay-0.15)-0.3*balance_torque, interval);

  lpf_update(&lpf_angle, 0.05, interval, pitch_bias - pitch_filter.angle);
  lpf_update(&lpf_angrate, 0.04, interval, 0.0 - pitch_filter.rate);

  overspeed=0.0;

#if 0
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
  overspeed_integral=flim(overspeed_integral, -0.5, 0.5);
#endif
  
  float lpf_angle_corrected = lpf_angle + flim(0.4*overspeed + overspeed_integral, -0.4, 0.4);

#if 0
  if (softstart==1.0) {
    lpf_angintegral = flim(lpf_angintegral + lpf_angle_corrected*interval, -0.08, 0.08);
  }
#endif

  if (!tipped && (lpf_angle > 0.4 || lpf_angle < -0.4)) tipped=1;
  

#if defined(WOBBLE_TEST)
  wobble_time += interval;
  float wobble=sin(wobble_time)*0.5;
  set_motor(wobble);
  return;
#endif

  if (tipped) {
    // too tippy to try to recover.
    set_motor_idle();
    return;
  }

  //gainmul = exp(1.5*gain_knob);
  //gainmul=0.9;
  
  balance_torque = (p_gain*lpf_angle_corrected + d_gain*lpf_angrate + i_gain*lpf_angintegral) * softstart;

  pitch_filter.angle_noise = fmin(pitch_filter.angle_noise, fmax(balance_torque, -balance_torque)*30.0);
  lpf_update(&pitch_filter.angle_noise, 0.5, interval, 0.0);
  
  float cmd = (cur_speed_est + 0.4*balance_torque) / fmax(0.5, batt_voltage*2.0);
  if (erect) {
    cur_speed_est = flim(cur_speed_est + 0.5*interval*balance_torque,
                         -0.9,0.9);
  }

  cmd=flim(cmd, -hard_speed_lim, hard_speed_lim);

  if (pitch_filter.angle_inited && pitch_filter.rate_inited) {
    softstart = fmin(1.0, softstart + 0.3*interval);
  }

#if 0
  if (tcnt0_ext&0x1000) {
    set_motor_idle();
  } else {
    set_motor(cmd);
  }
#else
  if (!erect && softstart>0.5 && cmd>-0.1 && cmd<0.1) erect=1;
  if (erect) {
    set_motor(-cmd);
  } else {
    set_motor(0.0);
  }
#endif

  uart_tx_uint8('b');
  uart_tx_float(interval);
  uart_tx_float(lpf_angle);
  uart_tx_float(lpf_angrate);
  uart_tx_float(lpf_angintegral);
  uart_tx_float(softstart);
  uart_tx_float(cmd);
  uart_tx_float(balance_torque);
  uart_tx_float(cur_speed_est);

}

void stabilize_analog()
{
  uint16_t samples[8];
  uint8_t i;
  uint8_t last_s0=0;

  for (i=0; i<20+ADC_HIST; ) {
    if (adc_collect_samples(samples, &last_s0)) i++;
  }
}    

void
timer_init(void)
{

  TCCR0 = 0 |
    (1<<CS02) | (1<<CS01) | (1<<CS00); // prescale clk/1024
  timer0_seconds_conv = 1.0/((float)CLOCK_SPEED/1024);

  // PWM mode is "PWM, Phase Correct, 10-bit"
  TCCR1A = 0 |
    (1<<COM1A1) | (1<<COM1A0) | // set on match up, clear on match down
    (1<<COM1B1) | (1<<COM1B0) | // set on match up, clear on match down
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
    16 MHz / 1 / 1024 / 2 gives 8 kHz, probably about right
  */
}

void blinkled()
{
  uint16_t x;
  static uint16_t k;

  DDRB |= 0x01;
  PORTB |= 0x01;
  for (x=0; x<30000; x++) {
    k++;
  }
  PORTB &= ~0x01;
  for (x=0; x<30000; x++) {
    k++;
  }
}

int main( void )
{
  //blinkled();

  timer_init();
  uart_init_tx();

  sei();

  adc_init();
  stabilize_analog();

  uart_tx_str_P(PSTR("!unicycle 1.4\n"));
#ifdef __BDMICRO__
  uart_tx_str_P(PSTR("!BDMICRO version\n"));
#elif defined(__STK_PROTO__)
  uart_tx_str_P(PSTR("!STK_PROTO version\n"));
#elif defined(__ROTOMOTION__)
  uart_tx_str_P(PSTR("!ROTOMOTION version\n"));
#endif

  tcnt0_ext=TCNT0;
  balance_init();
  while (1) {
    extend_count(TCNT0, &tcnt0_ext);
    
    if (uart_tx_qlen()>10) continue;

    uart_tx_uint8('&');
    uart_tx_uint8(TCNT0);

    dump_adc_log();

    if (0) {
      float debugangle=tcnt0_ext*(2.0*3.1415926/65536.0);
      set_motor(1.0*sin(debugangle));
      //set_motor(-0.9);
      if (tcnt0_ext&0x1000) {
        set_motor(+0.5);
      } else {
        set_motor(-0.5);
      }
    } else {
      balance();
    }
    PORTB &= ~0x01;
    uart_tx_uint8(';');
  }
}
