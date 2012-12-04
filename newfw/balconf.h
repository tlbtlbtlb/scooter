#ifndef _SCOOTER_BAL_BALCONF_H
#define _SCOOTER_BAL_BALCONF_H

#include "dsp.h"

struct filter2_coeff {
  dsp_t bcoeff[3];
  dsp_t acoeff[2];
};

struct filter2_state {
  dsp_t x[3];
  dsp_t y[3];
};

struct poly3 {
  dsp_t c0;
  dsp_t c1;
  dsp_t c2;
  dsp_t c3;
};

/*
  Most parameters that affect how it feels should be here, though
  there are still some things in the code
 */
struct bal_config {

  dsp_t vp_height;

  dsp_t vel_err_lim;
  dsp_t vel_err_i_lim;
  dsp_t vel_err_i_deadband;

  dsp_t yaw_rate_err_lim;
  dsp_t yaw_rate_err_i_lim;

  dsp_t neutral_pitch;
  dsp_t target_pitch_lim;
  dsp_t pitch_err_i_lim;

  dsp_t vel_coeff;
  dsp_t vel_i_coeff;
  dsp_t yaw_rate_coeff;
  dsp_t yaw_rate_i_coeff;

  dsp_t pitch_coeff;
  dsp_t pitch_rate_coeff;
  dsp_t pitch_i_coeff;
  dsp_t current_accel_coeff;
  dsp_t current_vel_coeff;

  dsp_t yaw_steering_coeff;
  dsp_t steering_corr;

  dsp_t hard_fwd_lim;
  dsp_t hard_yaw_lim;

  dsp_t fwd_vel_lim;
  dsp_t rev_vel_lim;

  struct filter2_coeff fwd_cmd_filter;
  struct filter2_coeff yaw_cmd_filter;
  struct filter2_coeff target_pitch_filter;

  // polynomial coefficients of temperature correction curve
  struct poly3 pitch_temp_corr;
  struct poly3 roll_temp_corr;
  struct poly3 yaw_temp_corr;

  int16_t deadband;
  int16_t hysteresis;
};

struct func_cache {
  dsp_t input;
  dsp_t output;
  uint16_t misses;
  uint16_t dummy1;
};

/*
  Some of these things are really state, updated from cycle to cycle
  (like lpf_angle) and some things are always calculated from scratch
  each time (like batt_voltage, read from an ADC.) We bundle them all
  into this big structure so we can send it to the balctl program to
  make graphs.

  Making this packed ensures compatibility between i386-FreeBSD and avr versions.
*/
struct bal_state {

  dsp_t realtime;               // time since power up (sec)
  dsp_t interval;               // interval since last iteration (sec)
  int32_t inverse_interval;     // whole number intervals/moment

  dsp_t speed_est;
  dsp_t vp_vel;
  dsp_t vp_vel_abs;
  dsp_t vel_err;
  dsp_t yaw_rate_err;
  dsp_t pitch_rate_err;
  dsp_t pitch_err_i;

  dsp_t bat_voltage;            // battery voltage (0=0v, 1=50v)
  dsp_t pitch_gyro;             // forward pitch rate from gyro, (radians/sec)
  dsp_t pitch_temp;
  dsp_t pitch_gyro_temp_corr;   // temperature correction to above
  dsp_t pitch_ltr_corr;
  dsp_t yaw_gyro;             // yaw rate from gyro, (radians/sec)
  dsp_t yaw_temp;
  dsp_t yaw_gyro_temp_corr;   // temperature correction to above
  dsp_t yaw_ltr_corr;
  dsp_t steering_sensor;

  dsp_t roll_gyro;             // roll rate from gyro, (radians/sec)
  dsp_t roll_temp;
  dsp_t roll_gyro_temp_corr;   // temperature correction to above
  dsp_t roll_ltr_corr;

  dsp_t x_accel, y_accel, z_accel, est_fwd_accel;

  dsp_t lmis;                   // left motor current sensor, (0=0A, 1==100A, -1=-100A)
  dsp_t rmis;
  dsp_t batis;

  dsp_t pitch_rate;
  dsp_t pitch;

  dsp_t roll_rate;
  dsp_t roll;

  dsp_t yaw_rate;

  dsp_t effective_pitch;
  dsp_t target_pitch;
  dsp_t target_pitch_rate;
  dsp_t pitch_err;

  dsp_t softstart;              // ramps up 0..1 in first second. We use it to avoid a lurch at startup

  dsp_t fwd_cmd;                // The average voltage of the drive motor(s)
  dsp_t yaw_cmd;                // The differential voltage of the drive motor(s)
  dsp_t l_pwm;                  // actual PWM values, (cmd + differential steering)/bat voltage
  dsp_t r_pwm;

  dsp_t target_age;
  dsp_t target_vel;
  dsp_t target_yaw_rate;
  dsp_t external_pitch_adj;
  dsp_t neutral_pitch;
  dsp_t neutral_pitch_rate;

  dsp_t target_hold_pos;
  dsp_t target_hold_yaw;

  int16_t limit_cmd_error, limit_bat_voltage_error, limit_pitch_error, limit_wheelie_error, limit_current_error;

  int16_t kill_yaw_integral;
  int16_t kill_vel_integral;
  
  uint16_t tickmask;

  uint8_t mode;
  uint8_t at_limit;
  uint8_t pitch_inited;
  uint8_t thermal_mode;

} __attribute__((packed));


struct segbase_linstats1 {
  dsp_t sy;
  dsp_t syy;
  uint16_t s1;
};

struct bal_internal {
  struct segbase_linstats1 pitch_mon;
  struct segbase_linstats1 temp_mon;

  dsp_t lmis_bias, rmis_bias, batis_bias;

  struct func_cache batfac_cache;
};

struct bal_filters {
  struct filter2_state fwd_cmd_filter;
  struct filter2_state yaw_cmd_filter;
  struct filter2_state target_pitch_filter;
  struct filter2_state target_vel_filter;
  struct filter2_state target_yaw_rate_filter;
};


#endif
