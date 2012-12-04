/*
  COPYRIGHT
*/
#ifndef _SCOOTER_BAL_BALCONF_H
#define _SCOOTER_BAL_BALCONF_H

#if defined(__SEGWELL2__)   // the 2-wheeled scooter with handlebar steering

  #define __BAL2__
  #define DO_STEERING
//#define DO_BAT

#elif defined(__JORDANWAY__)   // the 2-wheeled scooter with handlebar steering, Jordan's version

  #define __BAL3__
  #define DO_STEERING
//#define DO_BAT

#elif defined(__UNICYCLE2__)

  // BAL4 is scooter/electronics/unicycle1
  #define __BAL4__

#elif defined(__ZOOMBOARD__) // the hub motor board

  #define __BAL3__
  //#define DO_ENCODERS
  //#define DO_BAT

#else

  #error "No hardware defined"

#endif

//#define DO_DRIVE
//#define CATCH_GLITCHES
//#define DO_MISSION_LOG

// This also gets read by balctl.py

struct gyro_filter {
  float angle;
  float rate_bias;
  float rate;
  float angle_noise;

  uint8_t rate_inited;
  uint8_t angle_inited;
  uint8_t dummy1;        // avoid alignment issues by padding to 4 bytes
  uint8_t dummy2;
};

/*
  Most parameters that affect how it feels should be here, though
  there are still some things in the code
 */
struct bal_config {
  
  float p_gain; // proportional gain, torque = p_gain * tilt angle
  float d_gain; // derivative gain, torque = d_gain * d(tilt_angle)/dt
  float i_gain; // integral gain
#ifdef DO_STEERING
  float yaw_steer_gain;  // steer_torque = yaw_steer_gain * yaw_rate
  float yaw_steer_speed_thresh; // speed below which we cut yaw gain
#endif

  float hard_speed_lim; // never give a higher PWM value than this: only for testing
  float fwd_speed_lim;  // start limiting speed by tilting back when we hit this speed
  float rev_speed_lim;  // ditto for reverse
  float beep_speed_lim; // start beeping when we exceed this speed.
  float rev_stick_shake_speed_lim; // do stick shake when going faster than this in reverse
  
#ifdef DO_STEERING
  float steer_rate_num; // these define how maximum steering rate varies with speed
  float steer_rate_den;
#endif

#ifdef DO_STEERING
  float steering_bias;     // adjusts for centering error in the steering controls
  float yaw_target_mult;   // ratio between expected yaw_rate and desired_steer
#endif

  float balance_torque_lim;  // don't let abs(balance_torque) get higher than this to limit motor current
  float motor_torque_factor; // depends on the resistance of your motor

  float fuse_alarm_thresh;

  float fwd_accel_coupling; // subtract balance_torque*fwd_accel_coupling from fwd_accel

  float crossover_boost;// gives a little extra gain around zero speed, to overcome motor stiction
#ifdef __BAL2__
  float dummy;
#endif
};

struct bal_hwadj {
#ifdef DO_STEERING
  float steering_bias;     // adjusts for centering error in the steering controls
  float yaw_bias;       // adjusts for offset error in yaw gyro
#endif
  
  float fwd_accel_bias; // adjust for offset error in the accelerometer
  float right_accel_bias;
  float bat_voltage_mult;
};

/*
  Command for driving it over the serial link
 */
struct bal_drive {
  float speed_targ;
  float steering_targ;
};

/*
  Online estimator of battery charge state. It's a cookie-cutter least-squares linear fit
  between battery voltage and current.
 */
struct bat_state {

  // Accumulator variables
  float s1;
  float si;
  float sii;
  float sv;
  float svv;
  float siv;

  // estimated variables
  float delta;
  float resistance;
  float ocvoltage;
  float sigma_resistance;
  float sigma_ocvoltage;
  float fullloadvoltage;

};

/*
  Some of these things are really state, updated from cycle to cycle
  (like lpf_angle) and some things are always calculated from scratch
  each time (like batt_voltage, read from an ADC.) We bundle them all
  into this big structure so we can send it to the balctl program to
  make graphs.
*/
struct bal_state {

  float realtime;               // time since power up (sec)
  float interval;               // interval since last iteration (sec)

  float bat_voltage;           // battery voltage (0=0v, 1=50v)
  float pitch_rate;             // forward pitch rate from gyro, (radians/sec)
  float fwd_accel;              // foward acceleration (Gs)
  float fwd_accel_net;          // without estimated accel due to forward motion
#ifdef DO_STEERING
  float steering;               // steering (-1 .. +1)
  float yaw_rate;               // yaw rate from gyro (radians/sec)
#endif
#if defined(__BAL3__)
  float roll_rate;
  float right_accel;
#endif
#if defined(__BAL2__) || defined(__BAL3__) || defined(__BAL4__)
  float knob1;                  // A general purpose adjusting knob mounted on the console
  float knob2;
  float knob3;
#endif
#if defined(__BAL2__) || defined(__BAL3__) || defined(__BAL4__)
  float lmis;                   // left motor current sensor, (0=0A, 1==100A, -1=-100A)
#ifdef DO_STEERING
  float rmis;
#endif
  float batis;
#endif
#if defined(__BAL2__)
  float b1is;                   // battery 1 current sensor
  float b2is;
#endif

  float bt_lim;

  float full_speed_time;        // count up time it's at full speed so we can give up

  float lpf_angle;              // filtered version of the tilt angle
  float lpf_angle_corrected;    // the above, after adding in intentional tilt (for limiting speed)
  float lpf_angrate;            // filtered version of pitch rate
  float lpf_angintegral;        // integral
#ifdef DO_STEERING
  float lpf_steering;           // filtered version of steering
  float lpf_yaw_rate;           // filtered version of yaw rate
  float target_yaw;             // expected yaw rate (radians/sec) based on steering input
#endif

  float lpf_knob1;

  float softstart;              // ramps up 0..1 in first second. We use it to avoid a lurch at startup
  float start_protection;       // ramps up 0..1 in first few seconds. We use it to avoid a lurch at startup
  float overspeed;              // Will be >0 for fwd overspeed, <0 for negative overspeed
  float overspeed_integral;     // Will be >0 for persistent fwd overspeed

  float balance_torque;         // torque calculated by the feedback loop
  float cur_speed_est;          // speed, -1..1 (corresponding to PWM value of motors)
  float cur_cg_speed_est;       // above, adjusted for the speed of the center of gravity by adding in pitch rate
  float stick_shake;            // Amount of stick shake. Set nonzero to annoy the rider
  float gain_reduction;         // Normally 1, set below 1 to soften response. We do this in drive mode

  float cmd;                    // The average PWM value of the drive motor(s)
#ifdef DO_STEERING
  float steer_rate;             // A function of speed, sets the maximum steering rate
  float steer_diff;             // The actual steering command, difference between left and right
#endif
  float left_pwm;               // actual PWM values, cmd + differential steering
#ifdef DO_STEERING
  float right_pwm;
#endif

#ifdef __BAL3__
  float pwmtempa;
  float pwmtempb;
  float pwmtempc;
  float pwmtempd;
#endif

  struct gyro_filter pitch_filter;  // The pitch angle estimator

#ifdef DO_DRIVE
  struct bal_drive drive;       // The most recent drive command
  float drive_age;              // Time since we received the above, so we know when to stop driving
  float drive_steer;            // The filtered version of drive, active when age is recent
  float drive_tilt;
#endif

#ifdef DO_BAT
  struct bat_state bat;         // Battery state estimator
  float bat_speed_lim;          // Estimate of the maximum speed we have enough battery power for
#endif

#ifdef DO_ENCODERS
  uint32_t enc1_pos;
  uint32_t enc2_pos;
  float enc1_vel;
  float enc2_vel;
#endif

  uint32_t dummy;

  uint8_t mode;
  uint8_t mode_reason;
};

#ifdef DO_MISSION_LOG
struct bal_mission_log_ent {
  float realtime;
  float bat_voltage;
  float bat_current;
  float bat_resistance;
  float bat_ocvoltage;
  float cur_speed_est;
};

#define N_MISSION_ENTRIES (4096/sizeof(bal_mission_log_ent))

#endif

#endif
