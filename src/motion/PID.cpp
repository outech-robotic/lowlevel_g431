/*
 * PID.cpp
 *
 *  Created on: 8 déc. 2019
 *      Author: ticta
 */

#include "PID.h"

PID::PID() {
  reset();
}


void PID::reset(){
  last_setpoint = 0;
  last_error = 0;
  comp_integral = 0;
}


void PID::set_coefficients(float new_kp, float new_ki, float new_kd, uint32_t new_freq){
  kp = new_kp;
  ki = new_ki / new_freq;
  kd = new_kd * new_freq;
}


void PID::set_kp(float new_kp){
  kp = new_kp;
}


void PID::set_ki(float new_ki, uint32_t new_freq){
  ki = new_ki/new_freq;
}


void PID::set_kd(float new_kd, uint32_t new_freq){
  kd = new_kd*new_freq;
}


void PID::set_output_limit(int32_t new_limit){
  min = -new_limit;
  max = new_limit;
}


void PID::set_anti_windup(int32_t new_limit){
  integral_min = -new_limit;
  integral_max = new_limit;
}


void PID::get_coefficients(float* ret_kp, float* ret_ki, float* ret_kd){
  *ret_kp = kp;
  *ret_ki = ki;
  *ret_kd = kd;
}


int16_t PID::compute(int32_t input, int32_t setpoint){
  int64_t res;

  error = setpoint-input;
  derivative_error = (error-last_error) - (setpoint - last_setpoint);
  last_error = error;
  last_setpoint = setpoint;

  //Proportionnal component of output
  comp_proportional = ((int64_t)kp) * ((int64_t)error);

  //Integral component
  comp_integral += ((int64_t)ki) * ((int64_t)error);
  if(comp_integral > integral_max)
    comp_integral = integral_max;
  else if(comp_integral < integral_min)
    comp_integral = integral_min;

  //Derivative component
  comp_derivative = (int64_t)kd * (int64_t)derivative_error;

  //Complete scaled output
  res = comp_proportional + comp_integral + comp_derivative;

  // Saturation
  if(res > max)
    res = max;
  else if(res<min)
    res = min;

  return res;
}


int32_t PID::get_error(){
  return error;
}


int32_t PID::get_proportional(){
  return comp_proportional;
}


int64_t PID::get_integral(){
  return comp_integral;
}


int32_t PID::get_derivative(){
  return comp_derivative;
}
