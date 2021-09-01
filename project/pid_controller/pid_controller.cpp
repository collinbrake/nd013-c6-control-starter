/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <vector>
#include <iostream>
#include <math.h>

using namespace std;

PID::PID() {}

PID::~PID() {
	
}

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini) {
  Kp = Kpi;
  Ki = Kii;
  Kd = Kdi;
  output_lim_max = output_lim_maxi;
  output_lim_min = output_lim_mini;
}


void PID::UpdateError(double new_error) {
  last_error = error;
  error = new_error;
  accum_error += error;
}

double PID::TotalError() {
  double control;
  double error_slope = (error - last_error)/delta_time;
  control = Kp*error + Ki*accum_error + Kd*error_slope; 
  return control;
}

// UpdateDeltaTime updates the time difference between current loop iteration and last
double PID::UpdateDeltaTime(double new_delta_time) {
   delta_time = new_delta_time;
}