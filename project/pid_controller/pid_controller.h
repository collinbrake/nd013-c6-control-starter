/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PID {
public:

  	/*
    * Errors
    */
    double error; // error for this loop iteration
  	double last_error; // error for last loop iteration
  	double accum_error; // accumulated error for integral calculation

  	/*
    * Coefficients
    */
    double Kp, Ki, Kd; // proportional, integral, and derivative coefficients

    /*
    * Output limits
    */
    double output_lim_max;
  	double output_lim_min;
  	
  	/*
    * Time delta
    */
    double delta_time; // time difference between current loop iteration and last (for derivative calculation)

    /*
    * Constructor
    */
    PID();

    /*
    * Destructor.
    */
    virtual ~PID();

    /*
    * Initialize PID.
    */
    void Init(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini);

    /*
    * Update the PID error variables given cross track error.
    */
    void UpdateError(double new_error);

    /*
    * Calculate the total PID error.
    */
    double TotalError();
  
    /*
    * Update the delta time.
    */
    double UpdateDeltaTime(double new_delta_time);
};

#endif //PID_CONTROLLER_H


