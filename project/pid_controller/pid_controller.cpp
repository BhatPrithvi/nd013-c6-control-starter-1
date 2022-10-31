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

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini) {
   /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   **/
  this-> Kpi = Kpi;
  this-> Kdi = Kdi;
  this-> Kii = Kii;
  this-> output_lim_maxi = output_lim_maxi;
  this-> output_lim_mini = output_lim_mini;
  PError = 0;
  IError = 0;
  DError = 0;
  //this-> new_delta_time = 0;
  preverr = 0;
}


void PID::UpdateError(double cte) {
   /**
   * TODO: Update PID errors based on cte.
   **/
  IError += cte* this ->new_delta_time;
  PError = cte;
  
  /*The next few lines of code include the integral windup for tuning PID gains. This is done by preventing the integral term from accumulating above or below pre-determined bounds as mentioned in https://en.wikipedia.org/wiki/Integral_windup#cite_note-4.
  The codes are based on http://brettbeauregard.com/blog/2011/04/improving-the-beginner%e2%80%99s-pid-reset-windup/
  and https://github.com/BanaanKiamanesh/Anti-Windup-PID-Controller/blob/main/PID.cpp
  */
  if (output_lim_maxi > PError)
      lim_max_integ = output_lim_maxi - PError;  // Limit maximum integral bounds
  else
      lim_max_integ = 0.0;

  if (output_lim_mini < PError)
     lim_max_integ = output_lim_mini - PError; // Limit minimium integral bounds
  else
        lim_min_integ = 0.0;

    // Limit Integrator gain
     if (IError > lim_max_integ)
        IError = lim_max_integ;

     else if (IError < lim_min_integ)
         IError = lim_min_integ;
  
  if (this ->new_delta_time>0)
  {
    DError = (cte-preverr)/this->new_delta_time ;
  }
  preverr = cte;
}

double PID::TotalError() {
   /**
   * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
    double control;
    control = Kpi*PError + Kii*IError +Kdi*DError;
    if (control < output_lim_mini)
    {
     control = output_lim_mini;
    }
    else if (control > output_lim_maxi)
    {
      control = output_lim_maxi;
    }
    return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * TODO: Update the delta time with new value
   */  
   this ->new_delta_time = new_delta_time;
   return new_delta_time;
}