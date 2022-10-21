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
  new_delta_time = 0;
}


void PID::UpdateError(double cte) {
   /**
   * TODO: Update PID errors based on cte.
   **/
  
  if (new_delta_time>0)
  {
    DError = (cte-PError)/new_delta_time ;
  }
  IError += cte* new_delta_time;
  PError = cte;
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