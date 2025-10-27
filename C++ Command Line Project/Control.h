#ifndef CONTROL_H
#define CONTROL_H

#include <iostream>
#include <cmath> // Required for abs
#include "PID.h"


bool check5(double value1, double value2);

void set_speed(double &setpoint);

void run_pid(PID &pid, double setpoint, double max_out, double min_out, double mass, double friction, int max_it, bool &success);
   

void tune(PID &pid, double &setpoint, double max_out, double min_out, double mass, double friction, int max_it);

void change_set(int &max_it, double &mass, double &friction, double &max_out, double &min_out);

void pid_menu();

#endif
