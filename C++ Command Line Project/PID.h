#ifndef PID_H
#define PID_H

class PID {
public:
    double kp, ki, kd;
    double integrator, prev_error;
    double dtime;
    double max_output, min_output; // Output limits
    double max_integrator_value, min_integrator_value; // Integrator limits
    PID() 
        : kp(0), ki(0), kd(0), integrator(0), prev_error(0), dtime(0),
          max_output(100), min_output(-100), 
          max_integrator_value(50), min_integrator_value(-50) {}
    double compute(double setpoint, double current_speed) {
        double error = setpoint - current_speed;
        double proportional = kp * error;
        integrator += (error + prev_error) / 2 * dtime;
        if (integrator > max_integrator_value) integrator = max_integrator_value;
        if (integrator < min_integrator_value) integrator = min_integrator_value;
        double integral = ki * integrator;
        double derivative = (dtime != 0) ? kd * (error - prev_error) / dtime : 0.0;
        prev_error = error;
        double output = proportional + integral + derivative;
        if (output > max_output) output = max_output;
        if (output < min_output) output = min_output;

        return output;
    }
};

#endif