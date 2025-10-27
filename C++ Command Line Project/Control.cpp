#include <iostream>
#include <cmath> // Required for abs
#include "Colours.h"
#include "Control.h"
using namespace std;

bool check5(double value1, double value2) {
    const double tolerance = 1e-4; 
    return abs(value1 - value2) < tolerance;
}

void set_speed(double &setpoint) {
    cout << Colour::CLEAR;
    cout << Colour::CYAN << "Enter the desired speed (m/s): " << Colour::RESET;
    cin >> setpoint;
    setpoint = abs(setpoint);
    cout << Colour::GREEN << "Setpoint updated to " << setpoint << " m/s.\n" << Colour::RESET;
}

void run_pid(PID &pid, double setpoint, double max_out, double min_out, double mass, double friction, int max_it, bool &success) {
    double speed = 0.0; 
    double output_force;
    success = false; // Default to false until target is reached
    double drag;

    for (int i = 0; i < max_it; ++i) {
        output_force = pid.compute(setpoint, speed);
        if (output_force > max_out) {
            output_force = max_out;
        } else if (output_force < min_out) {
            output_force = min_out;
        }
        drag = speed * speed * 0.1 + 0.05 * speed;
        if (drag > friction * (mass * 9.8)) drag = friction * (mass * 9.8);
        double acceleration = (output_force - drag) / mass; 
        speed += acceleration * pid.dtime; 
        cout << Colour::MAGENTA << "Step " << i + 1 << ": Speed = " << speed << " m/s, Control Signal = " << output_force << "\n" << Colour::RESET;

        if (check5(speed, setpoint)) {
            cout << Colour::GREEN << "Target speed reached after " << max_it*pid.dtime << " seconds.\n" << Colour::RESET;
            success = true;
            break;
        }
    }

    if (!success) {
        cout << Colour::RED << "Warning: Target speed not reached after " << max_it*pid.dtime << " seconds.\n" << Colour::RESET;
    }
}

void tune(PID &pid, double &setpoint, double max_out, double min_out, double mass, double friction, int max_it) {
    int choice;
    do {
        cout << Colour::YELLOW << "\n--- Tune PID ---\n" << Colour::RESET;

        // tuning guidance
        cout << Colour::MAGENTA << "\nTypical steps for manually tuning a PID controller:\n";
        cout << "1. Determine what characteristics of the system need improvement.\n";
        cout << "2. Use KP to decrease the rise time. (" << pid.kp << ")\n";
        cout << "3. Use KI to eliminate the steady-state error.(" << pid.ki << ")\n";
        cout << "4. Use KD to reduce the overshoot and settling time.(" << pid.kd << ")\n" << Colour::RESET;

        // Tuning options
        cout << Colour::BLUE << "\n1. Change Kp\n";
        cout << "2. Change Ki\n";
        cout << "3. Change Kd\n";
        cout << "4. Change Sample Time (dtime)\n";
        cout << "5. Run Simulation\n";
        cout << "6. Back to Main Menu\n";
        cout << "Enter your choice: " << Colour::RESET;
        cin >> choice;

        switch (choice) {
        case 1:
            cout << Colour::MAGENTA << "Current value of Kp is " << pid.kp << "\n"<< Colour::RESET;
            cout << Colour::CYAN << "Enter new value for Kp: " << Colour::RESET;
            cin >> pid.kp;
            break;
        case 2:
            cout << Colour::MAGENTA << "Current value of Ki is " << pid.ki << "\n"<< Colour::RESET;
            cout << Colour::CYAN << "Enter new value for Ki: " << Colour::RESET;
            cin >> pid.ki;
            break;
        case 3:
            cout << Colour::MAGENTA << "Current value of Kd is " << pid.kd << "\n"<< Colour::RESET;
            cout << Colour::CYAN << "Enter new value for Kd: " << Colour::RESET;
            cin >> pid.kd;
            break;
        case 4:
            cout << Colour::MAGENTA << "Current sample time is " << pid.dtime << "\n"<< Colour::RESET;
            cout << Colour::CYAN << "Enter new value for Sample Time (dtime): " << Colour::RESET;
            cin >> pid.dtime;
            break;
        case 5: {
            bool success;
            run_pid(pid, setpoint, max_out, min_out, mass, friction, max_it, success);
            if (!success) {
                cout << Colour::CYAN << "Returning to tuning menu for adjustments...\n" << Colour::RESET;
            }
            break;
        }
        case 6:
            cout << Colour::GREEN << "Returning to Main Menu...\n" << Colour::RESET;
            break;
        default:
            cout << Colour::RED << "Invalid choice. Please try again.\n" << Colour::RESET;
            break;
        }
    } while (choice != 6);
}

void change_set(int &max_it, double &mass, double &friction, double &max_out, double &min_out) {
    int choice;
    do {
        cout << Colour::YELLOW << "\n--- Change Settings ---\n" << Colour::RESET;
        cout << Colour::BLUE << "1. Change Maximum Steps\n";
        cout << "2. Change Mass (kg)\n";
        cout << "3. Change Friction Coefficient\n";
        cout << "4. Change Maximum Output\n";
        cout << "5. Change Minimum Output\n";
        cout << "6. Back to Main Menu\n";
        cout << "Enter your choice: " << Colour::RESET;
        cin >> choice;

        switch (choice) {
        case 1:
            cout << Colour::CYAN << "Enter new maximum steps: " << Colour::RESET;
            cin >> max_it;
            break;
        case 2:
            cout << Colour::CYAN << "Enter new mass (kg): " << Colour::RESET;
            cin >> mass;
            break;
        case 3:
            cout << Colour::CYAN << "Enter new friction coefficient: " << Colour::RESET;
            cin >> friction;
            break;
        case 4:
            cout << Colour::CYAN << "Enter new maximum output: " << Colour::RESET;
            cin >> max_out;
            break;
        case 5:
            cout << Colour::CYAN << "Enter new minimum output: " << Colour::RESET;
            cin >> min_out;
            break;
        case 6:
            cout << Colour::GREEN << "Returning to Main Menu...\n" << Colour::RESET;
            break;
        default:
            cout << Colour::RED << "Invalid choice. Please try again.\n" << Colour::RESET;
            break;
        }
    } while (choice != 6);
}

void pid_menu() {
    PID pid;                 
    pid.kp = 0.5;            
    pid.ki = 0.1;             
    pid.kd = 0.05;            
    pid.dtime = 0.1;         
    double setpoint = 0.0;    
    double max_out = 100.0; 
    double min_out = 0.0;   
    double mass = 80.0;        
    double friction = 0.02;    
    int max_it = 1000; 
    bool success;

    int choice;

    do {
        cout << "\n-----------------" << Colour::RGB_colour(252, 133, 0, true) << " PID " << Colour::RESET << "-----------------\n";
        cout << "|    " << Colour::RGB_colour(252, 133, 0, true) << "1. Set Desired Speed" << Colour::RESET << "            |\n";
        cout << "|    " << Colour::RGB_colour(252, 133, 0, true) << "2. Tune PID" << Colour::RESET << "                     |\n";
        cout << "|    " << Colour::RGB_colour(252, 133, 0, true) << "3. Run PID" << Colour::RESET << "                      |\n";
        cout << "|    " << Colour::RGB_colour(252, 133, 0, true) << "4. View Current Settings" << Colour::RESET << "        |\n";
        cout << "|    " << Colour::RGB_colour(252, 133, 0, true) << "5. Change Simulation Settings" << Colour::RESET << "   |\n";
        cout << "|    " << Colour::RGB_colour(220, 20, 60, true) << "6. Exit" << Colour::RESET << "                         |\n";
        cout << "---------------------------------------\n";
        cout << "Enter your choice: " << Colour::RESET;
        cin >> choice;

        switch (choice) {
        case 1:
            set_speed(setpoint);
            break;
        case 2:
            cout << Colour::CLEAR;
            tune(pid, setpoint, max_out, min_out, mass, friction, max_it);
            break;
        case 3:
            run_pid(pid, setpoint, max_out, min_out, mass, friction, max_it, success);
            if (!success) {
                cout << Colour::CYAN << "Consider tuning the PID parameters.\n" << Colour::RESET;
            }
            break;
        case 4: 
            cout << Colour::CLEAR;
            cout << Colour::CYAN << "Kp = "<< pid.kp <<"\n" ;
            cout << "Ki = "<< pid.ki <<"\n" ;
            cout << "Kd = "<< pid.kd <<"\n" ;
            cout << "Sample Time = "<< pid.dtime << "\n";
            cout << "Setpoint = "<< setpoint <<"\n";
            cout << "Maximum Output = "<< max_out <<"\n";
            cout << "Minimum Output = "<< min_out <<"\n";
            cout << "Mass = "<< mass <<"\n";
            cout << "Friction coefficient = "<< friction <<"\n";
            cout << "Steps = "<< max_it <<"\n"<< Colour::RESET;
            break;
            
            
        case 5:
            change_set(max_it, mass, friction, max_out, min_out);
            pid.max_output = max_out;
            pid.min_output = min_out;
            break;
        case 6:
            cout << Colour::GREEN << "Exiting the program.\n" << Colour::RESET;
            break;
        default:
            cout << Colour::RED << "Invalid choice. Please try again.\n" << Colour::RESET;
            break;
        }
    } while (choice != 6);
}