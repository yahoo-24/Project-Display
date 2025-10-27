#include "Robot.h"
#include <iostream>
using namespace std;

Robot::Robot(){ // Constructor 
    _battery = 100; // default battery value if nothing passed in the mutator
}

void Robot::set_battery(int battery){ // mutator 
    if(battery < 0){ // if passing a negative value, set the battery to 0%
        cout << "Battery out of range (< 0), value will be set to:" << endl;
        _battery = 0;
    }
    else if(battery > 100){ // if passing a value > 100, set the battery to 100%
        cout << "Battery out of range (> 100) value will be set to:" << endl;
        _battery = 100;
    }
    else{ // else, accept the value passed in the mutator
        _battery = battery;
    }
}

int Robot::get_battery(){ // accessor
    return _battery; // return the value passed in the mutator, or use the default value
}