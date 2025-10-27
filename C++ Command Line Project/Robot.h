#ifndef ROBOT_H
#define ROBOT_H

class Robot{ // Defining a class called Robot
    public:
        Robot(); // constructor 
        void set_battery(int battery); // mutator (setter) function to set the value passed to the robot battery
        int get_battery(); // accessor (getter) function to get the battery value

    private:
        int _battery; // private battery value
};

#endif