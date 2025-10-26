#ifndef GLOBALS_H
#define GLOBALS_H

#include "Joystick.h"
#include "mbed.h"

extern Joystick joystick;  //attach and create joystick object
extern InterruptIn joystick_button, special_ability_button, pause_button;
extern Timeout fire_rate, power_up, special_ability, round_end, survival;
extern BufferedSerial pc;
extern DigitalOut power_up_led, ability_led;
extern PwmOut buzzer;

extern float y_pos;
extern float x_pos;
extern float wind_x;
extern float wind_y;
extern volatile int g_buttonA_flag;
extern volatile int g_special_flag;
extern volatile int g_fire_timer;
extern volatile int reset_plane_to_default;
extern volatile int g_special_ability_timer;
extern volatile int g_pause_flag;
extern volatile int g_round_end;
extern volatile int g_survival;

#endif