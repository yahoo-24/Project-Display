#include "Globals.h"

Joystick joystick(PC_3, PC_2);  //attach and create joystick object
InterruptIn joystick_button(PC_10), special_ability_button(PC_4), pause_button(PB_0);
Timeout fire_rate, power_up, special_ability, round_end, survival;
BufferedSerial pc(USBTX, USBRX, 115200);
DigitalOut power_up_led(PH_1), ability_led(PH_0);
PwmOut buzzer(PA_15);

float y_pos = 35; // User position
float x_pos = 42;
float wind_x = 0; // Wind directions
float wind_y = 0;
volatile int g_buttonA_flag = 0; // Flag used to indicate weapon fired
volatile int g_special_flag = 0; // Flag used to indicate special ability is used
volatile int g_fire_timer = 0; // Flag used to indicate that the delay before firing again is done
volatile int reset_plane_to_default = 0; // Flag used to indicate end of a power-up
volatile int g_special_ability_timer = 1; // Flag used to indicate the end of the 20s delay before using the ability again
volatile int g_pause_flag = 0; // Flag used to indicate that the user clicked pause
volatile int g_round_end; // 5s Delay caused by this flag after a round ends
volatile int g_survival = 0; // Raised when the user survives the allocated time.