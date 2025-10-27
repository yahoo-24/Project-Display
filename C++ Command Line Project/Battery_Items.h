#ifndef BATTERY_ITEMS_H
#define BATTERY_ITEMS_H

#include <string>
#include <vector>
#include "Robot.h"
using namespace std;

bool is_float( string myString );

// function to check the validity of the battery value
int check_robot_battery(); 

// function to check the validity of the reduction amount
int check_battery_redcution(); 

// function to show and draw current battery's value 
int display_battery(int battery);

// function to generate the vector corresponding to a given battery value
vector<string> generate_battery_vector(int battery);

// function to show the charging process of the battery
int charge_battery(int battery);

// function to show the discharging process of the battery, with a specific amount of reduction
int discharge_battery(int battery, int reduction);

// function to turn all the character of a string into lower case
string lowercase(string str);

// function to transform a float in a 2 decimal point string float value
string precision(float flt);

// function to append a new item into the warehouse
void append_item(string store_item_file, string item_name, float item_price, int item_quantity);

// function to update the quantity of an existing item
void update_quantity(string store_item_file, string item_name, int new_item_quantity);

// function to update the price of an existing item
void update_price(string store_item_file, string item_name, float new_item_price);

// function to update the quantities of warehouse items after fullfilling an order
void auto_update_quantities(string store_item_file, string order_file);

// function that gets the total of the order fullfilled
void get_total(string store_item_file, string order_file);

// function that deletes an existing item from the warehouse
void delete_item(string store_item_file, string item_name);

// function to display some animation when fullfilling an order
void pick_up_order(string store_item_file, string order_file, int current_robot_battery);

// function to check the validity of the quantity passed 
int check_quantity();

// function to check the validity of the price value passed
float check_price();

#endif