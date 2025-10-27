#include <iostream>
#include <string>
#include <regex>
#include <thread>
#include <chrono>
#include <vector>
#include <fstream> // Files I/O 
#include <iomanip>
#include <sstream>
#include <windows.h>

#include "funcs.h"
#include "LoopPath.h"
#include "Control.h"
#include "Battery_Items.h"
#include "util.h"
#include "Colours.h"
using namespace std;

int get_menu2_user_input() { // checking the user input in menu 2 (Robot Battery Management)
  int input;
  string input_string;
  bool valid_input = false;
  int menu_items = 4; // we had 3 options + back to main menu option

  do {
    cout << "\nEnter your choice: ";
    cin >> input_string;
    valid_input = is_integer(input_string);
    // if input is not an integer, print an error message
    if (valid_input == false) {
      cout << "Enter an integer!\n";
    } else {  // if it is an int, check whether in range
      input = stoi(input_string);  // convert to int
      if (input >= 1 && input <= menu_items) {
        valid_input = true;
      } else {
        cout << "Invalid menu item!\n";
        valid_input = false;
      }
    }
  } while (valid_input == false);

  return input;
}

int get_menu3_user_input() { // checking the user input in menu 3 (Orders and Warehouse Management) 
  int input;
  string input_string;
  bool valid_input = false;
  int menu_items = 6; // we had 5 option + back to main menu option

  do {
    cout << "\nEnter your choice: ";
    cin >> input_string;
    valid_input = is_integer(input_string);
    // if input is not an integer, print an error message
    if (valid_input == false) {
      cout << "Enter an integer!\n";
    } else {  // if it is an int, check whether in range
      input = stoi(input_string);  // convert to int
      if (input >= 1 && input <= menu_items) {
        valid_input = true;
      } else {
        cout << "Invalid menu item!\n";
        valid_input = false;
      }
    }
  } while (valid_input == false);

  return input;
}
 
void menu_item_1() {
  cout << "\n>> Path Planning\n";
  // you can call a function from here that handles menu 1
  path_finder::execute();

}

void menu_item_2() {
  bool menu_2_chosen = true;
  while(menu_2_chosen == true){
    cout << "\n>> Robot Battery Management\n";
    cout << "\n------- " <<  Colour::RGB_colour(128, 239, 128, 1, 0, 0) << Colour::BLACK << "Robot Battery Management" << Colour::RESET << " ------\n";
    cout << "|\t                              |\n";
    cout << "|\t" << Colour::RGB_colour(128, 239, 128, 1) << Colour::BLACK << "1. Display Robot Battery" << Colour::RESET << "      |\n";
    cout << "|\t" << Colour::RGB_colour(128, 239, 128, 1) << Colour::BLACK << "2. Charge Robot Battery" << Colour::RESET << "       |\n";
    cout << "|\t" << Colour::RGB_colour(128, 239, 128, 1) << Colour::BLACK << "3. Discharge Robot Battery" << Colour::RESET << "    |\n";
    cout << "|\t" << Colour::BG_RED << "4. Back to main menu" << Colour::RESET << "          |\n";
    cout << "|\t                              |\n";
    cout << "---------------------------------------\n";

    int user_choice = get_menu2_user_input();
    switch(user_choice){
      case 1:{
        cout << Colour::CLEAR;
        cout << "\n>> Display Robot Battery \n";
        cout << endl;

        int robot_battery = check_robot_battery();
        cout << endl;
        cout << "Battery: " << robot_battery << "%" << endl;
        Robot robot;
        robot.set_battery(robot_battery);
        int battery_value = robot.get_battery();
        display_battery(battery_value);

        break;
      }
      case 2:{
        cout << Colour::CLEAR;
        cout << "\n>> Charge Robot Battery \n";
        cout << endl;
        
        int robot_battery = check_robot_battery();
        cout << endl;
        cout << "Battery pre-charging: " << robot_battery << "%" << endl;
        Robot robot;
        robot.set_battery(robot_battery);
        int battery_value = robot.get_battery();
        charge_battery(battery_value);
        break;
      }
      case 3:{
        cout << Colour::CLEAR;
        cout << "\n>> Discharge Robot Battery \n";
        cout << endl;
        int robot_battery = check_robot_battery();
        Robot robot;
        robot.set_battery(robot_battery);
        int battery_value = robot.get_battery();

        int reduction = check_battery_redcution();
        cout << endl;
        cout << "Battery pre-discharging: " << robot_battery << "%" << endl;
        discharge_battery(battery_value, reduction);
        break;
      }
      case 4:
        menu_2_chosen = false;
        break;
    } 
  }
  // you can call a function from here that handles menu 2
}

void menu_item_3() {
  bool menu_3_chosen = true;
  while(menu_3_chosen == true){
    cout << "\n>> Orders and Warehouse Management\n";
    cout << "\n--- " <<  Colour::RGB_colour(0, 204, 255, 1) << Colour::BLACK << "Orders and Warehouse Management" << Colour::RESET << " ---\n";
    cout << "|\t                              |\n";
    cout << "|\t" <<  Colour::RGB_colour(0, 204, 255, 1) << Colour::BLACK << "1. Fullfill Customer Order" << Colour::RESET << "    |\n";
    cout << "|\t" <<  Colour::RGB_colour(0, 204, 255, 1) << Colour::BLACK << "2. Add New Item" << Colour::RESET << "               |\n";
    cout << "|\t" <<  Colour::RGB_colour(0, 204, 255, 1) << Colour::BLACK << "3. Update Item Price" << Colour::RESET << "          |\n";
    cout << "|\t" <<  Colour::RGB_colour(0, 204, 255, 1) << Colour::BLACK << "4. Update Item Quantity" << Colour::RESET << "       |\n";
    cout << "|\t" <<  Colour::RGB_colour(0, 204, 255, 1) << Colour::BLACK << "5. Delete Item" << Colour::RESET << "                |\n";
    cout << "|\t" <<  Colour::BG_RED << "6. Back to main menu" << Colour::RESET << "          |\n";
    cout << "|\t                              |\n";
    cout << "---------------------------------------\n";
    int user_choice = get_menu3_user_input();

    switch(user_choice){
      case 1:{
        cout << Colour::CLEAR;
        cout << "\n>> Fullfill Order \n";
        cout << endl;

        int robot_battery = check_robot_battery();
        cout << endl;
        pick_up_order("my_warehouse_items.csv", "order_customer.csv", robot_battery);
        break;
      }

      case 2:{
        cout << Colour::CLEAR;
        cout << "\n>> Add New Item \n";
        cout << endl;

        string new_item;
        cout << "Enter the new item you wish to add to the warehouse: ";
        getline(cin >> ws, new_item);

        float new_price = check_price();
        int new_quantity = check_quantity();

        append_item("my_warehouse_items.csv", new_item, new_price, new_quantity);
        break;
      }

      case 3:{
        cout << Colour::CLEAR;
        cout << "\n>> Update Item Price \n";
        cout << endl;
        string item;
        cout << "Enter the item you wish to update its price: ";
        getline(cin >> ws, item);

        float new_price = check_price();

        update_price("my_warehouse_items.csv", item, new_price);
        break;
      }

      case 4:{
        cout << Colour::CLEAR;
        cout << "\n>> Update Item Quantity \n";
        cout << endl;
        string item;
        cout << "Enter the item you wish to update its quantity: ";
        getline(cin >> ws, item);

        float new_quantity = check_quantity();

        update_quantity("my_warehouse_items.csv", item, new_quantity);
        break;
      }

      case 5:{
        cout << Colour::CLEAR;
        cout << "\n>> Delete Item \n";
        cout << endl;
        string item;
        cout << "Enter the item you wish to delete from the warehouse: ";
        getline(cin >> ws, item);

        delete_item("my_warehouse_items.csv", item);
        
        break;
      }

      case 6:{
        menu_3_chosen = false;
        break;
      }
    } 
  }
  // you can call a function from here that handles menu 2
}

void menu_item_4(){
  cout << Colour::CLEAR << "\n>> PID 4\n";
  pid_menu();
  // you can call a function from here that handles menu 4

}

