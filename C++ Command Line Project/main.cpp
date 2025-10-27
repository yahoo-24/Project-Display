//ELEC/XJEL2645 - Simple Command Line Interface example
// Dr Craig A. Evans, Dr Tim J. Amsdon and Dr James Avery

#include <iostream>
#include <regex> // needed to parse inputs
#include <windows.h>

#include "funcs.h" // sub functions go in here
#include "Colours.h" 


void main_menu(); // runs in the main loop
void print_main_menu(); // output the main menu description

int get_user_input(); // get a valid integer choice from the user input
void select_menu_item(int input); // run the desired code based on the users choice
void go_back_to_main(); // print message to prompt user to return to main menu
bool is_integer(std::string num); // check input is 

int main(int argc, char const *argv[]){
  SetConsoleOutputCP(CP_UTF8);
  // this will run forever until we hit the exit(1); line in select_menu_item()
  while(1){
    main_menu();
  }
  return 0;
}

void main_menu() {
  print_main_menu();
  int input = get_user_input();
  select_menu_item(input);
}

int get_user_input() { // checking the user input in the main menu
  int input;
  string input_string;
  bool valid_input = false;
  int menu_items = 5; // we had 4 options + exit option

  do {
    cout << "\nSelect item: ";
    getline(cin >> ws, input_string);
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

void select_menu_item(int input) { // selecting between sub menus depending on the user input 
  switch (input) {
    case 1:
      menu_item_1();
      break;
    case 2:
      cout << Colour::CLEAR;
      menu_item_2();
      break;
    case 3:
      cout << Colour::CLEAR;
      menu_item_3();
      break;
    case 4:
      menu_item_4();
      break;
    default:
      cout << "Bye!\n";
      exit(1);
      break;
  }
}

void print_main_menu() { // Main menu content
  cout << Colour::CLEAR;
  cout << "\n----------------" << Colour::RGB_colour(255, 183, 3, 0, 1, 0, 1) << " Main menu " << Colour::RESET << "-----------------\n";
  cout << "|\t                                   |\n";
  cout << "|\t" << Colour::RGB_colour(0, 128, 128) << "1. Path Planning" << Colour::RESET <<"                   |\n";
  cout << "|\t" << Colour::RGB_colour(128, 239, 128) << "2. Robot Battery Management" << Colour::RESET <<"        |\n";
  cout << "|\t" << Colour::RGB_colour(0, 204, 255) << "3. Orders and Warehouse Management" << Colour::RESET <<" |\n";
  cout << "|\t" << Colour::RGB_colour(252, 133, 0) << "4. PID Controller" << Colour::RESET <<"                  |\n";
  cout << "|\t" << Colour::RGB_colour(220, 20, 60) << "5. Exit" << Colour::RESET <<"                            |\n";
  cout << "|\t                                   |\n";
  cout << "--------------------------------------------\n";
}

void go_back_to_main() { // message to promp the user to go back to main menu
  string input;
  do {
    cout << "\nEnter 'b' or 'B' to go back to main menu: ";
    cin >> input;
  } while (input != "b" && input != "B");
}

