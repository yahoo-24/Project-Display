#include <windows.h>
#include <iomanip>
#include <sstream>
#include <iostream>
#include <regex>
#include <thread>
#include <chrono>
#include <vector>
#include <string>
#include <fstream>

#include "util.h"
using namespace std;

// https://stackoverflow.com/questions/447206/c-is_float-function
bool is_float( string myString ) {
    istringstream iss(myString);
    float f;
    iss >> noskipws >> f; // noskipws considers leading whitespace invalid
    // Check the entire string was consumed and if either failbit or badbit is set
    return iss.eof() && !iss.fail(); 
}

int check_robot_battery(){ // function to check the validity of the battery value 
  int robot_battery; // the battery value we are interested in
  string robot_battery_str; // string version of it 

  do { 
  // user prompt to enter a battery value between 0 and 100, and should be a multiple of 5
  cout << "Enter the battery (0% - 100%) of the robot (multiple of 5): "; 

  // assign it to the string
  cin >> robot_battery_str;

  // while it is not an integer string
  // keep asking for a integer string
  while(is_integer(robot_battery_str) == false){
  cout << "Enter an integer !!: ";
  cin >> robot_battery_str;  
  }

  // then assign it to to the integer value
  robot_battery = stoi(robot_battery_str);

  // we repeat that while the value passed is out of the 0 - 100 range or it is not a multiple of 5
  } while ((stoi(robot_battery_str) % 5) != 0 || stoi(robot_battery_str) > 100 || stoi(robot_battery_str) < 0);

  // return the valid integer battery value
  return robot_battery;
}

int check_battery_redcution(){ // function to check the validity of the reduction amount
  int battery_reduction; // the reduction value that we are interested in
  string battery_reduction_str; // string version of it

  do { 
  // user prompt to enter a reduction value between 0 and 100, and should be a multiple of 5
  cout << "Enter the reduction (0% - 100%) value (multiple of 5): ";

  // assign it to the string 
  cin >> battery_reduction_str;

  // while it is not an integer string
  // keep asking for a integer string
  while(is_integer(battery_reduction_str) == false){
  cout << "Enter an integer !!: ";
  cin >> battery_reduction_str;  
  }

  // then assign it to to the integer value
  battery_reduction = stoi(battery_reduction_str);

  // we repeat that while the value passed is out of the 0 - 100 range or it is not a multiple of 5
  } while ((stoi(battery_reduction_str) % 5) != 0 || stoi(battery_reduction_str) > 100 || stoi(battery_reduction_str) < 0);

  // return the valid integer reduction value
  return battery_reduction;
}


// Robot Battery Management functions:

// function to show and draw current battery's value 
int display_battery(int battery){
    // if the value passed is out of the 0 - 100 range, adjsut the value appropriately to either 0 or 100
    if(battery < 0){
        cout << "Battery (" << battery << ") should be in the range (0 - 100)%" << endl;
        cout << "Battery adjusted to 0%" << endl;
        battery = 0;
    }
    else if(battery > 100){
        cout << "Battery (" << battery << ") should be in the range (0 - 100)%" << endl;
        cout << "Battery adjusted to 100%" << endl;
        battery = 100;
    }

    vector<string> battery_vector (20, " ━"); // start with a emtpy vector, with 20 elements of dashes " ━"
    int vector_size = battery_vector.size(); // get the size of the vector
    int squares_num = battery / 5; // get the number of square emojis, or square blocks required in the vector

    if(battery <= 100 && battery > 50){ // if 50% < battery <= 100%, fill in the vector with "🟩", then continue with " ━" if possible
        for(int i = 0; i < squares_num; i++){
            battery_vector[i] = "🟩";
        }
    }
    else if(battery <= 50 && battery > 20){ // else if 20% < battery <= 50%, fill in the vector with "🟨", then " ━"
        for(int i = 0; i < squares_num; i++){
            battery_vector[i] = "🟨";
        }
    }
    
    else if(battery <= 20 && battery > 0){ // else if 0% < battery <= 20%, fill in the vector with "🟥", then " ━"
        for(int i = 0; i < squares_num; i++){
            battery_vector[i] = "🟥";
        }
    }

    // Draw a shape resembling the a real battery, and display the current battery vector using a for loop
    cout << "╭─────────────────────────────────────────╮" << endl;
    cout << "│";
    for(int x = 0; x < battery_vector.size(); x++){
        cout << battery_vector[x];
    }
    cout << " │ " << battery << "%" << endl;
    cout << "╰─────────────────────────────────────────╯" << endl;

    return battery; // return also the current battery value
}

// function to generate the vector corresponding to a given battery value
vector<string> generate_battery_vector(int battery){
    // handling edge cases where battery is out of the range 0 - 100
    if(battery < 0){
        cout << "Battery (" << battery << ") should be in the range (0 - 100)%" << endl;
        cout << "Battery adjusted to 0%" << endl;
        battery = 0;
    }
    else if(battery > 100){
        cout << "Battery (" << battery << ") should be in the range (0 - 100)%" << endl;
        cout << "Battery adjusted to 100%" << endl;
        battery = 100;
    }

    vector<string> battery_vector (20, " ━"); // start with a emtpy vector, with 20 elements of dashes " ━"
    int vector_size = battery_vector.size(); // get the size of the vector
    int squares_num = battery / 5; // get the number of square emojis, or square blocks required in the vector

    if(battery <= 100 && battery > 50){ // if 50% < battery <= 100%, fill in the vector with "🟩", then continue with " ━" if possible
        for(int i = 0; i < squares_num; i++){
            battery_vector[i] = "🟩";
        }
    }
    else if(battery <= 50 && battery > 20){ // else if 20% < battery <= 50%, fill in the vector with "🟨", then " ━"
        for(int i = 0; i < squares_num; i++){
            battery_vector[i] = "🟨";
        }
    }
    
    else if(battery <= 20 && battery > 0){ // else if 0% < battery <= 20%, fill in the vector with "🟥", then " ━"
        for(int i = 0; i < squares_num; i++){
            battery_vector[i] = "🟥";
        }
    }

    return battery_vector; // return the current battery vector, which will be needed in other functions
}

// function to show the charging process of the battery
int charge_battery(int battery){ 
    // generate the battery vector
    vector<string> battery_vector = generate_battery_vector(battery);
    int size = battery_vector.size(); // get the size of the generated vector

    // if battery was already 100%, meaning no further charging is possible
    // then we display a message, draw the full charged battery, and return 100
    if(battery == 100){
        cout << "Battery is already 100% !!!" << endl;
        display_battery(100);
        return 100;
    }
    else{ // else, meaning the battery is less than 100% 
        int squares_num = battery / 5; // calculate the number of square emojis present in the passed vector

        // Draw the battery shape 
        cout << "╭─────────────────────────────────────────╮" << endl;
        cout << "│                                         │" << endl; 
        cout << "╰─────────────────────────────────────────╯";

        // Move the cursor up by 1 line, to show a charging animation each time the battery is incremented by 5%
        cout << "\033[F";  

        // Start charging animation
        for (int i = squares_num; i <= size; i++) {
            cout << "\r│"; // Start of battery bar line

            // Update the battery vector for all squares up to the current one
            if (i <= 4) { // Red range for 0% < battery <= 20%
                for (int k = 0; k < i; k++) {
                    battery_vector[k] = "🟥";
                }
            } else if (i > 4 && i <= 10) { // Yellow range for 20% < battery <= 50%
                for (int k = 0; k < i; k++) {
                    battery_vector[k] = "🟨";
                }
            } else if (i > 10) { // Green range for 50% < battery <= 100%
                for (int k = 0; k < i; k++) {
                    battery_vector[k] = "🟩";
                }
            }

            // After one incrementation (by 5%), display the current state of the vector
            for(int j = 0; j < size; j++){
                cout << battery_vector[j];
            }

            // and calculate the value of the battery in the current state 
            int current_battery = battery + 5*(i - squares_num); // increments battery by 5% each iteration
            cout << " │ " << current_battery << "%";

            // Animation delay of 1s (1000 ms)
            Sleep(1000);
        }

        // Move down by 2 lines
        cout << endl; 
        cout << endl;
        cout << "Battery post-charging: 100%" << endl;

        // Return the fully charged battery value, 100
        return 100;
    }
    
}

// function to show the discharging process of the battery, with a specific amount of reduction
int discharge_battery(int battery, int reduction){  
    // generate the battery vector
    vector<string> battery_vector = generate_battery_vector(battery);
    int size = battery_vector.size(); // get the size of generated vector

    // if battery was 0%, no further reduction is possible
    // therefore, display a message, draw the empty battery and return 0
    if(battery == 0){
        cout << "Battery is already 0% !!! " << endl;  
        display_battery(0);
        return 0;
    }
    // else if the reduction was greater than the battery, then discharging is not possible
    // therefore, we display an overflow message, we draw the current battery, and return its value
    else if(battery < reduction){
        cout << "Overflow !!!: cannot consume " << reduction << "%" << " from a " << battery << "% battery" << endl;  
        display_battery(battery);
        return battery;
    }

    else{ // else, proceed with the discharging process
        int squares_num = battery / 5; // calculate the number of square emojis present in the passed vector 
        int squares_reduction = reduction / 5; // calculate the number of square reduction required

        // Draw the battery shape 
        cout << "╭─────────────────────────────────────────╮" << endl;
        cout << "│                                         │" << endl; 
        cout << "╰─────────────────────────────────────────╯";

        // Move the cursor up by 1 line, to show a discharging animation each time the battery is reduced by 5%
        cout << "\033[F";  

        // Start discharging animation
        for(int i = squares_num; i >= (squares_num - squares_reduction); i--){
            cout << "\r│"; // Start of battery bar line

            // Update the battery vector for all squares up to the current one, then finish off with " ━" where necessary
            if(i > 10 && i < 20){
                for(int k = 0; k <= i; k++){ // Green range for 50% < battery <= 100%
                    battery_vector[k] = "🟩";
                }
                battery_vector[i] = " ━";
            }
            else if(i >= 5 && i <= 10){ // Yellow range for 50% < battery <= 100%
                for(int k = 0; k <= i; k++){
                    battery_vector[k] = "🟨";
                }
                battery_vector[i] = " ━";
            }

            else if(i < 5){ // Red range for 50% < battery <= 100%
                for(int k = 0; k <= i; k++){
                    battery_vector[k] = "🟥";
                }
                battery_vector[i] = " ━";
            }

            // After one decrement (by 5%), display the current state of the vector
            for(int j = 0; j < size; j++){
                cout << battery_vector[j];
            }

            // and calculate the value of the battery in the current state 
            int current_battery = battery + 5*(i - squares_num); // decrements battery by 5% each iteration
            cout << " │ " << current_battery << "%";

            // Animation delay of 1s (1000 ms)
            Sleep(1000);

            
        }
        // Move down by 2 lines
        cout << endl;
        cout << endl;
        cout << "Battery post-discharging: " << battery - reduction << "%" <<  endl;
        cout << "reduced by " << reduction << "%" <<  endl;

        // return the discharged battery value
        return (battery - reduction);
    }
}


// Orders and Warehouse Management functions:

string lowercase(string str){ // function to turn all the character of a string into lower case
    for(int i = 0; i < str.length(); i++){
        str[i] = tolower(str[i]);
    }
    return str;
}

string precision(float flt){ // function to transform a float in a 2 decimal point string float value
    ostringstream oss;
    oss << fixed << setprecision(2) << flt;
    string precision_string = oss.str();
    return precision_string;
}

// function to append a new item into the warehouse
void append_item(string store_item_file, string item_name, float item_price, int item_quantity){
    string lower_item_name = lowercase(item_name); // turn into lower case string 

    ifstream file1; // reading a file

    string store_line1 = ""; // string to store one line of the warehouse csv file each time
    vector<string> store_items; // vector to store the items' names available in the warehouse 
    vector<string> store_lines; // vector to store all the lines of the warehouse csv file
    bool exists = false; // bool value to check if the item to be appended already exists in the warehouse

    file1.open(store_item_file); // open the file containg the warehouse items info 

    while(getline(file1, store_line1)){ // reading line by line
        if (store_line1.empty() == false) { // if the line is not empty
            store_lines.push_back(store_line1); // store the line in the vector

            int store_pos1 = store_line1.find(","); // look for the comma position in the line
            string store_item = store_line1.substr(0, store_pos1); // cut the line and get the store item name
            
            if(lowercase(store_item) == lower_item_name){ 
                // if the item already exists, turn the bool into true
                exists = true;
                // then break from the while loop since a match is found
                break;
            }
            else{ // else, keep it false and keep iterating in the while loop
               exists = false; 
            }
            
        }
        else{ // else, if the line is empty, meaning everyline was read, then break from the while loop
            break;
        }
    }
    file1.close();

    if(exists == false){ // if the bool was false, meaning the item does not exist in the warehouse
        // then write to the warehouse file and append the item at the end of the warehouse items list
        ofstream f; // writing to the file
        f.open(store_item_file, ios_base::app); // open and make sure to append the item rather than overwriting
        string price_str = precision(item_price); // price set to 2 decimals and turned into string
        string quantity_str = to_string(item_quantity); // quantity turned into string as well

        // make a new line
        // and append it the warehouse file
        f << lowercase(item_name) << "," << price_str << "," << quantity_str << "\n";
        f.close(); 
        cout << "'" << lowercase(item_name) << "' added successfully to the warehouse" << endl;
    }
    else{ // else, meaning it already exists in the warehouse, then display this message
        cout << "'" << lowercase(item_name) << "' already exists in the warehouse. Maybe update its price or quantity " <<endl;
    }
    
}

// function to update the quantity of an existing item
void update_quantity(string store_item_file, string item_name, int new_item_quantity){
    string lower_item_name = lowercase(item_name); // turn into a lower case string
     
    ifstream file1; // reading a file 
    string store_line1 = ""; // string to store one line of the warehouse csv file each time
    vector<string> store_items; // vector to store the items' names available in the warehouse
    vector<string> store_prices; // vector to store the items' prices available in the warehouse
    vector<string> store_quantities; // vector to store the items' quantities available in the warehouse
    vector<string> store_lines; // vector to store all the lines of the warehouse csv file
    bool exists = false; // bool to check if an item we wish to update its quantity exists in the warehouse
    file1.open(store_item_file); // open the warehouse csv file

    while(getline(file1, store_line1)){ // read line by line
        if (store_line1.empty() == false) { // if line read is not empty
            // then append that line into the store_lines vector
            store_lines.push_back(store_line1);

            int store_pos1 = store_line1.find(","); // then find the position of the 1st comma
            string store_item = store_line1.substr(0, store_pos1); // cut that line before the 1st comma, to get the item name
            store_items.push_back(store_item); // append that item name into the items' names vector

            string store_line2 = store_line1.substr(store_pos1 + 1, store_line1.length()); // use that line after cutting 
            int store_pos2 = store_line2.find(","); // then find the position of the 2nd comma
            string store_price = store_line2.substr(0, store_pos2); // cut that line before the 2nd comma, to get the item price
            store_prices.push_back(store_price); // append that item price into the items' prices vector

            string store_line3 = store_line2.substr(store_pos2 + 1, store_line2.length()); // use that line after cutting
            string store_quantity = store_line3; // assign it to the quantity of the item, since we are only left with it
            // and no more cutting is needed
            store_quantities.push_back(store_quantity); // append that item quantity to the items' quantities vector

            if(lowercase(store_item) == lower_item_name){ // during that process, check if the item we wish to update exists in warehoouse
                exists = true; // if yes, turn the bool into true
                // and continue to the while loop
            }
            
        }
        else{ // else if the line read is empty, meaning we have finished reading all the lines
            // then break from the while loop
            break;
        }
    }
    file1.close(); // close the warehouse csv file 

    if(exists == true){ // if we found that the item we wish to update exists
    // then we proceed to the uodating process
    // we iterate through the the items vector
        for(int i = 0; i < store_items.size(); i++){  
            // we check where this match happens 
            if(lowercase(store_items[i]) == lowercase(item_name)){
                // and we update the item's quantity and replace the old line with a new one
                store_quantities[i] = to_string(new_item_quantity); // turn the new quantity passed into a string

                // update the new line
                store_lines[i] = store_items[i] + "," + store_prices[i] + "," + store_quantities[i];
                // then break from the iteration
                break;
            }
        }

        ofstream ff; // writing to a file 
        ff.open(store_item_file); // open the warehouse csv file

        // iterate through the updated store_lines vector
        for(int i = 0; i < store_lines.size(); i++){
            // overwrite the old lines of the warehouse csv file with the new lines
            ff << store_lines[i] << endl;
        }
        ff.close(); // close the the file

        // display a message
        cout << "'" << lowercase(item_name) << "' store quantity updated to: " << new_item_quantity << endl;
    }

    else{ // else if the item does not exist in the warehouse, therefore we cannot update its quantity
        // display a message as well
        cout << "'" << lowercase(item_name) << "' quantity cannot be updated because it does not exist in store !" <<endl;
    }
}

// function to update the price of an existing item
void update_price(string store_item_file, string item_name, float new_item_price){
    string lower_item_name = lowercase(item_name); // turn into lower case string

    ifstream file1; // reading a file 
    string store_line1 = ""; // string to store one line of the warehouse csv file each time
    std::vector<string> store_items; // vector to store the items' names available in the warehouse
    std::vector<string> store_prices; // vector to store the items' prices available in the warehouse
    std::vector<string> store_quantities; // vector to store the items' quantities available in the warehouse
    std::vector<string> store_lines; // vector to store all the lines of the warehouse csv file
    bool exists = false; // bool to check if an item we wish to update its price exists in the warehouse
    file1.open(store_item_file); // open the warehouse csv file

    while(getline(file1, store_line1)){ // read line by line
        if (store_line1.empty() == false) { // if line read is not empty
            // then append that line into the store_lines vector
            store_lines.push_back(store_line1);

            int store_pos1 = store_line1.find(","); // then find the position of the 1st comma
            string store_item = store_line1.substr(0, store_pos1); // cut that line before the 1st comma, to get the item name
            store_items.push_back(store_item); // append that item name into the items' names vector

            string store_line2 = store_line1.substr(store_pos1 + 1, store_line1.length()); // use that line after cutting 
            int store_pos2 = store_line2.find(","); // then find the position of the 2nd comma
            string store_price = store_line2.substr(0, store_pos2); // cut that line before the 2nd comma, to get the item price
            store_prices.push_back(store_price); // append that item price into the items' prices vector

            string store_line3 = store_line2.substr(store_pos2 + 1, store_line2.length()); // use that line after cutting
            string store_quantity = store_line3; // assign it to the quantity of the item, since we are only left with it
            // and no more cutting is neede
            store_quantities.push_back(store_quantity); // append that item quantity to the items' quantities vector

            if(lowercase(store_item) == lower_item_name){ // during that process, check if the item we wish to update exists in warehoouse
                exists = true; // if yes, turn the bool into true
                // and continue to the while loop
            }
            
        }
        else{ // else if the line read is empty, meaning we have finished reading all the lines
            // then break from the while loop
            break;
        }
    }
    file1.close(); // close the file

    if(exists == true){ // if the item exists, then we proceed to the price updating process
        // iterate through the store_items vector and find where is the match
        for(int i = 0; i < store_items.size(); i++){
            if(store_items[i] == lowercase(item_name)){ // when the match is found
                // change the float price into a 2 decimal string float
                store_prices[i] = precision(new_item_price);

                // then update the full line, then break
                store_lines[i] = store_items[i] + "," + store_prices[i] + "," + store_quantities[i];
                break;
            }
        }
        // after breaking, start writing to the same warehouse csv file
        ofstream ff;
        ff.open(store_item_file); // open the csv file

        for(int i = 0; i < store_lines.size(); i++){ // iterate through the updated lines
            ff << store_lines[i] << endl; // and overwrite the old lines with the new updated lines
        }

        ff.close(); // close the file
        // then display a message
        cout << "'" << lowercase(item_name) << "' store price updated to: £" << precision(new_item_price) << endl;

    }
    else{
        // else if it does not exist, display another message
        cout << "'" << lowercase(item_name) << "' price cannot be updated because it does not exist in store !" <<endl;
    }
}

// function to update the quantities of warehouse items after fullfilling an order
void auto_update_quantities(string store_item_file, string order_file){
    ifstream file1; // reading a file 
    string store_line1 = ""; // string to store one line of the warehouse csv file each time
    std::vector<string> store_items; // vector to store the items' names available in the warehouse
    std::vector<string> store_prices; // vector to store the items' prices available in the warehouse
    std::vector<int> store_quantities; // vector to store the items' quantities available in the warehouse
    std::vector<string> store_lines; // vector to store all the lines of the warehouse csv file
    file1.open(store_item_file); // open the warehouse csv file
    
    while(getline(file1, store_line1)){ // read line by line
        if (store_line1.empty() == false) { // if line read is not empty
            // then append that line into the store_lines vector
            store_lines.push_back(store_line1);

            int store_pos1 = store_line1.find(","); // then find the position of the 1st comma
            string store_item = store_line1.substr(0, store_pos1); // cut that line before the 1st comma, to get the item name
            store_items.push_back(store_item); // append that item name into the items' names vector

            string store_line2 = store_line1.substr(store_pos1 + 1, store_line1.length()); // use that line after cutting 
            int store_pos2 = store_line2.find(","); // then find the position of the 2nd comma
            string store_price = store_line2.substr(0, store_pos2); // cut that line before the 2nd comma, to get the item price
            store_prices.push_back(store_price); // append that item price into the items' prices vector

            string store_line3 = store_line2.substr(store_pos2 + 1, store_line2.length()); // use that line after cutting
            string store_quantity = store_line3; // assign it to the quantity of the item, since we are only left with it
            // and no more cutting is needed
            store_quantities.push_back(stoi(store_quantity)); // append that item quantity to the items' quantities vector
        }
        else{ // else meaning the line is now empty
            // then break
            break;
        }
    }
    file1.close(); // close that file


    // order items:
    ifstream file2; // read from file
    string order_line1 = ""; // string to store one line of the order csv file each time
    std::vector<string> order_items; // vector to store the items' names available in the order
    std::vector<int> order_quantities; // vector to store the items' quantities available in the order
    std::vector<string> order_lines; // vector to store all the lines of the warehouse csv file

    file2.open(order_file); // open the order csv file
    while(getline(file2, order_line1)){ // read line by line
        if (order_line1.empty() == false) {
            // append each line
            order_lines.push_back(order_line1);

            // find the comma position and cut to get the item name
            int order_pos1 = order_line1.find(",");
            string order_item = order_line1.substr(0, order_pos1);
            order_items.push_back(order_item); // append the item name 

            // the rest is the quantity
            string order_line2 = order_line1.substr(order_pos1 + 1, order_line1.length());
            string order_quantity = order_line2;
            order_quantities.push_back(stoi(order_quantity)); // append the integer version of the quantity
        }
        else{ // else, meaning we finished reading the lines
            // then break
            break;
        }
    }
    file2.close(); // close the file
    
    // now iterate through the order items vector
    // and with a nested loop, iterate through the store items vector
    for(int i = 0; i < order_items.size(); i++){ 
        for(int j = 0; j < store_items.size(); j++){
            if(lowercase(order_items[i]) == lowercase(store_items[j])){ // if there is a match, then apply the following 
                if(store_quantities[j] == 0){ // if the items quantity in the warehouse was 0, keep it 0
                // since not further reduction is possible
                    store_quantities[j] = 0;
                }

                else if(store_quantities[j] < order_quantities[i]){ // else if it is less than the orders quantity
                    // keep it as it is since there is less quantity in the warehouse
                    store_quantities[j] = store_quantities[j];
                }

                else{ // else, reduce the warehouse quantity by taking out the order quantity
                    store_quantities[j] = store_quantities[j] - order_quantities[i];
                }
                
                // then update the store lines with the updated store quantity
                store_lines[j] = store_items[j] + "," + store_prices[j] + "," + to_string(store_quantities[j]);
                // then break and move to the next ordered item
                break;
            }
        }
    }

    // after done with updating
    ofstream f; // write to a file
    f.open(store_item_file); // open the warehouse csv file again

    for(int i = 0; i < store_lines.size(); i++){ // iterate through the updated store lines
        f << store_lines[i] << endl; // and overwrite the old lines with the updated ones
    }
    f.close(); // as the end, close the file
}

// function that gets the total of the order fullfilled
void get_total(string store_item_file, string order_file){
    ifstream file1; // reading a file 
    string store_line1 = ""; // string to store one line of the warehouse csv file each time
    std::vector<string> store_items; // vector to store the items' names available in the warehouse
    std::vector<float> store_prices; // vector to store the items' prices available in the warehouse
    std::vector<int> store_quantities; // vector to store the items' quantities available in the warehouse
    std::vector<string> store_lines; // vector to store all the lines of the warehouse csv file
    file1.open(store_item_file); // open the warehouse csv file

    while(getline(file1, store_line1)){ // read line by line
        if (store_line1.empty() == false) { // if line read is not empty
            // then append that line into the store_lines vector
            store_lines.push_back(store_line1);

            int store_pos1 = store_line1.find(","); // then find the position of the 1st comma
            string store_item = store_line1.substr(0, store_pos1); // cut that line before the 1st comma, to get the item name
            store_items.push_back(store_item); // append that item name into the items' names vector

            string store_line2 = store_line1.substr(store_pos1 + 1, store_line1.length()); // use that line after cutting 
            int store_pos2 = store_line2.find(","); // then find the position of the 2nd comma
            string store_price = store_line2.substr(0, store_pos2); // cut that line before the 2nd comma, to get the item price
            store_prices.push_back(stof(store_price)); // append that item price into the items' prices vector

            string store_line3 = store_line2.substr(store_pos2 + 1, store_line2.length()); // use that line after cutting
            string store_quantity = store_line3; // assign it to the quantity of the item, since we are only left with it
            // and no more cutting is needed
            store_quantities.push_back(stoi(store_quantity)); // append that item quantity to the items' quantities vector
        }
        else{ // else, meaning the line read is empty
            // then break since we reached the end of the list
            break;
        }
    }
    file1.close(); // close the file 


    // order items:
    ifstream file2; // read from file
    string order_line1 = ""; // string to store one line of the order csv file each time
    std::vector<string> order_items; // vector to store the items' names available in the order
    std::vector<int> order_quantities; // vector to store the items' quantities available in the order
    std::vector<string> order_lines; // vector to store all the lines of the warehouse csv file

    file2.open(order_file); // open the order csv file
    while(getline(file2, order_line1)){ // read line by line
        if (order_line1.empty() == false) {
            // append each line
            order_lines.push_back(order_line1);

            // find the comma position and cut to get the item name
            int order_pos1 = order_line1.find(",");
            string order_item = order_line1.substr(0, order_pos1);
            order_items.push_back(order_item); // append the item name 

            // the rest is the quantity
            string order_line2 = order_line1.substr(order_pos1 + 1, order_line1.length());
            string order_quantity = order_line2;
            order_quantities.push_back(stoi(order_quantity)); // append the integer version of the quantity
        }
        else{ // else, meaning we finished reading the lines
            // then break
            break;
        }
    }
    file2.close(); // close the file

    // calculate total:
    float total = 0; // float to store the total value
    float ordered_item_price = 0; // float to store a temporary value of the ordered item with its quantity
    int counter = 0; // create a counter

    std::vector<string> existing_items; // vector to store the order items that exist in store
    std::vector<string> existing_items_prices; // vector to store their corresponding prices 
    std::vector<string> existing_items_quantities; // and the corresponding quantities in the warehouse

    // 2 bool variables to handle edge cases
    bool nothing_left = false; 
    bool not_enough = false;

    // iterate through the order items
    for(int x = 0; x < order_items.size(); x++){
        // initial values for the variables
        counter = 0;
        nothing_left = false;
        not_enough = false;

        // Meanwhile, iterate through the warehouse items
        for(int y = 0; y < store_items.size(); y++){
            if(lowercase(order_items[x]) == lowercase(store_items[y])){ // if there was a match, do the following

                if(store_quantities[y] == 0){ // if quantity is left in store for that ordered item
                    // display a message
                    cout << "No '" << store_items[y] << "' left in the store" << endl;
                    nothing_left = true; // turn the nothing_left bool into true
                    // then break and move to the next ordered items
                    break;
                }

                else if(order_quantities[x] > store_quantities[y]){ // else if the quantity ordered is greater
                    // display a message
                    cout << "Not enough '" << store_items[y] << "' in the store. There are " << store_quantities[y] << " left"<< endl;
                    not_enough = true; // then turn the not_enough bool into true
                    // then break and move to the next ordered items
                    break;
                }

                else{ // else, meaning the ordered item is available perfectly
                    ordered_item_price = (store_prices[y])*(order_quantities[x]); // calculate the price of the item, based on the quantity ordered
                    total = total + ordered_item_price; // add that to the total 
                    existing_items.push_back(lowercase(store_items[y])); // append that ordered item to the existing items vector
                    existing_items_prices.push_back(precision(store_prices[y])); // append its warehouse price into the existing items prices vector
                    existing_items_quantities.push_back(to_string(order_quantities[x])); // append its warehouse price into the existing items quantities vector
                    counter = 0; // reset the counter 
                    break;
                }
            }   
        }

        // edge case of the customer orders an item that does not exist
        for(int i = 0; i < existing_items.size(); i++){ // we iterate through the existing items vector
            if(lowercase(order_items[x]) != existing_items[i]){ // if it did not exist, increment the counter
                counter++; // then increment counter

                // if the item still does not exist, and we checked all the existing items
                // then the item does not even exist in store
                if(counter == existing_items.size() && nothing_left == false && not_enough == false){
                    // display this message
                    cout << "'" << order_items[x] << "' does not exist in store" << endl;
                }
            }
        }
    }


    // Printing the receipt
    // the items, prices, and quantities ordered should have the same spacing between them
    // using the following logic
    cout << endl;
    cout << "---------------- Receipt ----------------" << endl;
    cout << "Item                Unit Price     Amount" << endl;
    cout << endl;
    for(int i = 0; i < existing_items.size(); i++){
        int item_length = existing_items[i].length();
        std::string space1 = "";
        for(int s1 = 0; s1 < (20 - item_length); s1++){
            space1 = space1 + " ";
        } 
        int price_length = existing_items_prices[i].length() + 1;
        std::string space2 = "";
        for(int s2 = 0; s2 < (15 - price_length); s2++){
            space2 = space2 + " ";
        } 
        cout << existing_items[i] << space1 << "£" << existing_items_prices[i] << space2 << existing_items_quantities[i] << endl;
    }

    // at the end, print the total, to 2 decimal places
    cout << endl;
    cout << "Total: £" << precision(total) << endl;
    cout << "-----------------------------------------" << endl;
    cout << endl;

    // and automatically update the warehouse quantities, after taking the order
    auto_update_quantities(store_item_file, order_file);

}

// function that deletes an existing item from the warehouse
void delete_item(string store_item_file, string item_name){
    string lower_item_name = lowercase(item_name); // turn string into lowercase

    ifstream file1; // reading the file
    string store_line1 = ""; // string to store the value one line each time
    vector<string> store_items; // vector to store the items names in the warehouse csv file 
    vector<string> store_lines; // vector to store all the lines of the warehouse csv file
    bool exists = false; // bool to check if the item passed exists or not

    file1.open(store_item_file); // open the warehouse csv file

    while(getline(file1, store_line1)){ // read line by line
        if (store_line1.empty() == false) { // if line read is not empty
            // then append that line into the store_lines vector
            store_lines.push_back(store_line1);

            int store_pos1 = store_line1.find(","); // then find the position of the 1st comma
            string store_item = store_line1.substr(0, store_pos1); // cut that line before the 1st comma, to get the item name
            store_items.push_back(store_item); // append that item name into the items' names vector

            if(lowercase(store_item) == lower_item_name){ // if there was match, turn the bool into true, and keep iterating
                exists = true;
            }

        }
        else{ // else, meaning we reached an empty line, then break from the while loop
            break;
        }
    }
    file1.close(); // close the file at the end

    if(exists == true){ // if there was match
        for(int i = 0; i < store_items.size(); i++){ // for loop to find where that match was
            if(store_items[i] == lowercase(item_name)){ 
                // erase the line that contains the item we want to delete
                store_lines.erase(store_lines.begin() + i);
                // then break
                break;
            }
        }

        ofstream ff; // writing to a file
        ff.open(store_item_file); // open the same warehouse csv file 
        for(int i = 0; i < store_lines.size(); i++){ // for loop to iterate through the store lines after deleting the item
            ff << store_lines[i] << endl; // overwrite the old lines with the new ones
        }
        ff.close(); // close the file 
        // and display this message
        cout << "'" << lowercase(item_name) << "' is deleted from the store" << endl;
    }
    else{ // else, meaning the item does not exist in the store 
        // then display this message
        cout << "'" << lowercase(item_name) << "' cannot be deleted because it does not exist in store !" <<endl;
    }
}

// function to display some animation when fullfilling an order
void pick_up_order(string store_item_file, string order_file, int current_robot_battery){
    // if the battery passed is at or less than the 20% threshold, then display these messages
    if(current_robot_battery < 20){ 
        cout << "Cannot collect the order !" << endl;
        cout << "Battery (" << current_robot_battery << "%) below 20% threshold. Send robot to charging station" << endl;
    }
    else if(current_robot_battery == 20){
        cout << "Cannot collect the order !" << endl;
        cout << "Battery at 20% threshold. Send robot to charging station" << endl;
    }
    else{ // else, meaning the battery is valid

        // Animation to indicate that the robot is travelling around the warehouse
        string travelling = "Robot travelling";
        for(int i = 0; i < 3; i++){
            travelling = travelling + ".";
            cout << "\r" << travelling;
            // Animation delay of 1s (1500 ms)
            Sleep(1500);
        }
        cout << "\r";

        ifstream file22; // reading from a csv file
        string order_line = ""; // string to store one line of the order csv file each time
        int num_items_ordered = 0; // variable to store the number of items ordered
        vector<string> order_lines; // vector to store all the lines of the order csv file

        file22.open(order_file); // open the csv order file
        while(getline(file22, order_line)){ // while the line is not empty
            if (order_line.empty() == false) {
                order_lines.push_back(order_line); // append the line to the vector 
                num_items_ordered++; // increment the variable
            }
            else{ // else, meaning we reached an empty line, then break
                break;
            }
        }
        file22.close(); // close the file

        // Animation to indicate that the robot is looking for the items
        string searching = "Searching for items";
        for(int i = 0; i < num_items_ordered; i++){
            cout << "\r" << searching << " (" << i + 1 << ")";
            // Animation delay of 1s (1500 ms)
            Sleep(1500);
        } 
        cout << "\r";


        // Animation to show that indicate the robot is going back the delivery station
        string back = "Back to delivery station";
        for(int i = 0; i < 3; i++){
            back = back + ".";
            cout << "\r" << back;
            // Animation delay of 1s (1500 ms)
            Sleep(1500);
        }
        cout << "\r";
        Sleep(1500);

        // call the get total function, to print the receipt and calculate the total, then it updates the warehouse quantities
        get_total(store_item_file, order_file);

        // decrease the battery by 5% after taking the order and print it 
        current_robot_battery = current_robot_battery - 5;
        cout << "Remaining Battery: " <<  current_robot_battery << "%" << endl;
        // display the battery shape
        display_battery(current_robot_battery);
    }
}

// function to check the validity of the quantity passed 
int check_quantity(){ 
  int quantity; // integer value that we are interested in
  string quantity_str; // string version of that integer value

  // do while loop
  do {
  // ask the user for the quantity 
  cout << "Enter the quantity of the new item (> 0): ";
  cin >> quantity_str; // assign it the string

  while(is_integer(quantity_str) == false){ // while it is not a string integer
  cout << "Enter an integer !!: "; // keep asking for a integer 
  cin >> quantity_str; // reassign it to the string 
  }

  // after exiting the while loop, meaning it is a positive integer, assign it to the real quantity integer
  quantity = stoi(quantity_str); 
  } while (stoi(quantity_str) < 0);

  // return the integer
  return quantity;
}

// function to check the validity of the price value passed
float check_price(){  
  float price; // float value that we are interested in 
  string price_str; // string version of it

  // do while loop
  do {
  // ask the user for the price
  cout << "Enter the price of the new item (> 0): ";
  cin >> price_str; // assign it the string

  while(is_float(price_str) == false){ // while it not a float / integer
  cout << "Enter a float / integer !!: "; // keep asking for a float / integer
  cin >> price_str; // reassign it to the string 
  }

  // after exiting the do while loop, meaning it is a positive float / integer, assign it to the real float price 
  price = stof(price_str);
  } while (stof(price_str) < 0);

  // return the price
  return price;
}

