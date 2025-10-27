# C++ Command Line Project

## Contributors
Battery_Items.cpp, Battery_Items.h, Robot.cpp, Robot.h authored by ***Wassim Bouhamadi***  
Control.cpp, Control.h, PID.h authored by ***Abubakar Zareef Bakari***  
Colours.cpp, Colours.h, LoopPath.cpp, LoopPath.h, Nodes.h, RandomMazeGen.h, util.h authored by ***Yahia Abuhelweh***

## About The Project
The project was made with Abubakar Bakari and Wassim Bouhamadi as part of the ELEC2645 module at the University of Leeds. The goal of the project is to make a CLI menu system that performs technical operations. In this project, the group attempts to solve some of the common challenges in warehouse magagement and robots including path planning, control, database management. The project contains the following menu items:
- ***Path Planning***:
  - ***Maze Solver***: The user can generate a text file within the same directory of a maze that he or she wants to solve. The format is to have an *mxn* matrix of '*.*' and '*W*' representing a valid path and walls respectively. The file must contain one '*S*' and '*E*' for start and end respectively. The user may want to replace '*.*' with a number between 0 and 9 inclusive to give each node a different weight. The Maze Solver displays all the available text files in the directory, finds a solution, if exists, from start to finish for the inputted text file, prints the map and solution in the terminal and generates a text file that represents the path with a '*-*'.
  - ***Print Maze***: Allows the user to print a maze stored on a text file. The user can use this feature to print a maze again after solving it or to aid in generating a maze as making a maze from text file may be difficult hence this feature lets the user have a better look at the maze.
  -  ***Random Maze Generator***: Allows the user to generate an *mxn* random maze text file which can be used as a template or for testing the Maze Solver.
- ***Orders and Warehouse Management***:
  - ***Fullfill Order***: The user places the order in the csv file specifying the item and quantity. The csv file containing the warehouse items and quantities is updated. The user is notified whether the item exists or is in stock, and a price is printed to the terminal.
  - ***Add New Item***: Adds an item, its quantity and price to the warehouse csv file.
  - ***Delete Item***: Deletes an existing item in the warehouse csv file.
  - ***Update Item Quantity***: Updates the quantity of an existing item in the warehouse csv file.
  - ***Update Item Price***: Updates the price of an existing item in the warehouse csv file.
- ***PID Controller***:
  - ***Set Desired Speed***: The user sets the speed that they are trying to achieve in *m/s*.
  - ***Tune PID***: The PID controller values of *Ki*, *Kp*, *Kd* and sample time (*dtime*) can be modified in this category.
  - ***Run PID***: The simulation is run trying to achieve the desired speed set by the user within a specified number of steps whilst constantly providing feedback about the current speed and control signal.
  - ***View Current Settings***: Displays the *Kp*, *Ki*, *Kd*, sample time, desired speed, maximum output, minimum output, mass, friction coefficient and steps.
  - ***Change Simulation Settings***: Allows the user to change the values of *Kp*, *Ki*, *Kd*, sample time, desired speed, maximum output, minimum output, mass, friction coefficient and steps.
- ***Battery Management***:
  - ***Display Battery***: Presents a visual display of the battery.
  - ***Charge Battery***: Presents a visual display of the battery charging.
 
 ***Discharge Battery***: Presents a visual display of the battery discharging.

## Built With
<img src="https://upload.wikimedia.org/wikipedia/commons/1/18/ISO_C%2B%2B_Logo.svg" alt="Alt text" width="10%" height="10%">

## 🚀 Getting Started

Follow these steps to clone the repository, build the project, and run the executable.

### 1. Clone the repository
```bash
git clone https://github.com/yahoo-24/Project-Display.git
cd "Project-Display/C++ Command Line Project"
```
### 2. Build the project
Make sure you have make and a C++ compiler (e.g. g++) installed.
Change the path in util.h to where the text files you want to use are present.
Then run:
```bash
make
```
### 3. Run the programme
```bash
main.exe
```
or
```bash
./main.exe
```
or
```bash
./main
```
### Notes
- You may need to adjust the Makefile if your compiler or file structure is different.
- If you encounter permission issues on Linux/macOS, you can make the executable runnable with:
```
chmod +x main.exe
```
