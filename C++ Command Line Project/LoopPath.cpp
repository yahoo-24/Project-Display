
#include <iostream>
#include <string>
#include <vector>
#include <tuple>
#include <algorithm>
#include <fstream>
#include <unordered_set>
#include <map>
#include <regex>
#include "Colours.h"
#include "Nodes.h"
#include "RandomMazeGen.h"
#include "LoopPath.h"
#include "util.h"
using namespace std;

namespace path_finder {
    // A function for reading in the file. Store the file in a 2d vector and generate the start and end nodes
    Starter read(string filename, bool bypass) {
        ifstream file(filename);
        string line;
        Starter problem{{}, Node(), Node()};
        vector<char> row = {};
        // Initialise x and y for the coordinates. count_s and count_e checks that we have 1 end and 1 start
        int x {0}, y {0}, count_e {0}, count_s {0};
        int row_size;

        if (file.is_open()) {
            while (getline(file, line)) {
                // Erase \n
                line.erase(remove(line.begin(), line.end(), '\n'), line.end());            

                // Check row size are equal. Do not check for the first row as row size has not been set
                if (y != 0 && row_size != line.size()) {
                    cout << Colour::BG_RED << "All rows must have the same size!" << Colour::RESET << endl;
                    return Starter{{}, Node(), Node()};
                }

                for (char letter : line) {
                    // Add the letter to the row
                    row.push_back(letter);
                    // Check if it is a start or end
                    if (toupper(letter) == 'S') {
                        count_s += 1;

                        if (count_s > 1 && !bypass) {
                            cout << Colour::BG_RED << "Only 1 start is allowed!" << Colour::RESET << endl;
                            return Starter{{}, Node(), Node()};
                        }
                        // The first loop in explore will do 0 * j - 1 = -1 which is added to the map
                        problem.start = Node(make_tuple(x, y), 0, 0, make_tuple(-1, 0));
                    } else if (toupper(letter) == 'E') {
                        count_e += 1;

                        if (count_e > 1 && !bypass) {
                            cout << Colour::BG_RED << "Only 1 end is allowed!" << Colour::RESET << endl;
                            return Starter{{}, Node(), Node()};
                        }

                        problem.end = Node(make_tuple(x, y), 0, 0, make_tuple(-1, -1));
                    }
                    x++;
                }
                // Add the row to the vector
                problem.maze.push_back(row);
                // Reset the row and x. Append y for the new row. The 1st value of x will be the row size.
                row = {};
                y++;
                row_size = x;
                x = 0;
            }
            file.close();
            if (count_e == 0 && !bypass){
                cout << Colour::BG_RED << "There must be an end point!" << Colour::RESET << endl;
                return Starter{{}, Node(), Node()};
            } else if (count_s == 0 && !bypass) {
                cout << Colour::BG_RED << "There must be a starting point!" << Colour::RESET << endl;
                return Starter{{}, Node(), Node()};
            }
            return problem;
        } else {
            cerr << Colour::BG_RED << "Unable to open file" << Colour::RESET << endl;
            return Starter{{}, Node(), Node()};
        }
    }

    // Printing the maze
    void print_maze(const vector<vector<char>> &maze) {
        // Print the maze by using the characters below with different colours
        int j = maze[0].size();
        for (int x = 0; x < maze.size(); x++){
            for (int y = 0; y < j; y++){
                if (maze[x][y] == 'S'){
                    // The start
                    cout << Colour::RED << "██" << Colour::RESET;
                } else if (maze[x][y] == 'W'){
                    // The walls
                    cout << Colour::RGB_colour(72, 72, 72) << "██" << Colour::RESET;
                } else if (maze[x][y] == 'E'){
                    // The end
                    cout << Colour::BLUE << "██" << Colour::RESET;
                } else if (maze[x][y] == '-'){
                    // The solution path
                    cout << Colour::YELLOW << "██" << Colour::RESET;
                } else if (maze[x][y] >= '1' && maze[x][y] <= '9'){
                    // Weights are from 1 to 9
                    switch (maze[x][y])
                    {
                    case '1':
                        cout << Colour::RGB_colour(144, 238, 144) << "██" << Colour::RESET;
                        break;
                    case '2':
                        cout << Colour::RGB_colour(124, 252, 0) << "██" << Colour::RESET;
                        break;
                    case '3':
                        cout << Colour::RGB_colour(52, 205, 52) << "██" << Colour::RESET;
                        break;
                    case '4':
                        cout << Colour::RGB_colour(41, 150, 23) << "██" << Colour::RESET;
                        break;
                    case '5':
                        cout << Colour::RGB_colour(19, 136, 8) << "██" << Colour::RESET;
                        break;
                    case '6':
                        cout << Colour::RGB_colour(0, 92, 41) << "██" << Colour::RESET;
                        break;
                    case '7':
                        cout << Colour::RGB_colour(0, 66, 37) << "██" << Colour::RESET;
                        break;
                    case '8':
                        cout << Colour::RGB_colour(18, 53, 36) << "██" << Colour::RESET;
                        break;
                    case '9':
                        cout << Colour::RGB_colour(1, 50, 32) << "██" << Colour::RESET;
                        break;
                    default:
                        break;
                    }
                } else {
                    // All other traversable nodes including weight 0
                    cout << Colour::WHITE << "██" << Colour::RESET;
                }
            }
            cout << endl;
        }
    }

    // Writing the solution to the file.
    void write(const vector<vector<char>> &maze, string filename) {
        // Make a new filename
        filename = "Solution_" + filename;
        ofstream file;
        string line = "";
        // Open the file
        file.open(filename);
        for (vector<char> row : maze){
            for (char letter : row){
                // Add the letter to the line
                line += letter;
            }
            // Add the line to the file
            file << line << endl;
            // Restart the line into an empty string
            line = "";
        }
        file.close();
    }

    // Backtracking and finding the path.
    void backtracking(map<int, int> &path, int current, vector<vector<char>> &maze, string filename) {
        int j = maze[0].size();
        // The start node has a parent of -1 so that is where this terminates
        while (current != -1) {
            current = path[current];
            // If statement so when we get -1 we do not access a random location in memory
            // We also do not want to change the start into '-'
            if (current == -1){
                break;
            } else if (maze[current / j][current % j] != 'S'){
                maze[current / j][current % j] = '-';
            }
        }
        // Print the maze
        print_maze(maze);
        // Write the solution to a text file
        write(maze, filename);
    }

    // Makes a map of the City Block distances for all the nodes
    map<tuple<int, int>, int> city_block(Node end, int max_column_nums, int max_row_nums,
                                        vector<vector<char>> &maze) {
        // Calculates the City Block distance which is the absolute difference distace in the x and y directions
        map<tuple<int, int>, int> distances;
        for (int j = 0; j < max_row_nums; j++) {
            for (int i = 0; i < max_column_nums; i++) {
                // get<0> -> Column number  get<1> -> row number
                distances[make_tuple(i, j)] = abs(get<0>(end.get_node()) - i) + abs(get<1>(end.get_node()) - j);
            }
        }

        return distances;
    }

    bool sorter(Node v1, Node v2) { return v1.get_heuristic() > v2.get_heuristic(); } // Descending

    bool check_arrived(Node end, Node current) {
        // Both coordinates should align
        if (get<0>(end.get_node()) == get<0>(current.get_node()) && 
            get<1>(end.get_node()) == get<1>(current.get_node())) {
            return true;
        } else {
            return false;
        }
    }

    map<int, int> explore(Node &start, Node &end, vector<vector<char>> &maze, int &i, int &j, vector<Node> to_explore,
                unordered_set<int> &explored, map<tuple<int, int>, int> &distances, map<int, int> &relationship) {
        /*
        The function does the following:
            - Check that there is still some nodes to explore. If not then return an empty map.
            - Takes the last node in the vector and deletes it
            - Extracts the column and row of the node
            - Uses the row and column to see if the vector has already been explored
            - If so then ignore this node and go to next loop otherwise add it and its parent to the relationship map
            - Check if this is the end point
            - Insert the node to explored
            - Check all 4 neighbours and add the new ones to the to_explore vector with their steps and heuristic
            - Repeat loop.
        */
        // Keep running while there are nodes in the vector
        while (to_explore.size() != 0)
        {
            // Extract the first Node
            Node first = to_explore[to_explore.size() - 1];
            // Delete this node from the to_explore list
            to_explore.erase(to_explore.end());

            // Extract the x and y coordinates to be used in the if statements so we do not have to write this 4 times
            int column = get<0>(first.get_node());
            int row = get<1>(first.get_node());

            // Check that the node has not been explored before, if so then move on
            if (explored.find(column + j * row) != explored.end()) {
                continue;
            }

            // Add the node and its parent node to the map
            relationship[column + j * row] = get<1>(first.get_parent()) * j + get<0>(first.get_parent());

            // Check that this is not the end node
            if (check_arrived(end, first)) {
                return relationship;
            }

            // Put the first element to a list of explored nodes
            explored.insert(row*j + column);

            // For use later to find the heuristics for each node.
            // Steps would be the same for all of them if they do not have a weight. This is the default value of step
            int heuristic;
            int step = first.get_steps() + 1;


            // Check all 4 sides and see if there is no wall and it is not outside the array limits
            // Check that the node has not been explored
            if (column >= 1 && explored.find((column - 1) + j * row) == explored.end()) {
                if (maze[row][column - 1] != 'W'){
                    // Heuristic = Distance + Steps
                    // See if the value in the maze is weighted otherwise take the default value of step
                    if (maze[row][column - 1] >= '0' && maze[row][column - 1] <= '9'){
                        step = first.get_steps() + maze[row][column - 1] - '0';
                    }
                    heuristic = distances[make_tuple(column - 1, row)] + step;
                    // Add to the vector of nodes to be explored
                    to_explore.push_back(Node(make_tuple(column - 1, row), heuristic, step, make_tuple(column, row)));
                }
            }

            if (column < j - 1 && explored.find((column + 1) + j * row) == explored.end()) {
                if (maze[row][column + 1] != 'W'){
                    if (maze[row][column + 1] >= '0' && maze[row][column + 1] <= '9'){
                        step = first.get_steps() + maze[row][column + 1] - '0';
                    }
                    heuristic = distances[make_tuple(column + 1, row)] + step;
                    to_explore.push_back(Node(make_tuple(column + 1, row), heuristic, step, make_tuple(column, row)));
                }
            }

            if (row >= 1 && explored.find(column + j * (row - 1)) == explored.end()) {
                if (maze[row - 1][column] != 'W'){
                    if (maze[row - 1][column] >= '0' && maze[row - 1][column] <= '9'){
                        step = first.get_steps() + maze[row - 1][column] - '0';
                    }
                    heuristic = distances[make_tuple(column, row - 1)] + step;
                    to_explore.push_back(Node(make_tuple(column, row - 1), heuristic, step, make_tuple(column, row)));
                }
            }

            if (row < i - 1 && explored.find(column + j * (row + 1)) == explored.end()) {
                if (maze[row + 1][column] != 'W'){
                    if (maze[row + 1][column] >= '0' && maze[row + 1][column] <= '9'){
                        step = first.get_steps() + maze[row + 1][column] - '0';
                    }
                    heuristic = distances[make_tuple(column, row + 1)] + step;
                    to_explore.push_back(Node(make_tuple(column, row + 1), heuristic, step, make_tuple(column, row)));
                }
            }
            // Smallest heuristic goes first
            sort(to_explore.begin(), to_explore.end(), sorter);
        }
        // No solution found
        return map<int, int> {};
    }

    bool check_valid(Starter starter){
        // Empty vector indicates invalid file
        if (starter.maze.size() == 0){
            return false;
        } else {
            return true;
        }
    }

    // A function to initiate the explore function
    bool start(string filename) {
        /*
        The function starts the solving process. It does the following:
            - Check that the file is valid
            - Read the file
            - Extract the start, end, maze and its dimensions
            - Warn the user if the file is too large
            - Prepare the distances and relationship maps as well as the to_explore vector and explored set
            - Call the explore fuction
            - Call backtracking to return the solution
        */
        // read the file
        Starter starter = read(filename, 0);
        if (check_valid(starter)){
            // Extract the maze, start and end from the struct
            vector<vector<char>> maze = starter.maze;
            Node start = starter.start;
            Node end = starter.end;

            // Find the row and column size (i = row, j = column). Give the user a warning if the file is large.
            int i = maze.size();
            int j = maze[0].size();
            if (i * j >= 150 * 150){
                cout << "File is too large! Finding the solution may take a long time! To continue press 1: ";
                string check;
                getline(cin, check);
                if (check != "1"){
                    return false;
                }
            }

            // Initialise the vector to_explore with the start node to explore
            vector<Node> to_explore = {start};
            unordered_set<int> explored;
            // Make the distances map
            map<tuple<int, int>, int> distances = city_block(end, j, i, maze);
            map<int, int> relationship;
            // Add the parent of the start node which is -1
            relationship[get<0>(start.get_node()) + j * get<1>(start.get_node())] = -1;

            // Initiate the explore algorithm
            map<int, int> path = explore(start, end, maze, i, j, to_explore, explored, distances, relationship);
            // If empty then there was no solution, otherwise call the backtracking to print the solution
            if (path.empty()){
                cout << "There is no solution" << endl;
            } else {
                backtracking(path, get<0>(end.get_node()) + j * get<1>(end.get_node()), maze, filename);
            }

            return true;
        } else {
            return false;
        }
    }

    void execute() {
        cout << Colour::CLEAR;
        cout << "\n-----------" << Colour::RGB_colour(0, 128, 128, 1, 0, 0) << Colour::BLACK << " Path Planning " << Colour::RESET << "---------------\n";
        cout << "|\t\t\t\t\t|\n";
        cout << "|" << Colour::RGB_colour(0, 128, 128, 1) << Colour::BLACK << "\t1. Maze Solver\t\t\t" << Colour::RESET << "|\n";
        cout << "|" << Colour::RGB_colour(0, 128, 128, 1) << Colour::BLACK <<"\t2. Print Maze\t\t\t" << Colour::RESET << "|\n";
        cout << "|" << Colour::RGB_colour(0, 128, 128, 1) << Colour::BLACK << "\t3. Random Maze Generator\t" << Colour::RESET << "|\n";
        cout << "|\t" << Colour::BG_RED <<"4. Exit"<< Colour::RESET <<"\t\t\t\t|\n";
        cout << "|\t\t\t\t\t|\n";
        cout << "-----------------------------------------\n";

        bool invalid = true;
        string choice;
        cout << "Enter you choice: ";
        // Check value is between 1 and 4
        while (invalid){
            getline(cin, choice);
            if (choice > "4" || choice < "1"){
                cout << "Enter a valid choice: ";
            } else {
                invalid = false;
            }
        }
        // Item 1
        if (choice == "1"){
            // List all the text files in the directory
            list_files();
            cout << "Enter the name of the file that you want to explore: ";
            string filename;
            getline(cin, filename);

            // Check file is valid
            while(!start(filename)){
                cout << "Enter a valid file or c to cancel: ";
                getline(cin, filename);
                // To cancel operation
                if (filename == "c") {
                    break;
                }
            }
            
        } else if (choice == "2") { 
            // Item 2
            list_files();
            cout << "Enter the name of the file that you want to explore: ";
            string filename;
            getline(cin, filename);
            // Call the read function to read the file.
            Starter temp = read(filename, 1);

            // Check that the file exists
            while(!check_valid(temp)){
                cout << "Enter a valid file or c to cancel: ";
                getline(cin, filename);
                if (filename == "c") {
                    break;
                }
                temp = read(filename, 1);
            }

            // Print the maze if cancel operation was not called
            if (filename != "c") {
                print_maze(temp.maze);
            }
        } else if (choice == "3") {
            string row, column;
            cout << "Enter the row dimension: ";
            getline(cin, row);
            // Check that the row is a number
            while(!regex_match(row, regex("[+-]?[0-9]+"))) {
                cout << "Enter a valid Row: ";
                getline(cin, row);
            }
            // Row has to be between 1 and 1000
            while(stoi(row) <= 0 || stoi(row) > 1000){
                cout << "Enter a valid Row: ";
                getline(cin, row);
            }

            cout << "Enter the column dimension: ";
            getline(cin, column);
            // Check column is a number
            while(!regex_match(column, regex("[+-]?[0-9]+"))) {
                cout << "Enter a valid Column: ";
                getline(cin, column);
            }
            // Column has to be between 1 and 1000 unless row is 1 then it has to be between 2 and 1000
            while(stoi(column) <= 0 || (stoi(row) == 1 && stoi(column) <= 1) || stoi(column) > 1000){
                cout << "Enter a valid Column: ";
                getline(cin, column);
            }
            // Generate a random maze
            generate_maze(stoi(row), stoi(column));
            cout << "RandomMaze.txt generated" << endl;
        }

        if (choice != "4") {
            cout << "Enter any value to return to the menu ";
            getline(cin, choice);
            // Back to the beginning of the function
            execute();
        }
    }
}
