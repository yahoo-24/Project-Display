#ifndef RANDOMMAZEGEN_H
#define RANDOMMAZEGEN_H

// https://www.w3schools.com/cpp/cpp_howto_random_number.asp

#include <iostream>
#include <cstdlib> // rand(), srand()
#include <ctime>   // time()
#include <string>
#include <fstream>
using namespace std;

int generate_random_number() {
    // Generate a random number 0 to 10
    return rand() % 11;
}

void generate_maze(int max_rows, int max_columns) {
    // Random number generator seeded with the current time
    srand(time(0));

    int value {0};
    string line = "S";
    ofstream file;
    file.open("RandomMaze.txt");
    for (int i = 0; i < max_rows; i++) {
        for (int j = 0; j < max_columns; j++) {
            if (i == 0 && j == 0) {
                continue;
            } else if (i == max_rows - 1 && j == max_columns - 1) {
                line += "E";
                continue;
            }
            value = generate_random_number();
            if (value == 10){
                line += 'W';
            } else {
                line += to_string(value);
            }
        }
        file << line << endl;
        line = "";
    }
    file.close();
}

#endif