#ifndef LOOPPATH_H
#define LOOPPATH_H

#include <string>
#include <vector>
#include <tuple>
#include <unordered_set>
#include <map>
#include "Nodes.h"
using namespace std;

namespace path_finder {
    // A function for reading in the file. Store the file in a 2d vector and generate the start and end nodes
    Starter read(string filename, bool bypass);

    // Printing the maze
    void print_maze(const vector<vector<char>> &maze);

    // Writing the solution to the file.
    void write(const vector<vector<char>> &maze, string filename);

    // Backtracking and finding the path.
    void backtracking(map<int, int> &path, int current, vector<vector<char>> &maze, string filename);

    // Makes a map of the City Block distances for all the nodes
    map<tuple<int, int>, int> city_block(Node end, int max_column_nums, int max_row_nums,
                                        vector<vector<char>> &maze);

    bool sorter(Node v1, Node v2); // Descending

    bool check_arrived(Node end, Node current);

    map<int, int> explore(Node &start, Node &end, vector<vector<char>> &maze, int &i, int &j, vector<Node> to_explore,
                unordered_set<int> &explored, map<tuple<int, int>, int> &distances, map<int, int> &relationship);

    bool check_valid(Starter starter);

    // A function to initiate the Explore function
    bool start(string filename);

    void execute();
}

#endif