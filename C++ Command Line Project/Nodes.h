#ifndef NODES_H
#define NODES_H

#include <tuple>
#include <vector>
using namespace std;

// A class to keep track of all the nodes in the map. This may be changed to struct if no functions are added
class Node {
    public:
        Node(tuple<int, int> coordinates, int h, int step, tuple<int, int> parent){
            _node = coordinates;
            _heuristic = h;
            _steps = step;
            _parent = parent;
        }

        Node() {}

        tuple<int, int> get_node() { return _node; }

        int get_heuristic() { return _heuristic; }

        int get_steps() { return _steps; }

        tuple<int, int> get_parent() { return _parent; }

    private:
        tuple<int, int> _node; // Co-ordinates of the node (column number, row number)
        int _heuristic; // A weight associated with the node
        int _steps; // Stores the steps needed to get to this node
        tuple<int, int> _parent;
};

struct Starter {
    vector<vector<char>> maze;
    Node start;
    Node end;
};

#endif
