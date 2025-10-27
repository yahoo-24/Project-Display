#ifndef COLOURS_H
#define COLOURS_H

#include <string>
using namespace std;

namespace Colour {

    const string RESET = "\033[0m";
    const string CLEAR = "\033[2J\033[0;0H";
    
    // Text colours
    const string BLACK = "\033[30m";
    const string RED = "\033[31m";
    const string GREEN = "\033[32m";
    const string YELLOW = "\033[33m";
    const string BLUE = "\033[34m";
    const string MAGENTA = "\033[35m";
    const string CYAN = "\033[36m";
    const string WHITE = "\033[37m";

    // The same colour codes as above but used for modifying in the font() function
    const int BLACK_M = 30;
    const int RED_M = 31;
    const int GREEN_M = 32;
    const int YELLOW_M = 33;
    const int BLUE_M = 34;
    const int MAGENTA_M = 35;
    const int CYAN_M = 36;
    const int WHITE_M = 37;

    // Background colours
    const string BG_BLACK = "\033[40m";
    const string BG_RED = "\033[41m";
    const string BG_GREEN = "\033[42m";
    const string BG_YELLOW = "\033[43m";
    const string BG_BLUE = "\033[44m";
    const string BG_MAGENTA = "\033[45m";
    const string BG_CYAN = "\033[46m";
    const string BG_WHITE = "\033[47m";

    string font(int colour, bool Bold = false, bool Underlined = false, bool Slowblink = false);

    string RGB_colour(int red, int green, int blue, bool bg = false, bool bold = false, bool underlined = false, bool blink = false);
}
#endif
/*
int main() {
    cout << Colour::RED << "Red Text!" << Colour::RESET << endl;
    cout << Colour::font(Colour::GREEN_M, true, false, true) << "Bold Green Text!" << Colour::RESET << endl;
    cout << Colour::BG_YELLOW << Colour::BLACK << "Black Text - Yellow Background!" << Colour::RESET << endl;
    cout << Colour::CLEAR;
    cout << Colour::font(Colour::BLUE_M, true, true) << "Blue Bold Text!" << Colour::RESET << endl;
    cout << Colour::RGB(255, 200, 70) << "Hello" << Colour::RESET;
    
    return 0;
}
*/