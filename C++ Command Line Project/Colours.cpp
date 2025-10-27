#include <string>
using namespace std;

// https://stackoverflow.com/questions/4842424/list-of-ansi-color-escape-sequences

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

    // The same colour codes as above but used for modifying in the Font() function
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

    string font(int colour, bool Bold = false, bool Underlined = false, bool Slowblink = false) {
        /*
        Allows the user to choose from the predefined colours above and perform the 3 modifications: bold, underline, blink.
        */
        // Bold = '1', Underline = '4', Slowblink = '5'
        return "\033[" + 
                string(Bold ? "1;" : "") + 
                string(Underlined ? "4;" : "") +
                string(Slowblink ? "5;" : "") +
                to_string(colour) + "m";
    }

    string RGB_colour(int red, int green, int blue, bool bg = false, bool bold = false, bool underlined = false, bool blink = false){
        /*
        The function allows the programmer to input any value for the red, green and blue values to create any colour.
        The function also takes bg to indicate if it is a background colour. It also takes inputs for bold, underlined and blink.
        The effects of blink and bold are mutually exclusive so both cannot be applied.
        */
        return "\033[" + string(bold ? "1;" : "") + string(bg ? "48" : "38") + ";2;" + to_string(red) + ";" + 
                to_string(green) +
                ";" + to_string(blue) +
                string(underlined ? ";4" : "") + 
                string(blink ? ";5" : "") + "m";
    }
}