#ifndef UTIL_H
#define UTIL_H

#include <regex>
#include <string>
#include <windows.h>
using namespace std;

// https://codereview.stackexchange.com/questions/162569/checking-if-each-char-in-a-string-is-a-decimal-digit
inline bool is_integer(string num) {
  return regex_match(num, regex("[+-]?[0-9]+"));
}

// https://www.geeksforgeeks.org/cpp-program-to-get-the-list-of-files-in-a-directory/
inline void list_files() {
    // This variable contains the path to the directory
    // The path should be given inside double quotes
    // Therefore, escaped the enclosing quotes using
    // backslash
    string path(
        "\"C:\\Users\\path\\to\\text\\files\\*.txt\"");
 
    // The command which would do the file listing
    // A space is added at the end of the command
    // intentionally to make space for the path
    string command("dir /b ");
 
    // The path is appended after the command string
    command.append(path);
 
    // The string is converted to const char * as system
    // function requires
    const char* final_command = command.c_str();
 
    // Sending the final command as the argument to the
    // system function
    system(final_command);
}


#endif
