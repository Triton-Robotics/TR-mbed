#ifndef TR_EMBEDDED_MUTEX_H
#define TR_EMBEDDED_MUTEX_H

#include "mbed.h"
#include <cstdlib>
#include <queue>

/**
 * The TRMutex class defines the use of printing through Mutex and threading. This class contains 
 * methods that can be used in place of print statements for better efficiency.
*/
class TRMutex {

public:
    /**
     * Creates a thread which will be running alongside robot code, simultaneously 
     * printing data when methods from TRMutex are called.
    */
    Thread print_code_thread;


    /**
     * Constructor for creating a TRMutex object. By calling the constructor, 
     * the thread above automatically begins running. 
    */
    TRMutex(); 

    /**
     * Priority levels. If indicator is on FATAL, will print statements only with FATAL indicator. 
     * If indicator is on WARNING, will print statements with WARNING indicator or higher. 
     * If indicator is on DEFAULT, will print statements with DEFAULT warning indicator or higher
     * 
     * Hierachy: FATAL --> WARNING --> DEFAULT
     * 
     * DEFAULT is the default mode.
    */

    enum priorityLevels {
        FATAL = 1,
        wARNING = 2,
        DEFAULT = 3,
    };

    /**
     * int storing priorityIndicator
    */

    priorityLevels priorityIndicator;

    /**
     * Method to change priority level to determine which print statements will happen
     * @param enum of priority level desired
    */

    void updatePriority(priorityLevels desiredLevel);

    /**
     * Method to print integer arguments with no newline at the end.
     * @param integer to print 
     * @param enum of print priority level
    */
    void print(int integer, priorityLevels priority = DEFAULT);


    /**
     * Method to print string arguments with no newline at the end.
     * @param string to print
     * @param enum of print priority level
    */
    void print(char statement[], priorityLevels priority = DEFAULT);

    
    /**
     * Method to print integer arguments with a newline at the end.
     * @param integer to print
     * @param enum of print priority level
    */
    void println(int integer, priorityLevels priority = DEFAULT);


    /**
     * Method to print string arguments with a newline at the end.
     * @param string to print
     * @param enum of print priority level
    */
    void println(char statement[], priorityLevels priority = DEFAULT);


    /**
     * Method that mimics the printf() functionality
     * @param string that contains the formatting desired. (eg. %d, %s etc.) 
     * @param enum of print priority level
     * @param arguments to be formatted  
    */
    void printff(const char* format, priorityLevels priority = DEFAULT, ...);


    /**
     * Loop running within the thread called by the TRMutex constructor. 
    */
    static void loop();

};

#endif //TR_EMBEDDED_MUTEX_H