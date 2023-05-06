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
     * Method to print integer arguments with no newline at the end.
     * @param integer to print 
    */
    void print(int integer);


    /**
     * Method to print string arguments with no newline at the end.
     * @param string to print
    */
    void print(char statement[]);

    
    /**
     * Method to print integer arguments with a newline at the end.
     * @param integer to print
    */
    void println(int integer);


    /**
     * Method to print string arguments with a newline at the end.
     * @param string to print
    */
    void println(char statement[]);


    /**
     * Method that mimics the printf() functionality
     * @param string that contains the formatting desired. (eg. %d, %s etc.) 
     * @param arguments to be formatted  
    */
    void printff(const char* format, ...);


    /**
     * Loop running within the thread called by the TRMutex constructor. 
    */
    static void loop();

};

#endif //TR_EMBEDDED_MUTEX_H