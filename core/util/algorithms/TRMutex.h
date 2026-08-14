#pragma once

#include <zephyr/kernel.h>
#include <cstdlib>
#include <cstdarg>
#include <queue>

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
    WARNING = 2,
    DEFAULT = 3,
};

/**
 * The TRMutex class defines the use of printing through Mutex and threading. This class contains 
 * methods that can be used in place of print statements for better efficiency.
*/
class TRMutex {

public:

    /**
     * Constructor for creating a TRMutex object. By calling the constructor, 
     * the thread above automatically begins running. 
    */
    TRMutex(); 

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
    void print(int integer, priorityLevels priority);


    /**
     * Method to print string arguments with no newline at the end.
     * @param string to print
     * @param enum of print priority level
    */
    void print(char statement[], priorityLevels priority);

    
    /**
     * Method to print integer arguments with a newline at the end.
     * @param integer to print
     * @param enum of print priority level
    */
    void println(int integer, priorityLevels priority);


    /**
     * Method to print string arguments with a newline at the end.
     * @param string to print
     * @param enum of print priority level
    */
    void println(char statement[], priorityLevels priority);


    /**
     * Method that mimics the printf() functionality
     * @param string that contains the formatting desired. (eg. %d, %s etc.) 
     * @param enum of print priority level
     * @param arguments to be formatted  
    */
    void printff(const char* format, priorityLevels priority, ...);


    /**
     * Loop running within the thread called by the TRMutex constructor. 
    */
    static void loop();

private:

    static constexpr size_t STACK_SIZE = 1024;
    static constexpr int THREAD_PRIORITY = 7; // Lower prio than CONFIG_MAIN_THREAD_PRIORITY
    // Unlike mbedOS, higher number = less prio

    /**
     * Loop running within the thread started by the TRMutex constructor.
     * Zephyr thread entry points must match k_thread_entry_t: void(*)(void*, void*, void*).
    */

    static void loop(void* arg1, void* arg2, void* arg3);
 
    K_THREAD_STACK_MEMBER(thread_stack, STACK_SIZE);
    k_thread thread_data;

};

