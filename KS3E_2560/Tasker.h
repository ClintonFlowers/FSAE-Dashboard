/*
 * General-purpose task scheduler class
 * Clinton Flowers, 2019-6-2
 */

#ifndef TASKER_H
#define TASKER_H

#include <Arduino.h>

#define TASKS_ARRAY_SIZE 2 // This must match the number of tasks that are defined in the init method. Ideally, a linked list would obviate the need for this, but that is added complexity. 
#define ADC_READ_RESOLUTION 10   // For Teensy. Default is 10. Above 12, read time increases drastically.


class TaskClass {
protected:
    friend class Tasker;

    virtual void execute() = 0;

    bool shouldExecute(volatile unsigned long currentTime) {
        if (currentTime > lastExecutionTime + getExecutionDelay()) {
            lastExecutionTime = currentTime;
            return true;
        } else {
            return false;
        }
    };

    TaskClass() = default;;
    ~TaskClass() = default;;

private:
    virtual volatile unsigned int getExecutionDelay() = 0;
    volatile unsigned int lastExecutionTime = 0;
};

class Tasker {
public:
    void init();

    void taskLoop();   // Call this in the main loop to evaluate and execute tasks

    // Debug information
    long getLoopsCompleted();   // Not idempotent
    int getTasksSize(); // TODO: Unused?

    // Utility methods
    static float mapf(float x, float in_min, float in_max, float out_min, float out_max);
    static bool strContains(const String & superset, const String & subset);
    
protected:
    // Friend classes (usually VCUTask classes) which can access the protected VCU variables here
    //friend class DoSpecificDebugThing;

    // Debug variables
    int loopsCompleted{};
    
    // Other operational variables
    TaskClass **tasks = new TaskClass *[TASKS_ARRAY_SIZE];
    int maxAdcValue{};
};

#endif
