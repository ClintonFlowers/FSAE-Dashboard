#include "Tasker.h"
#include "Tasks.cpp"

void Tasker::init() {
    // We can set pin modes here
    
    if (!Serial) {
        Serial.begin(115200); // We do not wait for Serial to be initialized
    }

    // Declare tasks which should be executed
    tasks[0] = new PrintWew();
    tasks[1] = new PrintLad();

    // Default variable initialization
    // Sensor input pins
    this->maxAdcValue = pow(2, ADC_READ_RESOLUTION);
    this->loopsCompleted = 0;
}

void Tasker::taskLoop() {
    for (int i = 0; i < TASKS_ARRAY_SIZE; i++) {
        volatile long currentTime = micros(); // Putting this inside the for loop may make sense
        if (tasks[i]->shouldExecute(currentTime)) {
            tasks[i]->execute();
        }
    }
    this->loopsCompleted++;
}

long Tasker::getLoopsCompleted() {
    long result = this->loopsCompleted;
    this->loopsCompleted = 0;
    return result;
}

float Tasker::mapf(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

bool Tasker::strContains(const String &outer, const String &inner) {
    return outer.indexOf(inner) >= 0;
}
