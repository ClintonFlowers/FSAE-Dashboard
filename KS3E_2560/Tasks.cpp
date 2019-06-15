#include "Tasker.h"

class PrintWew : public TaskClass {
public:
    void execute() override {
//        Serial.println("wew");
    }

private:
    volatile unsigned int getExecutionDelay() override { return 5000000; }
};

class PrintLad : public TaskClass {
public:
    void execute() override {
//        Serial.println("lad");
    }

private:
    volatile unsigned int getExecutionDelay() override { return 6000100; }
};
