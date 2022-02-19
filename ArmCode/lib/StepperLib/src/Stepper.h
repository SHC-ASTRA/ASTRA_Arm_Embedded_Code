#include <Arduino.h>
#include <SPI.h>
#include <HighPowerStepperDriver.h>

#define CLOCKWISE 0
#define COUNTERCLOCKWISE 1

class Stepper
{
    private:
        // Pins to Drive the Stepper
        int pwmPin;
        int dirPin;
        int chipSelectPin;
        int homeSwitchPin;

        // Stepper 
        int steps;

        HighPowerStepperDriver sd;
};
