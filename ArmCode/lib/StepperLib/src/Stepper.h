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
        int steps = 0;
        int lowerLimit;
        int upperLimit;
        // 50 * 400 * 32 = 1 revolution of the small pulley
        // 2 revolution of small pulley = 1 revolution of big pulley

        HighPowerStepperDriver sd;

    public:
        Stepper(int pwmPin, int dirPin, 
                int chipSelectPin, int homeSwitchPin,
                int lowerLimit, int upperLimit);

        bool Initialize();
        void Enable() { sd.enableDriver(); }
        void Disable() { sd.disableDriver(); }

        void SetTarget();
        void SetTargetRate();
};
