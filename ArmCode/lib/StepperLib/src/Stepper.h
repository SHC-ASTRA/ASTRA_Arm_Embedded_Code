#include <Arduino.h>
#include <SPI.h>
#include <HighPowerStepperDriver.h>

#define CLOCKWISE 1
#define COUNTERCLOCKWISE -1

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
        float lowerLimit;
        float upperLimit;
        int direction;
        int timeLastStep;
        int timeStep;
        // 50 * 400 * 32 = 1 revolution of the small pulley
        // 2 revolution of small pulley = 1 revolution of big pulley
        // 50 * 400 * 32 * 2 = 360 degrees
        float stepsPerDegree = -(50 * 400 * 32) / 360.0;
        float targetRate;

        HighPowerStepperDriver sd;

    public:
        Stepper(int chipSelectPin, int homeSwitchPin,
                int lowerLimit, int upperLimit);

        bool Initialize();
        void Enable() { sd.enableDriver(); }
        void Disable() { sd.disableDriver(); }

        void Update();

        void SetDirection(int direction);
        void SetTarget(float targetAngle);
        void SetTargetRate(float targetRate);

        float GetRotation();
        bool IsActive() { return targetRate != 0; }
        float GetAngularRate() { return targetRate; } // If we implement accel, make this smarter
};
