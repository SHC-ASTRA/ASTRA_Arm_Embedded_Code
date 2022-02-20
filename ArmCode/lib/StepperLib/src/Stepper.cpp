#include "Stepper.h"

Stepper::Stepper(int chipSelectPin, int homeSwitchPin,
    int lowerLimit, int upperLimit)
{
    this->chipSelectPin = chipSelectPin;
    this->homeSwitchPin = homeSwitchPin;
    this->lowerLimit = lowerLimit;
    this->upperLimit = upperLimit;
}

bool Stepper::Initialize()
{
    SPI.begin();
    sd.setChipSelectPin(chipSelectPin);

    // Reset the driver to its default settings and clear latched status
    // conditions.
    sd.resetSettings();
    sd.clearStatus();

    // Select auto mixed decay.  TI's DRV8711 documentation recommends this mode
    // for most applications, and we find that it usually works well.
    sd.setDecayMode(HPSDDecayMode::AutoMixed);

    // Set the current limit. You should change the number here to an appropriate
    // value for your particular system.
    sd.setCurrentMilliamps36v4(1000);

    // Set the number of microsteps that correspond to one full step.
    sd.setStepMode(HPSDStepMode::MicroStep32);

    SetTargetRate(0);

    return sd.verifySettings();
}

void Stepper::Update()
{
    if (micros() - timeLastStep > timeStep)
    {
        sd.step();
        steps += direction;
        timeLastStep = micros();
    }
}

void Stepper::SetDirection(int direction)
{
    this->direction = direction;

    if (direction == COUNTERCLOCKWISE)
    {
        sd.setDirection(0);
    } else if (direction == CLOCKWISE)
    {
        sd.setDirection(1);
    }
}

float Stepper::GetRotation()
{
    return steps / stepsPerDegree;
}

void Stepper::SetTargetRate(float targetRate)
{
    this->targetRate = targetRate;

    if (targetRate > 0)
        SetDirection(COUNTERCLOCKWISE);
    if (targetRate < 0)
        SetDirection(CLOCKWISE);
    
    if (targetRate == 0)
        timeStep = INT32_MAX;
    else
    {
        // degrees / second
        // steps / degrees
        // steps / second
        timeStep = abs((1.0 / (targetRate * stepsPerDegree)) * 1000000);
    }
}