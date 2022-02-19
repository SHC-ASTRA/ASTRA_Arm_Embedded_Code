#include "Stepper.h"

Stepper::Stepper(int pwmPin, int dirPin,
    int chipSelectPin, int homeSwitchPin,
    int lowerLimit, int upperLimit)
{
    this->pwmPin = pwmPin;
    this->dirPin = dirPin;
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

    return sd.verifySettings();
}
