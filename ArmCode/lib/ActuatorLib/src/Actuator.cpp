#include "Actuator.h"

Actuator::Actuator(int pwmPin, int dirPin, int encoderPinA,
        int encoderPinB, int lowerLimit, int upperLimit,
        float baseLength, float sideALength, float sideBLength, 
        float dxde)
{
        this->pwmPin = pwmPin;
        this->dirPin = dirPin;
        this->encoderPinA = encoderPinA;
        this->encoderPinB = encoderPinB;
        this->lowerLimit = lowerLimit;
        this->upperLimit = upperLimit;
        this->baseLength = baseLength;
        this->sideALength = sideALength;
        this->sideBLength = sideBLength;
        this->dxde = dxde;
}

void Actuator::Initalize()
{
        // Initialize Control Pins
        pinMode(pwmPin, OUTPUT);
        pinMode(dirPin, OUTPUT);
        SetSpeed(0);
        SetDirection(EXTEND);

        // Initialize Encoder
        encoder = new Encoder(encoderPinA, encoderPinB);
        encoder->readAndReset();

        ResetBuffers();
}

void Actuator::ResetBuffers()
{
        // Clear buffers
        encoderBuffer.clear();
        timingBuffer.clear();

        encoderBuffer.unshift(encoder->read());
        timingBuffer.unshift(millis());
}

void Actuator::Update()
{
        if (BufferReadyForUpdate())
        {
                // Add to buffer
                // Recalculate extension and angle
                // Recalculate rate of change
        }

        // Check state
        // If target mode, then step in the direction that's towards the target
}

void Actuator::Home(bool retract)
{
        // Set the right direction for homing
        if (retract)
                SetDirection(RETRACT);
        else
                SetDirection(EXTEND);
        
        // Log the start position, then power the actuator
        int step = encoder->read();
        SetSpeed(HOMING_SPEED);

        // Check every 100ms whether the actuator has moved
        // If not, break out of the loop
        do
        {
                delay(100);
                int nextStep = encoder->read();

                if (nextStep == step)
                        break;
                else
                        step = nextStep;
        } while (1);
        
        // Power down actuator
        SetSpeed(0);

        // Reset encoder and buffers
        encoder->readAndReset();
        ResetBuffers();
}