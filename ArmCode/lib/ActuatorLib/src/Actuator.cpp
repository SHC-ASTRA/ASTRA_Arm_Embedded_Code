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

        controlMode = idle;
}

void Actuator::ResetBuffers()
{
        // Clear buffers
        encoderBuffer.clear();
        timingBuffer.clear();

        AddToBuffer();
}

void Actuator::AddToBuffer()
{
        encoderBuffer.unshift(encoder->read());
        timingBuffer.unshift(millis());
}

float Actuator::CalculateExtension(int encoderReading)
{
        return baseLength + dxde * encoderReading;
}

float Actuator::CalculateAngle(float actuatorExtension)
{
        return (180.f / M_PI) * acosf((powf(actuatorExtension, 2) - (powf(sideALength, 2) + powf(sideBLength, 2))) / (-2*sideALength*sideBLength));
}

void Actuator::CalculateAngularRate()
{
        int dT = timingBuffer.first() - timingBuffer.last();
        
        float oldAngle = CalculateAngle(
                CalculateExtension(encoderBuffer.last())
        );

        angularRateOfChange = (angle - oldAngle) / (dT) * 1000.0;
}

void Actuator::Update()
{
        if (BufferReadyForUpdate())
        {
                AddToBuffer();

                // Recalculate extension and angle
                extension = CalculateExtension(encoderBuffer.first());
                angle = CalculateAngle(extension);

                // Recalculate rate of change
                CalculateAngularRate();
        }

        // Check state
        // If target mode, then step in the direction that's towards the target
        if (controlMode == target)
        {
                int step = encoder->read();
                
                if (step == actuatorTarget)
                {
                        actuatorTarget = 0;
                        targetInitialStep = 0;
                        controlMode = idle;
                        SetSpeed(0);
                        return;
                }

                // Quadratic Formula allows for the actuator to ease in to approaching the target
                int ease = -((step - targetInitialStep)*(step - actuatorTarget)) / abs(actuatorTarget - targetInitialStep);
                int speed = min(255, abs(ease) + 100); 

                SetSpeed(speed);
                if (actuatorTarget < targetInitialStep)
                        SetDirection(RETRACT);
                else
                        SetDirection(EXTEND);
                        
        } else if (controlMode == rateOfChange)
        {

        }
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

void Actuator::Extend(int steps)
{
        controlMode = target;
        targetInitialStep = encoder->read();
        actuatorTarget = min(upperLimit, targetInitialStep + steps);
}
