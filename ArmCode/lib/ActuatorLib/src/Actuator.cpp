#include "Actuator.h"

Actuator::Actuator(int pwmPin, int dirPin, int encoderPinA,
                   int encoderPinB, int lowerLimit, int upperLimit,
                   float baseLength, float sideALength, float sideBLength,
                   float dxde, float worldTransformAngle,
                   float kp, float ki, float kd)
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
        this->worldTransformAngle = worldTransformAngle;
        this->Kp = kp;
        this->Ki = ki;
        this->Kd = kd;

        this->pidSetpoint = 0;
        this->pidInput = 0;
        this->pidOutput = 0;
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

        PID = new QuickPID(&pidInput, &pidOutput, &pidSetpoint);
        PID->SetTunings(Kp, Ki, Kd);
        PID->SetOutputLimits(-255, 255);
        PID->SetAntiWindupMode(QuickPID::iAwMode::iAwClamp);
        PID->SetSampleTimeUs(20000);
}

void Actuator::setPIDValues(float Kp, float Ki, float Kd)
{
        PID->SetTunings(Kp, Ki, Kd);
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

void Actuator::SetSignedSpeed(short speed)
{
        SetSpeed(abs(speed));
        SetDirection(speed > 0 ? EXTEND : RETRACT);
}

float Actuator::CalculateExtension(int encoderReading)
{
        return baseLength + dxde * encoderReading;
}

float Actuator::CalculateAngle(float actuatorExtension)
{
        return (180.f / M_PI) * acosf((powf(actuatorExtension, 2) - (powf(sideALength, 2) + powf(sideBLength, 2))) / (-2 * sideALength * sideBLength));
}

void Actuator::CalculateAngularRate()
{
        int dT = timingBuffer.first() - timingBuffer.last();

        float oldAngle = CalculateAngle(
            CalculateExtension(encoderBuffer.last()));

        angularRateOfChange = (angle - oldAngle) / (dT)*1000.0;
}

void Actuator::Update()
{
        bool newRateOfChange = false;
        if (BufferReadyForUpdate())
        {
                AddToBuffer();

                // Recalculate extension and angle
                extension = CalculateExtension(encoderBuffer.first());
                angle = CalculateAngle(extension);

                // Recalculate rate of change
                CalculateAngularRate();

                newRateOfChange = true;
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
                int ease = -((step - targetInitialStep) * (step - actuatorTarget)) / abs(actuatorTarget - targetInitialStep);
                int speed = min(255, abs(ease) + 100);

                SetSpeed(speed);
                if (actuatorTarget < targetInitialStep)
                        SetDirection(RETRACT);
                else
                        SetDirection(EXTEND);
        }
        else if (controlMode == rateOfChange)
        {
                int step = encoder->read();
                if (newRateOfChange)
                {       
                        pidInput = angularRateOfChange;

                        pidSetpoint = targetRate;

                        if ((step >= upperLimit-250 && targetRate>0) || (step <= lowerLimit+250 && targetRate < 0))
                        {
                                targetRate = 0; 
                        }

                        PID->Compute();

                        int signedSpeed = GetSpeed()*(GetDirection()==EXTEND?1:-1);
                        signedSpeed = round(max(min(signedSpeed + pidOutput, 255), -255));
                        SetSignedSpeed(signedSpeed);
                }

                if ((step >= upperLimit && actuatorDirection == EXTEND) || (step <= lowerLimit && actuatorDirection == RETRACT))
                {
                        SetSpeed(0);
                }
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

bool Actuator::IsEStopActive()
{
        int step = encoder->read();
        SetDirection(EXTEND);
        SetSpeed(HOMING_SPEED);
        delay(10);
        SetSpeed(0);
        int stepAfter = encoder->read();

        if (step != stepAfter)
                return false;

        step = encoder->read();
        SetDirection(RETRACT);
        SetSpeed(HOMING_SPEED);
        delay(10);
        SetSpeed(0);
        stepAfter = encoder->read();

        return step == stepAfter;
}

void Actuator::Extend(int steps)
{
        controlMode = target;
        targetInitialStep = encoder->read();
        actuatorTarget = max(lowerLimit, min(upperLimit, targetInitialStep + steps));
}

void Actuator::SetTarget(int step)
{
        controlMode = target;
        targetInitialStep = encoder->read();
        actuatorTarget = max(lowerLimit, min(upperLimit, step));
}

void Actuator::SetTargetRate(float rate)
{
        PID->SetMode(QuickPID::Control::automatic);
        
        controlMode = rateOfChange;
        targetRate = rate;

        // if (targetRate < 0)
        //         SetDirection(RETRACT);
        // if (targetRate > 0)
        //         SetDirection(EXTEND);
        // if (abs(targetRate) < ROC_MINIMUM_RATE)
        // {
        //         controlMode = idle;
        //         targetRate = 0;
        //         SetSpeed(0);
        //         PID->SetMode(QuickPID::Control::manual);
        // }
}

void Actuator::WaitForTarget()
{
        while (IsActive())
        {
                Update();
        }
}