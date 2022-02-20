#include <Arduino.h>
#include <CircularBuffer.h>
#include <Encoder.h>
#include <QuickPID.h>

#define EXTEND LOW
#define RETRACT HIGH

#define BUFFER_SIZE 10              // Size of the encoder buffer
#define BUFFER_TIME_STEP 10         // How often to add to the buffer

#define HOMING_SPEED 200            // Speed to use when homing actuators

#define ROC_CONTROL_TOLERANCE 0.15  // Degrees/second of toelrance to the rate of change
#define ROC_MINIMUM_RATE 0.02       // Degrees/second minimum rate of change
class Actuator
{
    private:
        // Pins to drive the actuator
        int pwmPin;
        int dirPin;

        // Pins to connect to the encoder
        int encoderPinA;
        int encoderPinB; 
        Encoder* encoder;

        // Safety limits on the actuators
        // Prevents over or under extension
        int lowerLimit;
        int upperLimit;

        // Actuator Status Variables
        short actuatorSpeed;
        short actuatorDirection;

        // Variables to calculate the arm angle
        float dxde; // change in extension per encoder step
        float baseLength; // Length of actuator
        float sideALength;
        float sideBLength;
        float extension; // Calculated extension of the actuator
        float angle; // Calculated angle of the joint
        float angularRateOfChange; // Calculated angularRateOfChange
        float worldTransformAngle; // Angle to transform calculated angle to world angle

        // Control Variables
        int actuatorTarget;
        int targetInitialStep;
        float targetRate;
        enum ControlMode {target, rateOfChange, idle};
        ControlMode controlMode;

        // Actuator Speed Tracking
        CircularBuffer<int, BUFFER_SIZE> encoderBuffer;
        CircularBuffer<int, BUFFER_SIZE> timingBuffer;

        // Buffer Helper Functions
        void ResetBuffers();
        bool BufferReadyForUpdate() { return millis() > (u_int32_t)(timingBuffer.first()) + BUFFER_TIME_STEP; }
        void AddToBuffer();

        // Helper Functions to Control the Actuator
        void SetSpeed(short speed) { analogWrite(pwmPin, speed); actuatorSpeed = speed; }
        void SetDirection(short direction) { digitalWrite(dirPin, direction); actuatorDirection = direction; }
        void SetSignedSpeed(short speed);

        // Actuator Kinematics Functions
        float CalculateExtension(int encoderReading);
        float CalculateAngle(float actuatorExtension);
        void CalculateAngularRate();

        //PID bits
        QuickPID* PID;
        float pidSetpoint;
        float pidInput;
        float pidOutput;
        float Kp;
        float Ki;
        float Kd;

    public:
        Actuator(int pwmPin, int dirPin, int encoderPinA,
        int encoderPinB, int lowerLimit, int upperLimit,
        float baseLength, float sideALength, float sideBLength, 
        float dxde, float worldTransformAngle,
        float kp, float ki, float kd);

        void Initalize();

        // Target Based Control
        // This function extends or retracts the actuator by a number of steps. Negative steps retract the actuator.
        void Extend(int steps);
        // This function enables the target control loop to get the actuator to the target step.
        void SetTarget(int target);
        // This function enables the rate control loop to move the joint at the desired rate.
        void SetTargetRate(float rate);
        // This function blocks the program until the actuator reaches the desired target step.
        void WaitForTarget();

        // This function homes the actuator
        // This function is blocking
        void Home(bool retract=true);

        // This function checks whether the E-Stop is activated
        bool IsEStopActive();

        // Control Loop
        void Update();

        void setPIDValues(float Kp, float Ki, float Kd);

        // Getter Functions

        // This function checks whether the control loop is idle.
        bool IsActive() { return controlMode != idle; }

        float GetAngle() { return angle; }
        float GetWorldAngle() { return GetAngle() + worldTransformAngle; }
        float GetAngularRate() { return angularRateOfChange; }
        int GetDirection () { return actuatorDirection; }
        float GetExtension() { return extension; }
        int GetSpeed() { return actuatorSpeed; }
        int GetStep() { return encoder->read(); }
        int GetTarget() { return actuatorTarget; }
        float GetTargetRate() { return targetRate; }
};