#include <CircularBuffer.h>
#include <Encoder.h>

#define EXTEND LOW
#define RETRACT HIGH

#define BUFFER_SIZE 10          // Size of the encoder buffer
#define BUFFER_TIME_STEP 10     // How often to add to the buffer

#define HOMING_SPEED 128        // Speed to use when homing actuators

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

        // Control Variables
        enum ControlMode {target, rateOfChange};
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

        // Actuator Kinematics Functions
        float CalculateExtension(int encoderReading);
        float CalculateAngle(float actuatorExtension);
        void CalculateAngularRate();

        // Control Loop
        void Update();

    public:
        Actuator(int pwmPin, int dirPin, int encoderPinA,
        int encoderPinB, int lowerLimit, int upperLimit,
        float baseLength, float sideALength, float sideBLength, 
        float dxde);

        void Initalize();

        // Target Based Control
        void Extend(int steps);
        void Retract(int steps);

        // This function homes the actuator
        // This function is blocking
        void Home(bool retract=true);

        // Getter Functions
        bool IsActive() { return actuatorSpeed != 0; }

        float GetExtension() { return extension; }
        float GetAngle() { return angle; }
        float GetAngularRate() { return angularRateOfChange; }
};