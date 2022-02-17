#include <CircularBuffer.h>
#include <Encoder.h>

#define EXTEND LOW
#define RETRACT HIGH

#define BUFFER_SIZE 50          // Size of the encoder buffer
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

        // Variables to calculate the arm angle
        float dxde; // change in extension per encoder step
        float baseLength; // Length of actuator
        float sideALength;
        float sideBLength;
        float extension; // Calculated extension of the actuator

        // Actuator Speed Tracking
        CircularBuffer<int, BUFFER_SIZE> encoderBuffer;
        CircularBuffer<int, BUFFER_SIZE> timingBuffer;

        void ResetBuffers(); // Safely resets the buffers
        bool BufferReadyForUpdate() { return millis() > timingBuffer.first() + BUFFER_TIME_STEP; }

        // Helper Functions to Control the Actuator
        void SetSpeed(short speed) { analogWrite(pwmPin, speed); }
        void SetDirection(short direction) { digitalWrite(dirPin, direction); }

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
};