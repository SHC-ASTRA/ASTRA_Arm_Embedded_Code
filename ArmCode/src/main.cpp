#include <Actuator.h>

// Global Variables
Stream* MySerial;

// Linear Actuator Declarations
Actuator Axis2(18, 19, 22, 23, 0, 17000, 317.5, 93.0, 390.0, 38.1/4000.0);

void setup()
{
    #pragma region Serial Configuration
    Serial.begin(115200);
    Serial1.begin(115200);

    Serial.println("status;Beginning Setup");
    Serial1.println("status;Beginning Setup");

    while (!Serial && millis() < 10000)
    {
        delay(10);
    }

    if(Serial)
    {
        Serial.println("status;Using USB Serial");
        Serial1.println("status;Using USB Serial");
        MySerial = &Serial;
    }
    else
    {
        Serial1.println("status;Using Hardware Serial1");
        MySerial = &Serial1;
    }
    #pragma endregion
   
    MySerial->println("status;Starting Arm Base Teensy!");

    MySerial->println("status;Initializing Axis 2");
    Axis2.Initalize();

}

void loop()
{
              
}