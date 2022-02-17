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

    while (!Serial && millis() < 100000)
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

    MySerial->println("status;Initializing Axis 2.");
    Axis2.Initalize();
    MySerial->println("status;Axis 2 Initalized, beginning homing sequence.");
    Axis2.Home();
    MySerial->println("status;Axis 2 finished Homing Sequence.");
    Axis2.Extend(7500);
    MySerial->println("status;Axis 2 Target set for 7500.");
}

int lastTime = 0;
bool toggle = false;

void loop()
{
    Axis2.Update();

    if(Axis2.IsActive() && (millis() - lastTime) > 250)
    {
        MySerial->printf("status;Axis 2: %d, %d, %d, %d, %f, %f\n", 
        Axis2.GetSpeed(), Axis2.GetDirection(), Axis2.GetStep(), Axis2.GetTarget(),
        Axis2.GetAngle(), Axis2.GetAngularRate());
        lastTime = millis();
    }

    if (!Axis2.IsActive())
    {
        toggle = !toggle;
        
        if (toggle)
            Axis2.Extend(3000);
        else
            Axis2.Extend(-3000);

    }
}