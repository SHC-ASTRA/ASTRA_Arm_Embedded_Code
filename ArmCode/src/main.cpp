#include <Actuator.h>

// Global Variables
Stream* MySerial;

// Linear Actuator Declarations
Actuator Axis2(18, 19, 22, 23,          // PIns
                0, 15000,               // Lower and Upper Limits
                318.3, 93.0, 390.0,     // Length of: Actuator Fully Retracted, Side A, Side B
                1 / 102.4);             // change in extension (mm) / change in encoder steps

Actuator Axis3(14, 15, 40, 41,          // Pins
                -20980+1000, 0,         // Lower and Upper Limits (Since Axis3 homes to extend, the range of travel is treated as negative)
                318.3, 138.0, 444.5,    // Length of: Actuator Fully Extended, Side A, Side B
                1 / 102.4);             // change in extension (mm) / change in encoder steps


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
    Axis2.Initalize();          MySerial->println("status;Axis 2 Initalized, beginning homing sequence.");

    while (Axis2.IsEStopActive())
    {
        MySerial->println("status;EStop detected... trying again in 1 second.");
        delay(1000);
    }

    Axis2.Home();               MySerial->println("status;Axis 2 finished Homing Sequence.");
    Axis2.SetTarget(10500);     MySerial->println("status;Axis 2 Target set for 10500.");
    Axis2.WaitForTarget();      MySerial->println("status;Axis 2 reached Target.");

    MySerial->println("status;Initializing Axis 3.");
    Axis3.Initalize();          MySerial->println("status;Axis 3 Initalized, beginning homing sequence.");
    Axis3.Home(false);          MySerial->println("status;Axis 3 finished Homing Sequence.");
    Axis3.SetTarget(-17500);    MySerial->println("status;Axis 3 Target set for -17500.");
    Axis3.WaitForTarget();      MySerial->println("status;Axis 3 reached Target.");
}

int lastTime = 0;
bool toggle = false;

void loop()
{
    /*
    Axis2.SetTarget(steps); - This function moves the actuator to the specified step.
    Axis2.Extend(steps); - This function extends the actuator by the specified steps. Negative steps = retract.
    Axis2.SetTargetRate(rate); - This function tells the actuator to match the specified rate in degrees per second. (positive = extend, negative = retract)
    Axis2.GetAngle(); - This function returns the angle the joint is at
    */

    // These functions update the control loops, need to run as frequently as possible.
    Axis2.Update();
    Axis3.Update();

    // Sample Debugging Script
    // Prints out actuator stats while a control loop is active
    if(Axis2.IsActive() && (millis() - lastTime) > 10)
    {
        MySerial->printf("status;Axis 2: %d, %d, %d, %d, %f, %f, %f\n", 
        Axis2.GetSpeed(), Axis2.GetDirection(), Axis2.GetStep(), Axis2.GetTarget(),
        Axis2.GetAngle(), Axis2.GetAngularRate(), Axis2.GetTargetRate());
        lastTime = millis();
    }
}