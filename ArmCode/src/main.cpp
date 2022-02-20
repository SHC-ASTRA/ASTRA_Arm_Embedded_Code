#include <Actuator.h>
#include <Stepper.h>

// Global Variables
Stream *MySerial;

// Stepper Declaration
Stepper Axis1(10, 29,     // Pins
              -270, 270); // Lower and Upper Limits

// Linear Actuator Declarations
Actuator Axis2(18, 19, 22, 23,     // PIns
               0, 16000,           // Lower and Upper Limits
               318.3, 90.0, 400.0, // Length of: Actuator Fully Retracted, Side A, Side B
               1 / 102.4,          // change in extension (mm) / change in encoder steps
               -55.9,              // Angle to transform calcualated angle to world angle
               4, 2, 0.2);         // PID Gains

Actuator Axis3(14, 15, 40, 41,      // Pins
               -20980 + 1000, 0,    // Lower and Upper Limits (Since Axis3 homes to extend, the range of travel is treated as negative)
               521.5, 138.0, 444.5, // Length of: Actuator Fully Extended, Side A, Side B
               1 / 102.4,           // change in extension (mm) / change in encoder steps
               -144.7,              // Angle to transform calculate angle to world angle
               4, 2, 0.2);          // PID Gains

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

    if (Serial)
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

    MySerial->println("status;Initializing Axis 1.");
    Axis1.Initialize();
    Axis1.Enable();

    MySerial->println("status;Initializing Axis 2.");
    Axis2.Initalize();
    MySerial->println("status;Axis 2 Initalized, beginning homing sequence.");

    while (Axis2.IsEStopActive())
    {
        MySerial->println("status;EStop detected... trying again in 1 second.");
        delay(1000);
    }

    Axis2.Home();
    MySerial->println("status;Axis 2 finished Homing Sequence.");
    Axis2.SetTarget(4000);
    MySerial->println("status;Axis 2 Target set for 10500.");
    // Axis2.WaitForTarget();      MySerial->println("status;Axis 2 reached Target.");

    MySerial->println("status;Initializing Axis 3.");
    Axis3.Initalize();
    MySerial->println("status;Axis 3 Initalized, beginning homing sequence.");
    Axis3.Home(false);
    MySerial->println("status;Axis 3 finished Homing Sequence.");
    Axis3.SetTarget(-1000);
    MySerial->println("status;Axis 3 Target set for -17500.");
    // Axis3.WaitForTarget();      MySerial->println("status;Axis 3 reached Target.");
}

int lastTime = 0;
int lastHeaderTime = 0;
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
    Axis1.Update();
    Axis2.Update();
    Axis3.Update();

    // Sample Debugging Script
    // Prints out actuator stats while a control loop is active
    if ((millis() - lastTime) > 50)
    {
        if (Axis1.IsActive())
        {
            MySerial->printf("feedback; 1: %f\n",
                             Axis1.GetRotation());
        }
        if (Axis2.IsActive())
        {
            MySerial->printf("feedback; 2: %f, %f, %f, %f, %i\n",
                             Axis2.GetWorldAngle(), Axis2.GetAngularRate(), Axis2.GetTargetRate(), Axis2.GetExtension(), Axis2.GetSpeed() * (Axis2.GetDirection() == EXTEND ? 1 : -1));
        }
        if (Axis3.IsActive())
        {
            MySerial->printf("feedback; 3: %f, %f, %f, %f, %i\n",
                             Axis3.GetWorldAngle(), Axis3.GetAngularRate(), Axis3.GetTargetRate(), Axis3.GetExtension(), Axis3.GetSpeed() * (Axis3.GetDirection() == EXTEND ? 1 : -1));
        }

        lastTime = millis();
    }

    if (MySerial->available() > 1)
    {
        String command = MySerial->readStringUntil('\n');
        int axis = command.charAt(0) - '0';
        float value = command.substring(2).toFloat();
        switch (axis)
        {
        case 1:
            Axis1.SetTargetRate(value);
            break;

        case 2:
            Axis2.SetTargetRate(value);
            break;

        case 3:
            Axis3.SetTargetRate(value);
            break;

        default:
            MySerial->printf("error; Invalid axis: %i\n", axis);
            break;
        }
    }
}