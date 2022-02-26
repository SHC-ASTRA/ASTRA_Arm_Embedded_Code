#include <Actuator.h>
#include <Stepper.h>

// Global Variables
Stream *MySerial;

// Stepper Declaration
Stepper Axis1(10, 29,     // Pins
              -270, 270); // Lower and Upper Li   Service2.srvmits

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

void home();

void setup()
{
#pragma region Serial Configuration
    Serial.begin(9600);
    Serial1.begin(9600);

    Serial.println("status;Beginning Setup");
    Serial1.println("status;Beginning Setup");

    while (!Serial && millis() < 10000)
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

    MySerial->println("status;Initializing Axis 3.");
    Axis3.Initalize();
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
            MySerial->printf("feedback;x=1,a=%f,r=%f\n",
                             Axis1.GetRotation(), Axis1.GetAngularRate());
        }
        if (Axis2.IsActive())
        {
            MySerial->printf("feedback;x=2,a=%f,r=%f\n",
                             Axis2.GetWorldAngle(), Axis2.GetAngularRate());
        }
        if (Axis3.IsActive())
        {
            MySerial->printf("feedback;x=3,a=%f,r=%f\n",
                             Axis3.GetWorldAngle(), Axis3.GetAngularRate());
        }

        lastTime = millis();
    }

    if (MySerial->available() > 0)
    {
        String command = MySerial->readStringUntil('\n');
        if(command.length()==0)
        {
            return;
        }
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

        case 'h' - '0':
            home();
            break;

        default:
            MySerial->printf("status;Invalid axis: %i\n", axis);
            break;
        }
    }
}

void home()
{
    MySerial->println("status;Beginning homing sequence.");

    if (Axis2.IsEStopActive() || Axis3.IsEStopActive())
    {
        MySerial->println("error;E-Stop active during homing sequence, aborting.");
        MySerial->println("homing_status;false");
        return;
    }

    MySerial->println("status;Axis 3 started homing.");
    Axis3.Home(false);
    Axis3.SetTarget(-19980);
    Axis3.WaitForTarget();
    MySerial->println("status;Axis 3 finished homing.");

    MySerial->println("status;Axis 2 started homing.");
    Axis2.Home();
    MySerial->println("status;Axis 2 finished homing.");

    MySerial->println("homing_status;true");
}