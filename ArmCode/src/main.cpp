#include <Arduino.h>
#include <SPI.h>
#include <HighPowerStepperDriver.h>
#include <Encoder.h>

#pragma region Variable Declarations
int stepPwmPin = 8;
int stepDirPin = 9;

int m2PwmPin = 18;
int m2DirPin = 19;

int m3PwmPin = 14;
int m3DirPin = 15;

int m4PwmPin = 36;
int m4DirPin = 37;

int m2Enc1 = 22;
int m2Enc2 = 23;

int m3Enc1 = 40;
int m3Enc2 = 41;

int m4Enc1 = 34;
int m4Enc2 = 35;

const int StepperCSPin = 10;
const int SensorCSPin = 24;

int p29State = LOW;

HighPowerStepperDriver sd;

int iter = 0;

Encoder encoder2(m2Enc1, m2Enc2);
Encoder encoder3(m3Enc1, m3Enc2);
Encoder encoder4(m4Enc1, m4Enc2);

int actuator2Targ = 0;
int actuator3Targ = 0;
int actuator4Targ = 0;

int actuatorTolerance = 50;
#pragma endregion


//*******************************
// Axis 2 Rate of Change
//*******************************
static float dxde = 38.1 / 4000.0;
static float min_length = 317.5;
static float side_a = 93.0;
static float side_b = 390.0;
float extension = 0;
int last_encoder_read = 0;
int time_last_read = 0;
float angle = 0;
void calculate_axis_2_extension()
{
  extension = min_length + dxde*encoder2.read();
}
void calculate_angle()
{
  angle = (180.f / M_PI) * acosf((powf(extension, 2) - (powf(side_a, 2) + powf(side_b, 2))) / (-2*side_a*side_b));
}


//*******************************
// Function Declarations
//*******************************
void monitorActuators();
void parseCommand(String command);

void setup()
{

  Serial.begin(115200);
  SPI.begin();
  sd.setChipSelectPin(StepperCSPin);

  while (!Serial)
  {
    delay(10);
  }

  Serial.println("status;Starting arm base code!");

  // Give the driver some time to power up.
  delay(1);

  // Reset the driver to its default settings and clear latched status
  // conditions.
  sd.resetSettings();
  sd.clearStatus();

  // Select auto mixed decay.  TI's DRV8711 documentation recommends this mode
  // for most applications, and we find that it usually works well.
  sd.setDecayMode(HPSDDecayMode::AutoMixed);

  // Set the current limit. You should change the number here to an appropriate
  // value for your particular system.
  sd.setCurrentMilliamps36v4(1000);

  // Set the number of microsteps that correspond to one full step.
  sd.setStepMode(HPSDStepMode::MicroStep32);

  if (sd.verifySettings())
  {
    Serial.println("status;Stepper driver passed verification!");
    sd.enableDriver();
  }
  else
  {
    Serial.println("status;ERROR: Stepper driver failed verification!");
  }
  // Enable the motor outputs.

  Serial.print("status;");
  Serial.println(sd.readFaults());

  delay(1000);

  pinMode(m2PwmPin, OUTPUT);
  pinMode(m2DirPin, OUTPUT);
  digitalWrite(m2DirPin, LOW);
  digitalWrite(m2PwmPin, LOW);

  pinMode(m3PwmPin, OUTPUT);
  pinMode(m3DirPin, OUTPUT);
  digitalWrite(m3DirPin, LOW);
  digitalWrite(m3PwmPin, LOW);

  pinMode(m4PwmPin, OUTPUT);
  pinMode(m4DirPin, OUTPUT);
  digitalWrite(m4DirPin, LOW);
  digitalWrite(m4PwmPin, LOW);

  Serial.print("status;");
  Serial.print(encoder2.read());
  Serial.print(", ");
  Serial.print(encoder3.read());
  Serial.print(", ");
  Serial.println(encoder4.read());

  encoder2.readAndReset();
  encoder3.readAndReset();
  encoder4.readAndReset();

  Serial.println("status;Arm base setup complete!");

  pinMode(29, INPUT);
}

unsigned long last_step = 0;
signed long steps = 0;
signed long target = 0;
signed int dir = 1;

unsigned long last_print = 0;

void loop()
{
  // TODO: Implement an interrupt based timing loop instead
  if (micros() > last_step + 100)
  {
    if ((steps > target && dir < 0) || (steps < target && dir > 0))
    {
      sd.step();
      steps += dir;
      last_step = micros();
    }
  }

  // if (millis() > last_print + 1000)
  // {
  //   last_print = millis();
  //   Serial.print("Positions: S=");
  //   Serial.print(steps);
  //   Serial.print(", 1=");
  //   Serial.print(encoder2.read());
  //   Serial.print(", 2=");
  //   Serial.print(encoder3.read());
  //   Serial.print(", 3=");
  //   Serial.println(encoder4.read());
  // }

  monitorActuators();

  if (Serial.available() > 1)
  {
    String command = Serial.readStringUntil('\n');
    parseCommand(command);
    // char device = message.charAt(0);
    // int value = message.substring(1).toInt();
    // if (device == 'S')
    // {
    //   target = value;
    //   if (target > steps)
    //   {
    //     dir = 1;
    //     sd.setDirection(1);
    //   }
    //   else if (target < steps)
    //   {
    //     dir = -1;
    //     sd.setDirection(0);
    //     digitalWrite(m1PwmPin, HIGH);
    //     digitalWrite(m2PwmPin, HIGH);
    //     digitalWrite(m3PwmPin, HIGH);
    //   }
    //   Serial.print("status;Setting target to ");
    //   Serial.println(target);
    // }
    // else if (device >= 'A' && device <= 'C')
    // {
    //   int actuatorIndex = (int)(device - 'A');

    //   switch (actuatorIndex)
    //   {
    //   case 0:
    //     actuator1Targ = value;
    //     break;
    //   case 1:
    //     actuator2Targ = value;
    //     break;
    //   case 2:
    //     actuator3Targ = value;
    //     break;

    //   default:
    //     break;
    //   }
    // }
    // else if (device == '0')
    // {
    //   target = steps;
    //   actuator1Targ = encoder1.read();
    //   actuator2Targ = encoder2.read();
    //   actuator3Targ = encoder3.read();
    // }
    // else if (device == 'Z')
    // {
    //   if (value == 0)
    //   {
    //     steps = 0;
    //     target = 0;
    //   }
    //   else if (value == 1)
    //   {
    //     encoder1.readAndReset();
    //     actuator1Targ = 0;
    //   }
    //   else if (value == 2)
    //   {
    //     encoder2.readAndReset();
    //     actuator2Targ = 0;
    //   }
    //   else if (value == 3)
    //   {
    //     encoder3.readAndReset();
    //     actuator3Targ = 0;
    //   }
    // }

    // Serial.print("status;Targets: S=");
    // Serial.print(target);
    // Serial.print(", 1=");
    // Serial.print(actuator1Targ);
    // Serial.print(", 2=");
    // Serial.print(actuator2Targ);
    // Serial.print(", 3=");
    // Serial.println(actuator3Targ);
  }

  // Read the axis 1 homing switch
  int newP29State = digitalRead(29);
  if (newP29State != p29State)
  {
    p29State = newP29State;
    Serial.print("status;Pin 29 changed to ");
    Serial.println(p29State ? "HIGH" : "LOW");
  }
}

void monitorActuators()
{
  calculate_axis_2_extension();
  calculate_angle();

  short speed_test = 255;
  if (encoder2.read() > actuator2Targ + actuatorTolerance / 2)
  {
    digitalWrite(m2DirPin, HIGH);
    //digitalWrite(m2PwmPin, HIGH);
    analogWrite(m2PwmPin, speed_test);
  }
  else if (encoder2.read() < actuator2Targ - actuatorTolerance / 2)
  {
    digitalWrite(m2DirPin, LOW);
    //digitalWrite(m2PwmPin, HIGH);
    analogWrite(m2PwmPin, speed_test);
  }
  else
  {
    analogWrite(m2PwmPin, 0);
  }

  if (encoder3.read() > actuator3Targ + actuatorTolerance / 2)
  {
    digitalWrite(m3DirPin, HIGH);
    digitalWrite(m3PwmPin, HIGH);
  }
  else if (encoder3.read() < actuator3Targ - actuatorTolerance / 2)
  {
    digitalWrite(m3DirPin, LOW);
    digitalWrite(m3PwmPin, HIGH);
  }
  else
  {
    digitalWrite(m3PwmPin, LOW);
  }

  if (encoder4.read() > actuator4Targ + actuatorTolerance / 2)
  {
    digitalWrite(m4DirPin, HIGH);
    digitalWrite(m4PwmPin, HIGH);
  }
  else if (encoder4.read() < actuator4Targ - actuatorTolerance / 2)
  {
    digitalWrite(m4DirPin, LOW);
    digitalWrite(m4PwmPin, HIGH);
  }
  else
  {
    digitalWrite(m4PwmPin, LOW);
  }
}

void parseCommand(String command)
{
  String exec = command.substring(0, command.indexOf(';'));
  if (exec.equals("move_actuator"))
  {
    int firstComma = command.indexOf(',');
    String actuator = command.substring(exec.length() + 1, firstComma);
    String delta_str = command.substring(firstComma + 1);

    int delta = atoi(delta_str.c_str());

    if (actuator.equals("2"))
    {
      actuator2Targ += delta;
    }
    else if (actuator.equals("3"))
    {
      actuator3Targ += delta;
    }
    else if (actuator.equals("4"))
    {
      actuator4Targ += delta;
    }
  } else if (exec.equals("move_stepper"))
  {
    String delta_str = command.substring(exec.length() + 1);

    int delta = atoi(delta_str.c_str());
  } else if (exec.equals("read_positions"))
  {
    Serial.print("positions; S=");
    Serial.print(steps);
    Serial.print(", 2=");
    Serial.print(encoder2.read());
    Serial.print(" "); Serial.print(angle);
    Serial.print(", 3=");
    Serial.print(encoder3.read());
    Serial.print(", 4=");
    Serial.println(encoder4.read());
  } else if (exec.equals("reset")) {
    String actuator = command.substring(exec.length() + 1, command.length()-1);
    if (actuator.equals("2"))
    {
      encoder2.readAndReset();
      actuator2Targ = 0;
    }
    else if (actuator.equals("3"))
    {
      encoder3.readAndReset();
      actuator3Targ = 0;
    }
    else if (actuator.equals("4"))
    {
      encoder4.readAndReset();
      actuator4Targ = 0;
    }
  }
}
