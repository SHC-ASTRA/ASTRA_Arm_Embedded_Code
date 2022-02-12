#include <Arduino.h>
#include <SPI.h>
#include <HighPowerStepperDriver.h>
#include <Encoder.h>

int stepPwmPin = 8;
int stepDirPin = 9;

int m1PwmPin = 18;
int m1DirPin = 19;

int m2PwmPin = 14;
int m2DirPin = 15;

int m3PwmPin = 36;
int m3DirPin = 37;

int m1Enc1 = 22;
int m1Enc2 = 23;

int m2Enc1 = 40;
int m2Enc2 = 41;

int m3Enc1 = 34;
int m3Enc2 = 35;

const int StepperCSPin = 10;
const int SensorCSPin = 24;

int p29State = LOW;

HighPowerStepperDriver sd;

int iter = 0;

Encoder encoder1(m1Enc1, m1Enc2);
Encoder encoder2(m2Enc1, m2Enc2);
Encoder encoder3(m3Enc1, m3Enc2);

int actuator1Targ = 0;
int actuator2Targ = 0;
int actuator3Targ = 0;

int actuatorTolerance = 50;

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

  pinMode(m1PwmPin, OUTPUT);
  pinMode(m1DirPin, OUTPUT);
  digitalWrite(m1DirPin, LOW);
  digitalWrite(m1PwmPin, LOW);

  pinMode(m2PwmPin, OUTPUT);
  pinMode(m2DirPin, OUTPUT);
  digitalWrite(m2DirPin, LOW);
  digitalWrite(m2PwmPin, LOW);

  pinMode(m3PwmPin, OUTPUT);
  pinMode(m3DirPin, OUTPUT);
  digitalWrite(m3DirPin, LOW);
  digitalWrite(m3PwmPin, LOW);

  Serial.print("status;");
  Serial.print(encoder1.read());
  Serial.print(", ");
  Serial.print(encoder2.read());
  Serial.print(", ");
  Serial.println(encoder3.read());

  encoder1.readAndReset();
  encoder2.readAndReset();
  encoder3.readAndReset();

  Serial.println("status;Arm base setup complete!");

  pinMode(29,INPUT);
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

  if (millis() > last_print+1000)
  {
    last_print = millis();
    Serial.print("Positions: S=");
    Serial.print(steps);
    Serial.print(", 1=");
    Serial.print(encoder1.read());
    Serial.print(", 2=");
    Serial.print(encoder2.read());
    Serial.print(", 3=");
    Serial.println(encoder3.read());
  }

  if (encoder1.read() > actuator1Targ + actuatorTolerance / 2)
  {
    digitalWrite(m1DirPin, HIGH);
    digitalWrite(m1PwmPin, HIGH);
  }
  else if (encoder1.read() < actuator1Targ - actuatorTolerance / 2)
  {
    digitalWrite(m1DirPin, LOW);
    digitalWrite(m1PwmPin, HIGH);
  }
  else
  {
    digitalWrite(m1PwmPin, LOW);
  }

  if (encoder2.read() > actuator2Targ + actuatorTolerance / 2)
  {
    digitalWrite(m2DirPin, HIGH);
    digitalWrite(m2PwmPin, HIGH);
  }
  else if (encoder2.read() < actuator2Targ - actuatorTolerance / 2)
  {
    digitalWrite(m2DirPin, LOW);
    digitalWrite(m2PwmPin, HIGH);
  }
  else
  {
    digitalWrite(m2PwmPin, LOW);
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

  if (Serial.available() > 1)
  {
    String message = Serial.readStringUntil('\n');
    char device = message.charAt(0);
    int value = message.substring(1).toInt();
    if (device == 'S')
    {
      target = value;
      if (target > steps)
      {
        dir = 1;
        sd.setDirection(1);
      }
      else if (target < steps)
      {
        dir = -1;
        sd.setDirection(0);
        digitalWrite(m1PwmPin, HIGH);
        digitalWrite(m2PwmPin, HIGH);
        digitalWrite(m3PwmPin, HIGH);
      }
      Serial.print("Setting target to ");
      Serial.println(target);
    }
    else if (device >= 'A' && device <= 'C')
    {
      int actuatorIndex = (int)(device - 'A');

      switch (actuatorIndex)
      {
      case 0:
        actuator1Targ = value;
        break;
      case 1:
        actuator2Targ = value;
        break;
      case 2:
        actuator3Targ = value;
        break;

      default:
        break;
      }
    }
    else if (device == '0')
    {
      target = steps;
      actuator1Targ = encoder1.read();
      actuator2Targ = encoder2.read();
      actuator3Targ = encoder3.read();
    }
    else if (device == 'Z')
    {
      if (value == 0)
      {
        steps = 0;
        target = 0;
      }
      else if(value == 1)
      {
        encoder1.readAndReset();
        actuator1Targ = 0;
      }
      else if(value == 2)
      {
        encoder2.readAndReset();
        actuator2Targ = 0;
      }
      else if(value == 3)
      {
        encoder3.readAndReset();
        actuator3Targ = 0;
      }
    }

    Serial.print("status;Targets: S=");
    Serial.print(target);
    Serial.print(", 1=");
    Serial.print(actuator1Targ);
    Serial.print(", 2=");
    Serial.print(actuator2Targ);
    Serial.print(", 3=");
    Serial.println(actuator3Targ);
  }

  // Read the axis 1 homing switch
  int newP29State = digitalRead(29);
  if (newP29State != p29State)
  {
    p29State = newP29State;
    Serial.print("Pin 29 changed to ");
    Serial.println(p29State ? "HIGH" : "LOW");
  }
}