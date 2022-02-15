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

// AS5X47 as5047d(SensorCSPin);

int iter = 0;

Encoder encoder1(m1Enc1, m1Enc2);
Encoder encoder2(m2Enc1, m2Enc2);
Encoder encoder3(m3Enc1, m3Enc2);

int actuator1Targ = 0;
int actuator2Targ = 0;
int actuator3Targ = 0;

int actuatorTolerance = 50;
Stream* MySerial;

void setup()
{

  Serial.begin(115200);
  Serial1.begin(115200);

  Serial.println("Beginning Setup");
  Serial1.println("Beginning Setup");

  while (!Serial && millis() < 10000)
  {
    delay(10);
  }
 
  if(Serial)
  {
    Serial.println("Using USB Serial");
    Serial1.println("Using USB Serial");
    MySerial = &Serial;
  }
  else
  {
    Serial1.println("Using Hardware Serial1");
    MySerial = &Serial1;
  }

  MySerial->println("Starting!");

  SPI.begin();
  sd.setChipSelectPin(StepperCSPin);

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
    MySerial->println("Stepper driver passed verification!");
    sd.enableDriver();
  }
  else
  {
    MySerial->println("ERROR: Stepper driver failed verification!");
  }
  // Enable the motor outputs.

  MySerial->println(sd.readFaults());

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

  MySerial->print(encoder1.read());
  MySerial->print(", ");
  MySerial->print(encoder2.read());
  MySerial->print(", ");
  MySerial->println(encoder3.read());

  encoder1.readAndReset();
  encoder2.readAndReset();
  encoder3.readAndReset();

  MySerial->println("Hi!");

  pinMode(29,INPUT);
}

unsigned long last_step = 0;
signed long steps = 0;
signed long target = 0;
signed int dir = 1;

unsigned long last_print = 0;

void loop()
{
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
    MySerial->print("Positions: S=");
    MySerial->print(steps);
    MySerial->print(", 1=");
    MySerial->print(encoder1.read());
    MySerial->print(", 2=");
    MySerial->print(encoder2.read());
    MySerial->print(", 3=");
    MySerial->println(encoder3.read());
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

  if (MySerial->available() > 1)
  {
    String message = MySerial->readStringUntil('\n');
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
      MySerial->print("Setting target to ");
      MySerial->println(target);
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

    MySerial->print("Targets: S=");
    MySerial->print(target);
    MySerial->print(", 1=");
    MySerial->print(actuator1Targ);
    MySerial->print(", 2=");
    MySerial->print(actuator2Targ);
    MySerial->print(", 3=");
    MySerial->println(actuator3Targ);
  }

  int newP29State = digitalRead(29);
  if (newP29State != p29State)
  {
    p29State = newP29State;
    MySerial->print("Pin 29 changed to ");
    MySerial->println(p29State ? "HIGH" : "LOW");
  }

  // if (millis() > last_send + 50)
  // {
  //   float angle = as5047d.readAngle();
  //   MySerial->println(angle);
  //   last_send = millis();
  // }
  // Read the measured angle
}