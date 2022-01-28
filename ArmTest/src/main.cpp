#include <Arduino.h>
#include <SPI.h>
#include <HighPowerStepperDriver.h>
#include <Encoder.h>

int stepPwmPin = 36;
int stepDirPin = 37;

int m1PwmPin = 18;
int m1DirPin = 19;

int m2PwmPin = 14;
int m2DirPin = 15;

int m3PwmPin = 36;
int m3DirPin = 37;

int m1Enc1 = 40;
int m1Enc2 = 41;

int m2Enc1 = 22;
int m2Enc2 = 23;

int m3Enc1 = 34;
int m3Enc2 = 35;

const int StepperCSPin = 10;
const int SensorCSPin = 24;

HighPowerStepperDriver sd;

//AS5X47 as5047d(SensorCSPin);

int iter = 0;

Encoder encoder1(m1Enc1, m1Enc2);
Encoder encoder2(m2Enc1, m2Enc2);
Encoder encoder3(m3Enc1, m3Enc2);

void setup() {

  Serial.begin(115200);
  SPI.begin();
  sd.setChipSelectPin(StepperCSPin);

  while(!Serial)
  {
    delay(10);
  }

  Serial.println("Starting!");

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

  if(sd.verifySettings())
  {
    Serial.println("Stepper driver passed verification!");
    sd.enableDriver();
  }
  else
  {
    Serial.println("ERROR: Stepper driver failed verification!");
  }
  // Enable the motor outputs.
  
  Serial.println(sd.readFaults());

  delay(1000);

  pinMode(m1PwmPin,OUTPUT);
  pinMode(m1DirPin,OUTPUT);
  digitalWrite(m1DirPin,LOW);
  digitalWrite(m1PwmPin,HIGH);

  pinMode(m2PwmPin,OUTPUT);
  pinMode(m2DirPin,OUTPUT);
  digitalWrite(m2DirPin,LOW);
  digitalWrite(m2PwmPin,HIGH);

  pinMode(m3PwmPin,OUTPUT);
  pinMode(m3DirPin,OUTPUT);
  digitalWrite(m3DirPin,LOW);
  digitalWrite(m3PwmPin,HIGH);

  delay(10000);
  digitalWrite(m1PwmPin,LOW);
  digitalWrite(m2PwmPin,LOW);
  digitalWrite(m3PwmPin,LOW);

  Serial.print(encoder1.read());
  Serial.print(", ");
  Serial.print(encoder2.read());
  Serial.print(", ");
  Serial.println(encoder3.read());

  encoder1.readAndReset();
  encoder2.readAndReset();
  encoder3.readAndReset();

  
  digitalWrite(m1DirPin,HIGH);
  digitalWrite(m2DirPin,HIGH);
  digitalWrite(m3DirPin,HIGH);

  Serial.println("Hi!");
}

unsigned long last_step = 0;
signed long steps = 0;
signed long target = 0;
signed int dir = 1;

void loop()
{ 
   if (micros() > last_step + 100)
   {
     if((steps > target && dir < 0) || (steps < target && dir > 0))
     {
       sd.step();
       steps += dir;
       last_step = micros();
     }
   }

   if (Serial.available() > 1) {
     target = Serial.parseInt();
     if (target > steps)
     {
       dir = 1;
       sd.setDirection(1);
     }
     else if (target < steps)
     {
        dir = -1;
        sd.setDirection(0);
        digitalWrite(m1PwmPin,HIGH);
        digitalWrite(m2PwmPin,HIGH);
        digitalWrite(m3PwmPin,HIGH);
     }
     Serial.print("Setting target to ");
     Serial.println(target);
   }
  

  // if (millis() > last_send + 50)
  // {
  //   float angle = as5047d.readAngle();
  //   Serial.println(angle);
  //   last_send = millis();
  // }
  // Read the measured angle
  

}