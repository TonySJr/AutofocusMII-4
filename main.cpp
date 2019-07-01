#include <Arduino.h>
#define BAUDSPEED 9600
#define DIR_PIN 6
#define STEP_PIN 7
#define HALFSTEP 8
#define QUARTERSTEP 9
//fullstep HALFSTEP AND QUARTERSTEP need to be pull low
#define DRIVERSLEEP 10
#define STEPPEROFF 11
#define DRIVERRESET 12

uint32_t steps = 20;
volatile uint32_t speed = 1500;

void Configure_Pins()
{
  for(uint8_t i = 6; i <= 12; i++)
  {
    pinMode(i, OUTPUT);
  } 
  for(uint8_t i = 10; i <= 12; i++)
  {
    digitalWrite(i,HIGH);
  }

  //legs configure
  digitalWrite(HALFSTEP,LOW);
  digitalWrite(QUARTERSTEP,LOW);

  //led configure
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,LOW);
}

void rotate(uint32_t steps)
{ 
  digitalWrite(STEPPEROFF,LOW);
  float usDelay = speed;
  for(uint32_t i = 0; i < steps; i++)
  { 
    digitalWrite(STEP_PIN, HIGH); 
    delayMicroseconds(usDelay);
    digitalWrite(STEP_PIN, LOW); 
    delayMicroseconds(usDelay); 
  } 
  digitalWrite(STEPPEROFF,HIGH);
}

void setup() {
  Serial.begin(BAUDSPEED);
  //configure pins
  Configure_Pins();
}

void loop() 
{
  if(Serial.available())
  {
    uint8_t instruct = Serial.read();
    switch (instruct)
    {
      case '1':
        digitalWrite(DIR_PIN,1);
        rotate(steps); 
        Serial.write(instruct); 
      break;
      case '2':
        digitalWrite(DIR_PIN,0);
        rotate(steps);
        Serial.write(instruct); 
      break;
    }
  }  
}
