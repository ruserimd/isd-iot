#include <Wire.h>
#include <avr/sleep.h>  
#include "TimerOne.h"
#include "BH1750.h"

#define lightLedPin 12
#define motionLedPin 13
#define pirPin 2

BH1750 lightMeter;

byte light_FLAG = 0;
byte motion_FLAG = 0;

void setup(){  
  Wire.begin();
  
  delay(200);
  Serial.begin(9600);

  // Set input & output pin for movement detection.
  pinMode(motionLedPin,OUTPUT);
  pinMode(pirPin,INPUT);

  // Set output pin for light detection.
  pinMode(lightLedPin,OUTPUT);
  
  attachInterrupt(digitalPinToInterrupt(2), motionWakeUp, CHANGE);

  Timer1.initialize(500000);       
  Timer1.attachInterrupt(lightWakeUp); 
  
  Serial.println("Light Sensor Started");
  Serial.println("Motion Sensor Started");
  delay(50);

  lightMeter.begin();
}

void loop(){  
  if (motion_FLAG == 1) {
    digitalWrite(motionLedPin, digitalRead(2));
    motion_FLAG = 0;    
  }
  
  if (light_FLAG == 1) {
    if (lightMeter.readLightLevel()>5) {
      digitalWrite(lightLedPin, LOW);
      delay(2000);
      digitalWrite(lightLedPin, HIGH);
      light_FLAG = 0;
    } else if (lightMeter.readLightLevel()<5) {      
      digitalWrite(lightLedPin, LOW);
      light_FLAG = 0;
    }
  }
  
  sleepNow();
}

/*
 * Set the light_FLAG to 1 when the interruption for the LIGHT sensor was produced. 
 */
void lightWakeUp()
{
  light_FLAG = 1; 
}

/*
 * Set motion_FLAG to 1 when the interruption for the PIR sensor was produced.
 */
void motionWakeUp(){
  motion_FLAG = 1;
}

/*
 * Sleep Arduino to use less power. In the Atmega8 datasheet
 * http://www.atmel.com/dyn/resources/prod_documents/doc2486.pdf on page 35
 * there is a list of sleep modes which explains which clocks and
 * wake up sources are available in which sleep mode.
 */
void sleepNow() {  
  set_sleep_mode(SLEEP_MODE_IDLE);                                 // Sleep mode is set here.  
  sleep_enable();                                                  // Enables the sleep bit in the mcucr register.  
  attachInterrupt(digitalPinToInterrupt(2), motionWakeUp, CHANGE); // Use interrupt pin 2 and run function. 
  Timer1.attachInterrupt(lightWakeUp); 
  sleep_mode();                                                     
  
  sleep_disable();                                                 // First thing after waking from sleep: disable sleep.  
  detachInterrupt(digitalPinToInterrupt(2));                       // Disables interrupt on pin 2 so the wakeUp code will not be executed during normal running time.  
}  
