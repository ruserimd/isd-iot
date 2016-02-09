#include <avr/sleep.h>  
#include "TimerOne.h"
#include "BH1750.h"
#include <SPI.h>        
#include <Ethernet.h>
#include <EthernetUdp.h>

#define lightLedPin 8
#define motionLedPin 9
#define pirPin 2
#define pirSensorCalibrationTime 10  // Calibration time for PIR sensor, 10-60ms according to datasheet.
#define lightTimerCounter 10         // After how many timer interruptions the led will blink. 0.5 x 10 = 5s.
#define motionPause 5000             // The amount of milliseconds the PIR sensor has to be low 
                                     // before we assume all motion has stopped.
#define HBInterval 5                 // Send control message after 5 x 5s = 25s.

BH1750 lightMeter;

byte light_FLAG = 0;
byte motion_FLAG = 0;
byte lightTurnedOff = 0;
byte HBTimerCounter = 0;
byte firstMessageToServer = 0;

// The MAC address and IP address for the controller. They will be dependent on local network.
byte mac[] = { 0x90, 0xA2, 0xDA, 0x0F, 0xB6, 0x47 };
IPAddress ip(172, 17, 41, 54);
unsigned int localPort = 9876;

IPAddress remoteIP(172, 17, 41, 71);
unsigned int remotePort = 9876;

// An EthernetUDP instance to let us send and receive packets over UDP.
EthernetUDP Udp;

// The time when the PIR sensor outputs a low impulse.
long unsigned int lowIn;     
boolean lockLow = true;
boolean takeLowTime;   

void setup(){    
  delay(200L);
  Serial.begin(9600);

  setupPins();
  setupInterruptions();
  lightMeter.begin();
  calibratePIRSensor();
  setupEthernet();
  bootIndicator();
  
  Serial.println();
  Serial.println("Light Sensor Started");
  Serial.println("Motion Sensor Started");
  Serial.println("Ethernet Started");
  delay(50L); 
}

/*
 * Give the sensor some time to calibrate. 10-60ms according to datasheet.
 */
void calibratePIRSensor() {
  Serial.print("Calibrating sensor ");
  for(int i = 0; i < pirSensorCalibrationTime; i++){
    Serial.print(".");
    delay(1000L);
  }
}

/*
 *  At bootup, flash LED 3 times quick so I know the reboot has occurred.
 */
void bootIndicator() {
  for (byte k = 1; k <= 3; k = k + 1) {
    digitalWrite(lightLedPin, HIGH);
    digitalWrite(motionLedPin, HIGH);
    delay(250L);
    digitalWrite(lightLedPin, LOW);
    digitalWrite(motionLedPin, LOW);
    delay(250L);
  }
  // Delay a bit more so it is clear we are done with setup.
  delay(750L);
}

void setupPins() {
  // Set input & output pin for movement detection.
  pinMode(motionLedPin,OUTPUT);
  pinMode(pirPin,INPUT);

  // Set output pin for light detection.
  pinMode(lightLedPin,OUTPUT);
}

void setupInterruptions() {
  // Set external interruption for PIR sensor.
  attachInterrupt(digitalPinToInterrupt(2), motionWakeUp, CHANGE);

  // Set timer for light measurements.
  Timer1.initialize(500000);       
  Timer1.attachInterrupt(lightWakeUp); 
}

/*
 * Setup device's IP, MAC and local port.
 */
void setupEthernet() {
  if (Ethernet.begin(mac) == 0) {
    Serial.println("Failed to configure Ethernet using DHCP");
  }

  // Start the Ethernet and UDP.
  Ethernet.begin(mac,ip);
  Udp.begin(localPort);

  // Ethernet Shield IP.
  Serial.println();
  Serial.print("Device IP : ");
  Serial.println(Ethernet.localIP());
}

void loop(){    

  sendTheFirstData();

  // Check for motion detection.
  if (motion_FLAG == 1) {   
    if(digitalRead(pirPin) == HIGH){
     digitalWrite(motionLedPin, HIGH); // The led visualizes the sensors output pin state.
     
      if(lockLow){  
       lockLow = false;                 // Makes sure we wait for a transition to LOW before any further output is made.            
       String motionValue = String(motion_FLAG); 
       String lightValue = String(lightMeter.readLightLevel());
       sendUDP("P", motionValue, "L", lightValue, "H", String(1));   
       HBTimerCounter = 0;             // Reset HB timer. 
       delay(50L);
      }         
      takeLowTime = true;
    }
    
    if(digitalRead(pirPin) == LOW){       
     digitalWrite(motionLedPin, LOW);  // The led visualizes the sensors output pin state.
    
     if(takeLowTime){
      lowIn = millis();          // Save the time of the transition from high to LOW.
      takeLowTime = false;       // Make sure this is only done at the start of a LOW phase.
     }
     
     // If the sensor is low for more than the given pause, 
     // we assume that no more motion is going to happen.
     if(!lockLow && millis() - lowIn > motionPause) {  
       // Makes sure this block of code is only executed again after 
       // a new motion sequence has been detected.
       lockLow = true;                                   
       motion_FLAG = 0;
       String motionValue = String(motion_FLAG); 
       String lightValue = String(lightMeter.readLightLevel());
       sendUDP("P", motionValue, "L", lightValue, "H", String(1));  
       delay(50L);
      }
    }
  }

  // Check for light detection.
  if (light_FLAG > lightTimerCounter) {
    if (lightMeter.readLightLevel()>5) {           // Light is on.
      digitalWrite(lightLedPin, HIGH);
      delay(1000L);
      digitalWrite(lightLedPin, LOW);
      light_FLAG = 0;
      lightTurnedOff = 0;                         // Light is detected.
      HBTimerCounter = 0;                         // Reset HB timer. 
    } else if (lightMeter.readLightLevel()<5) {    // Light is off.
      digitalWrite(lightLedPin, LOW);
      light_FLAG = 0;
      lightTurnedOff = 1;
      HBTimerCounter++;
      Serial.println(HBTimerCounter);
    }
  }

  checkForControlSignal();  
  sleepNow();
}

/*
 * Send for the first time the data from sensors to the server when the device is launched.
 */
void sendTheFirstData() {
  if (firstMessageToServer == 0) {
    String motionValue = String(motion_FLAG); 
    String lightValue = String(lightMeter.readLightLevel());
    sendUDP("P", motionValue, "L", lightValue, "H", String(1));
    firstMessageToServer = 1;  
  }
}

/*
 * Send UDP message from Arduino to server.
 * 
 * @param sensor_1 PIR Sensor ID, ex. "P"
 * @param value_1  value from PIR Sensor
 * @param sensor_2 Light Sensor ID, ex. "L"
 * @param value_2  value from Light Sensor
 * @param sensor_3 Heartbeat ID, ex. "H"
 * @param value_3  value for HB
 */
void sendUDP(String sensor_1, String value_1, String sensor_2, String value_2, String hb, String value_3) {
  String messageToServer = sensor_1 + " " + value_1 + "|" + sensor_2 + " " + value_2 + "|" + hb + " " + value_3;
  Serial.println(messageToServer);
  Udp.beginPacket(remoteIP, remotePort);
  Udp.print(messageToServer);
  Udp.endPacket();
  digitalWrite(lightLedPin, HIGH);
  delay(1000L);
  digitalWrite(lightLedPin, LOW);
}

/*
 * Increment the light_FLAG when the interruption for the LIGHT sensor was produced. 
 */
void lightWakeUp()
{
  light_FLAG++; 
}

/*
 * Set motion_FLAG to 1 when the interruption for the PIR sensor was produced.
 */
void motionWakeUp(){
  motion_FLAG = 1;
}

/*
 * If any sensor has not changed after the period of time specified by @HBInterval, 
 * it sends to the server a control message with the last data from sensors, showing that 
 * the device works.
 */
void checkForControlSignal() {
  if(lightTurnedOff == 1 && motion_FLAG == 0 && HBTimerCounter >= HBInterval){     
    String motionValue = String(motion_FLAG); 
    String lightValue = String(lightMeter.readLightLevel());
    sendUDP("P", motionValue, "L", lightValue, "H", String(1));   
    HBTimerCounter = 0;
  }
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
  sleep_mode();                                                      
  sleep_disable();                                                 // First thing after waking from sleep: disable sleep.  
  detachInterrupt(digitalPinToInterrupt(2));                       // Disables interrupt on pin 2 so the wakeUp code will not be executed during normal running time.  
}