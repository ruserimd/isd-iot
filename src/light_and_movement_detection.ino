#include <avr/sleep.h>  
#include "BH1750.h"
#include <SPI.h>        
#include <Ethernet.h>
#include <EthernetUdp.h>
#include "LowPower.h"

#define lightLedPin 8
#define motionLedPin 9
#define pirPin 2
#define pirSensorCalibrationTime 10  // Calibration time for PIR sensor, 10-60ms according to datasheet.
#define HBInterval 2                 // Depends of SLEEP_XS parameter in the LowPower.powerDown(SLEEP_4S, ADC_OFF, BOD_OFF) function.
                                     // X * HBInterval = seconds after that the control message is sent.

BH1750 lightMeter;

byte motion_sent = 0;
byte light_FLAG = 0;
byte motion_FLAG = 0;
byte HBTimerCounter = 0;
byte lightIndicator = 0;

// The MAC address and IP address for the controller. They will be dependent on local network.
byte mac[] = { 0x90, 0xA2, 0xDA, 0x0F, 0xB6, 0x47 };
IPAddress ip(172, 17, 41, 54);
unsigned int localPort = 9876;

IPAddress remoteIP(172, 17, 41, 85);
unsigned int remotePort = 9876;

// An EthernetUDP instance to let us send and receive packets over UDP.
EthernetUDP Udp;

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
}

/*
 * Setup device's IP, MAC and local port.
 */
void setupEthernet() {
  if (Ethernet.begin(mac) == 0) {
    Serial.println("Failed to configure Ethernet using DHCP");
  }

  // Start the Ethernet and UDP.
  Ethernet.begin(mac, ip);
  Udp.begin(localPort);

  // Ethernet Shield IP.
  Serial.println();
  Serial.print("Device IP : ");
  Serial.println(Ethernet.localIP());
}

void loop(){    

  // Check for motion detection.
  if (motion_FLAG == 1) {      
    if(digitalRead(pirPin) == HIGH){
     digitalWrite(motionLedPin, HIGH); // The led visualizes the sensors output pin state.
     digitalWrite(lightLedPin, HIGH);
      if (motion_sent == 0) {
        String motionValue = String(motion_FLAG); 
        String lightValue = String(lightMeter.readLightLevel());
        sendUDP("P", motionValue, "L", lightValue, "H", String(0));   
        motion_sent = 1;    
      }
    }
    
    if(digitalRead(pirPin) == LOW){       
     digitalWrite(motionLedPin, LOW); // The led visualizes the sensors output pin state.
     digitalWrite(lightLedPin, LOW);                                 
       motion_FLAG = 0;
       motion_sent = 0;
       String motionValue = String(motion_FLAG); 
       String lightValue = String(lightMeter.readLightLevel());
       sendUDP("P", motionValue, "L", lightValue, "H", String(0));  
    }
  }

  // Check for light detection.
  if (light_FLAG == 1) {    
    if (lightMeter.readLightLevel() > 5) { // Light is on.
      if (motion_FLAG == 1) {
        digitalWrite(lightLedPin, HIGH);
      } else {
        digitalWrite(lightLedPin, HIGH);
        delay(1000L);
        digitalWrite(lightLedPin, LOW);
      }
      light_FLAG = 0; // Light is detected.

      if (lightIndicator == 0) {
        String motionValue = String(motion_FLAG); 
        String lightValue = String(lightMeter.readLightLevel());
        sendUDP("P", motionValue, "L", lightValue, "H", String(0));  
      }
      lightIndicator = 1;
    } else if (lightMeter.readLightLevel() < 5) { // Light is off.
      digitalWrite(lightLedPin, LOW);
      light_FLAG = 0;
      
      if (lightIndicator == 1) {
        String motionValue = String(motion_FLAG); 
        String lightValue = String(lightMeter.readLightLevel());
        sendUDP("P", motionValue, "L", lightValue, "H", String(0));   
      }
      lightIndicator = 0;
    }
  }
    
    // Enter power down state for 4s with ADC and BOD module disabled
    LowPower.powerDown(SLEEP_4S, ADC_OFF, BOD_OFF);  
    lightWakeUp(); 
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
}

/*
 * Set the light_FLAG to 1 when the interruption for the LIGHT sensor was produced. 
 */
void lightWakeUp()
{
  light_FLAG = 1; 
  HBTimerCounter++;
  checkForControlSignal(); 
}

/*
 * Set the motion_FLAG to 1 when the interruption for the PIR sensor was produced.
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
  if(HBTimerCounter >= HBInterval){     
    String motionValue = String(motion_FLAG); 
    String lightValue = String(lightMeter.readLightLevel());
    sendUDP("P", motionValue, "L", lightValue, "H", String(1));   
    HBTimerCounter = 0;
  }
}