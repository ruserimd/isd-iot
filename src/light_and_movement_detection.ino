#include "BH1750.h"
#include <SPI.h>        
#include <Ethernet.h>
#include <EthernetUdp.h>
#include "LowPower.h"

#define lightLedPin 8
#define motionLedPin 9
#define pirPin 2
#define pirSensorCalibrationTime 10  // Calibration time for PIR sensor, 10-60ms according to datasheet.
int HBInterval = 2;                  // Depends of SLEEP_XS parameter in the LowPower.powerDown(SLEEP_4S, ADC_OFF, BOD_OFF) function.
                                     // X * HBInterval = seconds after that the control message is sent.

BH1750 lightMeter;

byte motion_sent = 0;
byte light_FLAG = 0;
byte motion_FLAG = 0;
byte HBTimerCounter = 0;
byte lightIndicator = 0;

// The MAC address, IP address and the port to listen for the controller. They will be dependent on local network.
byte mac[] = { 0x90, 0xA2, 0xDA, 0x0F, 0xB6, 0x47 };
IPAddress ip(172, 17, 41, 54);
unsigned int localPort = 9876;

// The IP address and the port of the server.
IPAddress remoteIP(172, 17, 41, 85);
unsigned int remotePort = 9876;

// Buffer to hold incoming packet from the network.
char packetBuffer[UDP_TX_PACKET_MAX_SIZE]; 

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

  delay(50L); 
}

/**
 * Give the sensor some time to calibrate. 10-60ms according to datasheet.
 */
void calibratePIRSensor() {
  for(int i = 0; i < pirSensorCalibrationTime; i++){
    delay(1000L);
  }
}

/**
 *  At bootup, flash LED 3 times quickly so I know the reboot has occurred.
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

/**
 * Setup device's IP, MAC and local port.
 */
void setupEthernet() {
  // Start the Ethernet and UDP.
  Ethernet.begin(mac, ip);
  Udp.begin(localPort);
}

void loop(){    

  readUDP();
  lightWakeUp(); 
  HBTimerCounter++;
  checkForControlSignal();

  // Check for motion detection.
  // If the motion_FLAG is set, the Arduino board send a UDP message to the server.
  // The message is sent only once, and not for the entire period as it detects motion. 
  // Also it turns on the two LEDs showing that motion is detected.
  if (motion_FLAG == 1) {      
    if(digitalRead(pirPin) == HIGH) {
     digitalWrite(motionLedPin, HIGH);
     digitalWrite(lightLedPin, HIGH);
      if (motion_sent == 0) {
        String motionValue = String(motion_FLAG); 
        String lightValue = String(lightMeter.readLightLevel());
        sendUDP("P", motionValue, "L", lightValue, "H", String(0));   
        motion_sent = 1;    
      }
    }

    // If the PIR sensor doesn't detect motion any more, the Arduino board sends a UDP message to the server. 
    // Now the motion_FLAG is reset.
    // Also it turns off the two LEDs showing that motion is no more detected.
    if(digitalRead(pirPin) == LOW) {       
     digitalWrite(motionLedPin, LOW);
     digitalWrite(lightLedPin, LOW);                                 
       motion_FLAG = 0;
       motion_sent = 0;
       String motionValue = String(motion_FLAG); 
       String lightValue = String(lightMeter.readLightLevel());
       sendUDP("P", motionValue, "L", lightValue, "H", String(0));  
    }
  }

  // Check for light detection.
  // The Arduino board sends a UDP message when the level of the light in the room suddenly changes, 
  // ie when the light turns off or on. 
  if (light_FLAG == 1) {    
    if (lightMeter.readLightLevel() > 5) { // Light is on.
      if (motion_FLAG == 1) {
        digitalWrite(lightLedPin, HIGH);
      } else {
        digitalWrite(lightLedPin, HIGH);
        delay(500L);
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

    // Enter power down state for 4s with ADC and BOD module disabled.
    LowPower.powerDown(SLEEP_4S, ADC_OFF, BOD_OFF);      
}

/*
 * Send UDP message from Arduino board to the server.
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
  Udp.beginPacket(remoteIP, remotePort);
  Udp.print(messageToServer);
  Udp.endPacket();
}

/**
 * Send an OK message to server, containing the valeu "1", showing that heartbeat frequency has successfully changed.
 */
void sendOK() {
  Udp.beginPacket(remoteIP, remotePort);
  Udp.print(1);
  Udp.endPacket();
}

/*
 * Set the light_FLAG to 1 when the interruption for the LIGHT sensor was produced. 
 */
void lightWakeUp() {
  light_FLAG = 1;  
}

/*
 * Set the motion_FLAG to 1 when the interruption for the PIR sensor was produced.
 */
void motionWakeUp() {
  motion_FLAG = 1;
}

/*
 * If any sensor has not changed after the period of time specified by @HBInterval, 
 * the Arduino board sends to the server a control message with the last data from sensors, showing that 
 * the device works.
 */
void checkForControlSignal() {
  if(HBTimerCounter >= HBInterval) {     
    String motionValue = String(motion_FLAG); 
    String lightValue = String(lightMeter.readLightLevel());
    sendUDP("P", motionValue, "L", lightValue, "H", String(1));   
    HBTimerCounter = 0;
  }
}

/**
 * Read the incoming message from network and send back an OK message.
 */
void readUDP() {
  int packetSize = Udp.parsePacket();
  if(packetSize) {
    Udp.read(packetBuffer,UDP_TX_PACKET_MAX_SIZE); // Read the packet into packetBufffer.
    HBInterval = atoi(&packetBuffer[0]);           // Set the incomming data to @HBInterval.
  
    sendOK(); 
  }
}