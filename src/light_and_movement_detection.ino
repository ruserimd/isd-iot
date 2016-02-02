#include <Wire.h>

int LightSensor_address = 0x23; // i2c Addresse
byte buff[2];

int lightLedPin = 12;
int motionLedPin = 13;

void setup(){
  
  Wire.begin();
  LightSensor_Init(LightSensor_address);
  
  delay(200);
  Serial.begin(9600);
  Serial.println("Light Sensor Started");
  Serial.println("Motion Sensor Started");

  // Set input & output pin for movement detection.
  pinMode(motionLedPin,OUTPUT);
  pinMode(2,INPUT);

  // Set output pin for light detection.
  pinMode(lightLedPin,OUTPUT);
}

void loop(){
   
  float valf=0;

  // Check if movement is detected.
  if (digitalRead(2)==1){
    Serial.println("Movement detected.");
    digitalWrite(13,digitalRead(2));
    delay(1000);
    digitalWrite(13,LOW);
  }

  // Check if light is on or of
  if(LightSensor_Read(LightSensor_address)==2){    
   valf=((buff[0]<<8)|buff[1])/1.2;

   if(valf>10) {
     Serial.println("Light is on.");
     digitalWrite(lightLedPin,HIGH);      
   } else if (valf<=0) {
     Serial.println("Light is off.");
     digitalWrite(lightLedPin,LOW);
    }
  }
  delay(1000);
}

/* Begin a transmission to the I2C slave device with the given address. 
 * Subsequently, queue bytes for transmission with the write() function 
 * and transmit them by calling endTransmission(). 
 * address: the 7-bit address of the device to transmit to
 */
void LightSensor_Init(int address){
  
  Wire.beginTransmission(address);
  Wire.write(0x10); // 1 [lux] aufloesung
  Wire.endTransmission();
}

/* Used by the master (Arduino) to request data (bytes) from a slave device (Light Sensor). 
 * address: the 7-bit address of the device to transmit to
 * return: the number of bytes returned from the slave device 
 */
byte LightSensor_Read(int address){
  
  byte i=0;
  Wire.beginTransmission(address);
  Wire.requestFrom(address, 2);
  while(Wire.available()){
    buff[i] = Wire.read(); 
    i++;
  }
  Wire.endTransmission();  
  return i;
}
