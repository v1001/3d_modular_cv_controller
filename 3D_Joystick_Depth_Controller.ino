//#define SERIAL_DEBUG
#define USE_HW_SPI

// Include the required Arduino libraries:
#include "MD_Parola.h"
#include "MD_MAX72xx.h"
#include "SPI.h"
#include <Wire.h>
#include <Adafruit_MCP4725.h>

// Hardware SPI:
#define HARDWARE_TYPE MD_MAX72XX::FC16_HW
#define MAX_DEVICES 1
#define CS_PIN 2
#define DATA_PIN 11
#define CLK_PIN 13

// Create a new instance of the MD_MAX72XX class:
#ifdef USE_HW_SPI
MD_MAX72XX myDisplay = MD_MAX72XX(HARDWARE_TYPE, CS_PIN, MAX_DEVICES);
#else
MD_MAX72XX myDisplay = MD_MAX72XX(HARDWARE_TYPE, DATA_PIN, CLK_PIN, CS_PIN, MAX_DEVICES);
#endif

Adafruit_MCP4725 dacX, dacY;

int xPos = 2048;
int yPos = 2048;

int switchState = 0;
int switchValue = 0;

uint64_t cycleCount = 0;
uint64_t lastSwitchCycle = 0;

void setup() {
  // Intialize the object:
  myDisplay.begin();
  // Clear the display:
  myDisplay.clear();
  // initialize serial communication at 115200 bits per second:
  #ifdef SERIAL_DEBUG
  Serial.begin(115200);
  #endif
  // initialize DAC
  dacX.begin(0x60);
  dacY.begin(0x61);
  // set state LEDs
  digitalWrite(4,1);
  digitalWrite(5,0);
  digitalWrite(6,0);
  //set digital and analog pin modes
  pinMode(12, INPUT);

}

uint8_t intensityFromDepth(int depth){
  if(depth>612){return 0xf;}
  else if(depth>575){return 0x0e;}
  else if(depth>537){return 0x0d;}
  else if(depth>500){return 0x0c;}
  else if(depth>462){return 0x0b;}
  else if(depth>425){return 0x0a;}
  else if(depth>387){return 0x09;}
  else if(depth>350){return 0x08;}
  else if(depth>312){return 0x07;}
  else if(depth>275){return 0x06;}
  else if(depth>237){return 0x05;}
  else if(depth>200){return 0x04;}
  else if(depth>162){return 0x03;}
  else if(depth>125){return 0x02;}
  else if(depth>87){return 0x01;}
  else{return 0x00;}
} 

void nextState(){
  if(switchState == 0){
    switchState = 1;
    digitalWrite(4,0);
    digitalWrite(5,1);
    digitalWrite(6,0);
  }else if(switchState == 1){
    switchState = 2;
    digitalWrite(4,0);
    digitalWrite(5,0);
    digitalWrite(6,1);
  }else if(switchState == 2){
    switchState = 0;
    digitalWrite(4,1);
    digitalWrite(5,0);
    digitalWrite(6,0);
  }else{
    switchState = 0;
    digitalWrite(4,1);
    digitalWrite(5,0);
    digitalWrite(6,0);
  }
  lastSwitchCycle = cycleCount;
}

// the loop routine runs over and over again forever:
void loop() {
  // icrement cycle counter
  cycleCount += 1;

  // read joystick switch
  int switchActive = digitalRead(12);
  if((switchActive && !switchValue)&&(cycleCount - lastSwitchCycle > 10)){
    nextState();
  }
  switchValue = switchActive;

  // read X value, update X position
  int sensorValueX = analogRead(A1);
  int xIncrement = (sensorValueX - 512) / 32;
  xPos += xIncrement;
  xPos = (xPos > 4095) ? 4095 : xPos;
  xPos = (xPos < 0) ? 0 : xPos; 

  // read Y value, update Y position
  int sensorValueY = analogRead(A3);
  int yIncrement = (sensorValueY - 512) / 32;
  yPos += yIncrement;
  yPos = (yPos > 4095) ? 4095 : yPos;
  yPos = (yPos < 0) ? 0 : yPos;

  // read Z value
  int sensorValueZ = analogRead(A2);

  //update brightness
  uint8_t depthIntensity = intensityFromDepth(sensorValueZ);
  myDisplay.control(0x2, depthIntensity);

  //update cursor
  uint8_t sensorRow = sensorValueY >> 7;
  uint8_t sensorCol = 7 - (sensorValueX >> 7);
  if((cycleCount >> 6) & 0x00000001){
    myDisplay.setRow(sensorRow, 0xFF ^ (1 << sensorCol));
    myDisplay.setColumn(sensorCol, 0xFF ^ (1 << sensorRow));
  }else{
    myDisplay.setRow(sensorRow, 0x00 ^ (1 << sensorCol));
    myDisplay.setColumn(sensorCol, 0x00 ^ (1 << sensorRow));
  }

  //update pixel
  uint8_t pointRow = yPos >> 9;
  uint16_t  pointCol = xPos >> 9;
  myDisplay.setPoint(pointRow, 7 - pointCol, true);

  //update dac
  dacX.setVoltage(4095-xPos, false);
  dacY.setVoltage(yPos, false);
   
  #ifdef SERIAL_DEBUG
  //print out the value you read:
  Serial.print(sensorValueX);
  Serial.print(',');
  Serial.print(sensorValueY);
  Serial.print(',');
  Serial.println(sensorValueZ);
  //print out the value of pixel:
  Serial.print(xPos);
  Serial.print(',');
  Serial.println(yPos);
  delay(4);
  #else
  delay(5);
  #endif

  myDisplay.setPoint(pointRow, 7 - pointCol, false);
  myDisplay.setRow(sensorValueY >> 7, 0x00);
  myDisplay.setColumn(7 - (sensorValueX >> 7), 0x00);
}
