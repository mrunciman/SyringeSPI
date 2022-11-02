///////////////////////////////////////////////////////
// include the SPI library:
#include <SPI.h>
#include "syringePump.h"

const int ACTIVE = 0;
const int CALIBRATING = 1;

// set pin 10 as the slave select for the X axis, 20 for Y axis
const byte selectPinX = 9;
const byte selectPinY = 8;
const byte selectPinZ = 10;
const byte selectPinP = 11;

const byte pressPinX = A0;
const byte pressPinY = A2;
const byte pressPinZ = A3;
const byte pressPinP = A3;


SyringePump pumpList[4] = {SyringePump(selectPinX, pressPinX), SyringePump(selectPinY, pressPinY), SyringePump(selectPinZ, pressPinZ), SyringePump(selectPinP, pressPinP)};


///////////////////////////////////////////////////////
unsigned long timeSinceStep = 0;
unsigned long timeNow;
unsigned long timeAtStep;
unsigned long writeTime;


////////////////////////////////////////////////////////
// Messages
char data[54]; // Char array to write stepNo, pressure and time into
char endByte = 'E';


//////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(115200);
  
  // set the peripheral select pins as output:
  pinMode(SS, OUTPUT);
  pinMode(MOSI, OUTPUT);
  pinMode(SCK, OUTPUT);
  pinMode(MISO, INPUT);


  // initialize SPI:
  SPI.begin();

  Serial.println("Start");

}


/////////////////////////////////////////////////////////


void writeSerial(){
  // Write angle and pressure data to control computer.
  writeTime = millis();
  data[0] ='\0';
  sprintf(data, "%f,%f,%f,%f,%d,%d,%d,%lu,%c\n", pumpList[0].angleIn, pumpList[1].angleIn, pumpList[2].angleIn, pumpList[3].angleIn, int(pumpList[0].pressureRead*10), int(pumpList[1].pressureRead*10), int(pumpList[2].pressureRead*10), writeTime, endByte);
  Serial.write(data);
}



int setState(SyringePump& pumpObject){
  if (pumpObject.calibrated == false){
    pumpObject.pumpState = CALIBRATING;
  }
  else if (pumpObject.calibrated){//some other condition
    pumpObject.pumpState = ACTIVE;
  }

}




///////////////////////////////////////////////////////////

void loop() {


  // for each item (object) in pumpList
  for(auto &item : pumpList){ 
    int state = item.pumpState;
    item.readPressure();
    switch (state)
    {
      case CALIBRATING:
        item.firstOut = 'p';
        item.lastOut = 'p';
        item.sendRecvFloat(item.selectPin, &item.dataOut, &item.dataIn);
        break;

      case ACTIVE:
        item.firstOut = '<';
        item.lastOut = '>';
        item.sendRecvFloat(item.selectPin, &item.dataOut, &item.dataIn);
        break;
      
      default:
        break;
    }
  }
  

  writeSerial();

  
  delay(10);

}