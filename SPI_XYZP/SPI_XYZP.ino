///////////////////////////////////////////////////////
// include the SPI library:
#include <SPI.h>
#include "syringePump.h"

// set pin 10 as the slave select for the X axis, 20 for Y axis
const byte selectPinX = 9;
const byte selectPinY = 8;
const byte selectPinZ = 10;
const byte selectPinP = 11;

const byte pressPinX = A0;
const byte pressPinY = A2;
const byte pressPinZ = A3;
const byte pressPinP = A3;


// SyringePump LHS(selectPinX, pressPinX);
// SyringePump RHS(selectPinY, pressPinY);
// SyringePump TOP(selectPinZ, pressPinZ);
// SyringePump PRI(selectPinP, pressPinP);

SyringePump pumpList[4];
pumpList[0] = SyringePump(selectPinX, pressPinX);
pumpList[1] = SyringePump(selectPinY, pressPinY);
pumpList[2] = SyringePump(selectPinZ, pressPinZ);
pumpList[3] = SyringePump(selectPinP, pressPinP);


////////////////////////////////////////////////////////
// Stepper variables

//For step count:
// int STEPS_PER_MM = 400;
// int prevMotorState = 0;
// int motorState = 0;
// int stepCount = 0;
// String stepRecv;
//
// int prevStepIn = 0;
// char stepRecv[7] = "00000";
// char byteRead = 0;
// int availableBytes = 0;
// long int stepIn = 0;
// int stepError = 0;

///////////////////////////////////////////////////////
unsigned long timeSinceStep = 0;
unsigned long timeNow;
unsigned long timeAtStep;
unsigned long writeTime;


////////////////////////////////////////////////////////
// Messages
char data[54]; // Char array to write stepNo, pressure and time into
char endByte = 'E';
// char disableMsg[3] = "D ";
// char limitHit[3] = "L ";

////////////////////////////////////////////////////////
// Actuator geometry to determine max steps
float STROKE = 25.0; //Eq. triangle length in mm
float L0 = STROKE/(1.0 - 2.0/PI); // flat length for contraction = STROKE
float ACT_WIDTH = 30.0; // width of muscle in mm
float NUM_L = 3; // number of subdivisions in muscle
float A_SYRINGE = PI*pow(13.25, 2.0); // piston area in mm^2
float FACT_V = (ACT_WIDTH*pow(L0 , 2.0))/(2.0*NUM_L);
float MAX_V = FACT_V*(2.0/PI); // volume in mm^3 when fully actuated
// steps to fill actuator
// int maxSteps = 5000; //((MAX_V/A_SYRINGE)*STEPS_PER_MM); 
// int minSteps = 0;


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


  // std::list<SyringePump> listOfPumps = 
}


/////////////////////////////////////////////////////////

// // Communicate with control system on computer
// void writeSerial(char msg){
//   writeTime = millis();
//   data[0] ='\0';
//   if (msg == 'S'){ // Normal operation, send stepCount etc
//     sprintf(data, "%06d,%d,%lu%s\r\n", stepCount, int(pressureAbs*10), writeTime, endByte);
//   }
//   else if (msg == 'D'){ // Python cut off comms, acknowledge this
//     sprintf(data, "%s%s,%d,%lu%s\r\n", disableMsg, shakeKey, int(pressureAbs*10), writeTime, endByte);
//   }
//   else if (msg == 'L'){ // Limit switch hit, advise Python
//     sprintf(data, "%s%s,%d,%lu%s\r\n", limitHit, shakeKey, int(pressureAbs*10), writeTime, endByte);
//   }
//   else if (msg == 'p'){ // Calibrating
//     sprintf(data, "%06d%s,%d,%lu%s\r\n", STABLE_TIME-stateCount, , int(pressureAbs*10), writeTime, endByte);
//   }
//   else if(msg = 'P'){ // Calibration finished
//     sprintf(data, "%06d%s,%d,%lu%s\r\n", stepCount, shakeKey, int(pressureAbs*10), writeTime, endByte);
//   }
//   Serial.write(data);
// }

void writeSerial(){

  // Write angle and pressure data to control computer.
  sprintf(data, "%f,%f,%f,%f,%d,%d,%d,%lu,%c\n", LHS.angleIn, RHS.angleIn, TOP.angleIn, PRI.angleIn, int(LHS.pressureRead*10), int(RHS.pressureRead*10), int(TOP.pressureRead*10), writeTime, endByte);
  Serial.write(data);
}



int setState(SyringePump& pumpObject){
  if (pumpObject.calibrated == false){
    pumpObject.pumpState = 0;
  }
  else if (pumpObject.calibrated){//some other condition
    pumpObject.pumpState = 1;
  }

}




///////////////////////////////////////////////////////////

void loop() {



  for(auto &item : pumpList){
    // item.method();
    int state = item.pumpState;
    switch (state)
    {
    case 0/* constant-expression */:
      /* code */
      break;
    
    default:
      break;
    }
  }
  // std::list<SyringePump>::iterator pumpObject; // put in setup()?
  // // Make iterate point to begining and incerement pumpObject one by one till pumpObject reaches the end of list.
  // for (pumpObject = listOfPumps.begin(); pumpObject != listOfPumps.end(); pumpObject++)
  // {
  //     // Access the object through iterator
  //     int state = pumpObject->pumpState;
     
  // }



  // Send desired positions to steppers
  Serial.print("Desired X position: ");
  Serial.println(LHS.dataOut.fData);
  LHS.sendRecvFloat(LHS.selectPin, &LHS.dataOut, &LHS.dataIn);
  Serial.print("Measured angle X: ");
  Serial.println(LHS.dataOut.fData);
 

  Serial.print("Desired Y position: ");
  Serial.println(RHS.dataOut.fData);
  RHS.sendRecvFloat(RHS.selectPin, &RHS.dataOut, &RHS.dataIn);
  Serial.print("Measured angle Y: ");   
  Serial.println(RHS.dataOut.fData);


  Serial.print("Desired Z position: ");
  Serial.println(TOP.dataOut.fData);
  TOP.sendRecvFloat(TOP.selectPin, &TOP.dataOut, &TOP.dataIn);
  Serial.print("Measured angle Y: ");   
  Serial.println(TOP.dataOut.fData);

  writeSerial();
  // Serial.println();
  
  delay(1000);

}