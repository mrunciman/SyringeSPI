#include "syringePump.h"


SyringePump::SyringePump(uint8_t CS_Pin, uint8_t pressPin):
  settingsSPI(1000000, MSBFIRST, SPI_MODE0)
{
    selectPin = CS_Pin;
    pinMode(selectPin, OUTPUT);
    digitalWrite(selectPin, HIGH);
    pressurePin = pressPin;
    pinMode(pressPin, INPUT);
}



float SyringePump::readPressure()
{
  vRead = analogRead(pressurePin);
  vOut = vRead * ADC_V;
  pressureRead = PSI_TO_KPA*(vOut - 0.1*vSupply)*(pMax - pMin)/(0.8*vSupply) - 0*pressureBaseline;
  // Serial.println("Pump");
  return pressureRead;
}



float SyringePump::filterPressure(float pressIn)
{
  float filtered = pressIn;
  // First take out any big jumps in position
  if (abs(pressurePrev - filtered) > 250){
    filtered = pressurePrev;
  }

  // Then stay between max and min
  if (filtered < PRESS_MIN)  {
    filtered = pressurePrev;
  }
  else if (filtered > PRESS_MAX){
    filtered = PRESS_MAX;
  }
  
  // Update previous value
  pressurePrev = filtered;

  // Output 
  return filtered;
}



void SyringePump::sendRecvFloat(int pinSS, dataFloat *outData, dataFloat *inData)
{
  SPI.beginTransaction(settingsSPI);
  // take the select pin low to activate buffer
  digitalWrite(pinSS, LOW);
  
  //Send first byte and discard last byte that was sent (lastByte)
  // transfer first sends data on MOSI, then waits and receives from MISO
  firstByte = SPI.transfer(firstOut);
  delayMicroseconds(microDelay);

  firstByte = SPI.transfer(outData->bData[0]);
  delayMicroseconds(microDelay);
  for (int i = 0; i < 4; i++){
    if (i < 3){
      // Send bytes 2 - 4 and receive bytes 1 -3
      inData->bData[i] = SPI.transfer(outData->bData[i+1]);
    }
    else if (i == 3){
      // Send placeholder last byte and receive byte 4
      inData->bData[i] = SPI.transfer(lastOut);
      delayMicroseconds(microDelay);
    }
    delayMicroseconds(microDelay); // delay between transmissions
  }
  // take the select pin high to de-select the chip:
  digitalWrite(pinSS, HIGH);
  SPI.endTransaction();
}


