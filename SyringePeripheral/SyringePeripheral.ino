
#include <uStepperS.h>
#include <Wire1.h>
#include <SparkFun_MS5803_TwoI2C.h>


/////////////////////////////////////////////////////////////////
// SPI and data 
int SS_Pin = 3;

union dataFloat{
  float fData;
  uint8_t bData[4];
};

volatile dataFloat posEncoder;
volatile dataFloat posFromMega;

volatile int posIndex = 0;
volatile uint8_t firstByte = 0;
volatile uint8_t lastByte = 0;

volatile bool process_it = false;

// volatile bool startMessage = false; 
// volatile bool endMessage = false; 

int filtAngle = 0;
int prevAngle = 0;

int filtPress = 0;
int prevPress = 0;

/////////////////////////////////////////////////////////////////////////
// Time variables

unsigned long timeSinceStep = 0;
unsigned long timeNow;
unsigned long timeAtStep;

//////////////////////////////////////////////////////////////////////
// State setup

int pumpState = 0;
bool uncalibratedFlag = true;
bool disconFlag = false;

////////////////////////////////////////////////////////////////////
// Pressure and calibration
MS5803 sensor(ADDRESS_LOW);
double pressureAbs = 1000.00; // Initial value
int PRESS_THRESH = 10;//mbar
int PRESS_MAX = 2500;
int PRESS_MIN = 400;
int PRESS_FINE = 50; // When within this threshold, use finer movements
double pressSetpoint = 900.00;//mbar
int pressureError;
int PRESS_STEPS_FINE = 15;
int PRESS_STEPS_LG = 30;
int pressSteps = PRESS_STEPS_LG; // Num steps to move when calibrating

////////////////////////////////////////////////////////
// Calibration
int stateCount = 0;
int startTime;
int STABLE_TIME = 4000; // time in milliseconds reqd for pressure to be at calibration setpoint
bool lowFlag = false;
bool highFlag = true;
int prevMotorState = 0;
int motorState = 0;


////////////////////////////////////////////////////////
// Stepper variables
uStepperS stepper;
int STEPS_PER_MM = 400; // Higher steps per mm value for ustepper s lite than old setup (old = 100 steps/mm)
int stepCount = 0;
int MIN_ANGLE = 2;
int MAX_ANGLE = 1000; // Initial guess - needs calculation


////////////////////////////////////////////////

void setup(void)
{
  noInterrupts();
  Serial.begin(115200);
  stepper.setup(CLOSEDLOOP,200);     //Initiate the stepper object to use closed loop control with 200 steps per revolution motor - i.e. 1.8 deg stepper 
  // Initialise data structures
  posEncoder.fData = 655.4; //stepper.encoder.getAngleMoved();
  //  Serial.println(posEncoder.fData);
  posFromMega.fData = 0.0;
  
  // For the closed loop position control the acceleration and velocity parameters define the response of the control:
  stepper.setMaxAcceleration(2000);   //use an acceleration of 2000 fullsteps/s^2
  stepper.setMaxVelocity(800);        //Max velocity of 800 fullsteps/s
  
  stepper.checkOrientation(5.0);      //Check orientation of motor connector with +/- 30 microsteps movement
  stepper.setControlThreshold(15);    //Adjust the control threshold - here set to 15 microsteps before making corrective action

  stepper.moveSteps(51200);           //Turn shaft 51200 steps, counterClockWise (equal to one revolution with the TMC native 1/256 microstepping)

  ///////////////////////////////////////////
  // have to send on controller in, peripheral out
  pinMode(MISO0, OUTPUT);
  pinMode(SS0, INPUT);
  pinMode(SS_Pin, INPUT);
  
  // turn on SPI in peripheral mode
  SPCR0 = 0;
  SPCR0 |= (1<<SPIE0)|(1<<SPE0)|(0<<DORD0)|(0<<MSTR0)|(0<<CPOL0)|(0<<CPHA0)|(0<<SPR01)|(0<<SPR00);
  SPSR0 &= ~(0<<SPI2X0);
  
  interrupts();
}

//////////////////////////////////////////////////////////////////////


// SPI interrupt routine
ISR (SPI_STC_vect){
  if (digitalRead(SS_Pin) == LOW){
    if (prevSelectState == 1){
      firstByte = SPDR0;
      prevSelectState = 0;
    }
    else{
      // Check index
      if (posIndex >= 4){
        lastByte = SPDR0;
        SPDR0 = 0;
        posIndex = 0;
        process_it = true;
      }
      else{
        //Read byte from SPI buffer
        posFromMega.bData[posIndex] = SPDR0;
        
        // Write encoder value to buffer
        SPDR0 = posEncoder.bData[posIndex];
        posIndex++;
      }
    }
  }
}// end of SPI interrupt routine



float angleFilt(float recvdAngle){
  float filtered = recvdAngle;
  // First take out any big jumps in position
  if (abs(prevAngle - filtered) > 250){
    filtered = prevAngle;
  }

  // Then keep between max and min
  if (filtered < MIN_ANGLE)  {
    filtered = prevAngle;
  }
  else if (filtered > MAX_ANGLE){
    filtered = MAX_ANGLE;
  }
  
  // Update previous value
  prevAngle = filtered;

  // Output 
  return filtered;
}




double pressureRead() {
  double pressureIn = sensor.getPressure(ADC_2048);
  // Filter out false readings
  // Stop motor and wait if pressure exceeds maximum
  if (pressureIn > PRESS_MAX){
    // extInterrupt = true;
    disconFlag = false; // Stop and disable
  }
  else if (pressureIn < 0){
    pressureIn = 0.00;
  }
  else if (pressureIn < PRESS_MIN){
    disconFlag = false;
  }
  return pressureIn;
}

void pressInitZeroVol(double pressureIn) {
  //Set state for motor motion based on comparison of pressure signal with setpoint
  pressureError = pressureIn - pressSetpoint;
  prevMotorState = motorState;
  // Assign motor state based on pressure error
  if (pressureIn < pressSetpoint - PRESS_THRESH){ // The pressure is less than setpoint minus threshold
    lowFlag = true;
    if (pressureIn > pressSetpoint - 2*PRESS_THRESH){
      // Pressure is at setpoint minus between 1 and 2 times threshold
      motorState = 0;
      // Increment counter if previous state was also zero
      // Pressure is stable if counter reaches some limit
      if (prevMotorState == 0){
        stateCount = millis() - startTime;
      }
      // Set back to zero if not
      else{
        stateCount = 0;
        startTime = millis();
      }
    }
    else{
      // Pressure too low, move plunger forward
      motorState = 1;
    }
  }
  else { // Pressure is over lower bound
    if (lowFlag == true){
      if (pressureIn <= pressSetpoint + PRESS_THRESH){
        // If we previously reached lower bound and within threshold, stop
        motorState = 0;
        // Increment counter if previous state was also zero
        // Pressure is stable if counter reaches some limit
        if (prevMotorState == 0){
          stateCount = millis() - startTime;
        }
        // Set back to zero if not
        else{
          stateCount = 0;
          startTime = millis();
        }
      }
      else{
        // If we previously reached lower bound but outside of threshold, move back
        motorState = 2;
        // Reset lower bound flag, to reach it again.
        lowFlag = false;
      }
    }
    else{ 
      // If haven't yet reached lower bound, move plunger back
      motorState = 2;
    }
  }

  // If close to target pressure, use finer movements
  if (motorState == 1 || motorState == 2){
    // if within 50 mbar of target pressure go slower
    if (abs(pressureError) < PRESS_FINE){ 
      pressSteps = PRESS_STEPS_FINE;
      // stepper.setMaxVelocity(SPEEDP_LOW);
    }
    else{
      pressSteps = PRESS_STEPS_LG;
      // stepper.setMaxVelocity(SPEEDP_HIGH);
    }
  }

  switch (motorState) {
    case 0:
      // Stop motor
      break;
    case 1:
      //Move motor forwards
      stepper.moveSteps(pressSteps);
      break;
    case 2:
      //Move motor backwards 
      stepper.moveSteps(-pressSteps);
      break;
  }
}



void setPumpState(){
    // Compare start and end bytes, if they are same then
    // use value to determine state

    if (firstByte == lastByte){
        // Use first byte to determine stat as it is more reliable
        if (firstByte == 'd'){ // Disable
            pumpState = 2;
        }
        else if (firstByte == 'p'){ // Calibrating
            pressureAbs = posFromMega.fData;
            pumpState = 3;
        }
        else if (firstByte == 'a'){ // Active
            pumpState = 4;
        }
        else{ // If not one of the above, disable
            pumpState = 2;
        }
    }
    else{ // If the two start and end bytes don't match, then ignore data
        ;
    }
}


float pressFilt(float pressIn){
  float filtered = pressIn;
  // First take out any big jumps in position
  if (abs(prevAngle - filtered) > 250){
    // filtered = prevAngle;
    ;
  }

  // Then stay between max and min
  if (filtered < PRESS_MIN)  {
    filtered = prevAngle;
  }
  else if (filtered > PRESS_MAX){
    filtered = PRESS_MAX;
  }
  
  // Update previous value
  prevAngle = filtered;

  // Output 
  return filtered;
}





//////////////////////////////////////////////////////////////////////////


void loop(void){




if (digitalRead(SS_Pin)==HIGH){
    prevSelectState = 1;
    // startMessage = false;
    // endMessage = false;
    posIndex = 0;
}

// If SPI data has been read, change state accordingly
if (process_it){
    process_it = false;
    setPumpState();
}

  switch(pumpState){
    //////////////////////////////////////////////////////////////////////  
    case 2:

      break;



    //////////////////////////////////////////////////////////////////////
    case 3:

      pressureAbs = posFromMega.fData;
      filtAngle = filtPress(pressureAbs);

      pressInitZeroVol(filtAngle);

      // If enough time has passed, say volume is 0, tell python and move on
      if (stateCount >= STABLE_TIME){
        // Step count should now be zero - muscle empty.
        stepCount = 0;
        stepper.encoder.setHome();
        // Notify that calibration is done
        pressFlag = false;

      }

      break;


    /////////////////////////////////////////////////////////////////////
    case 4:

        posEncoder.fData = stepper.encoder.getAngleMoved();

        // Serial.print("From controller: ");
        // Serial.println(posFromMega.fData);

        // Filter input angle 
        desiredAngle = posFromMega.fData;
        filtAngle = inputFilt(desiredAngle);

        // Make move
        stepper.moveToAngle(filtAngle); 
        
      break;
  }






  timeNow = micros();

  // Update current angular position
  posEncoder.fData = stepper.encoder.getAngleMoved();



}
