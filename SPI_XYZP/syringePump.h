#include <SPI.h>


class SyringePump{

    float vRead = 0.0;
    float vOut = 0.0;
    float vSupply = 5.0;
    float pMax = 150.0;
    float pMin = 0.0;

    //Convert from psi to kPa
    float PSI_TO_KPA = 6.89476;
    // conversion multiplier from Arduino ADC value to voltage in mV
    const float ADC_V = 0.0048828125;

    int PRESS_MAX = 2500;
    int PRESS_MIN = 400;

    int microDelay = 50;


    // Actuator geometry to determine max steps
    int STEPS_PER_MM = 400;
    float STROKE = 25.0; //Eq. triangle length in mm
    float L0 = STROKE/(1.0 - 2.0/PI); // flat length for contraction = STROKE
    float ACT_WIDTH = 30.0; // width of muscle in mm
    float NUM_L = 3; // number of subdivisions in muscle
    float A_SYRINGE = PI*pow(13.25, 2.0); // piston area in mm^2
    float FACT_V = (ACT_WIDTH*pow(L0 , 2.0))/(2.0*NUM_L);
    float MAX_V = FACT_V*(2.0/PI); // volume in mm^3 when fully actuated
    int maxSteps = 5000; //((MAX_V/A_SYRINGE)*STEPS_PER_MM); 
    int minSteps = 0;


///////////////////////////////////////////////////////////////////////////
public:
    SyringePump(uint8_t CS_Pin, uint8_t pressPin);
    uint8_t selectPin;
    uint8_t pressurePin;
    
    SPISettings settingsSPI;

    /////////////////////////////////////////////////////////////////////////
    // Variables for pressure and position data

    union dataFloat{
        float fData;
        uint8_t bData[4];
    };

    dataFloat dataIn;
    dataFloat dataOut;

    float angleIn;

    uint16_t pressureDes; 
    float pressureRead = 0.0;
    float pressureBaseline = 0.0;
    float pressurePrev;

    int stepCount = 0;
    int motorState = 0;
    int prevMotorState = 0;

    /////////////////////////////////////////////////////////////////////
    // State setting variables
    bool calibrated = false;
    bool active = false;
    int pumpState = 0;
    //Use start and end bytes to set state on pumps/peripherals
    char firstOut = '<';
    char lastOut = '>';

    byte firstByte = 0;
    byte lastByte = 255;

    ////////////////////////////////////////////////////////////////////////////
    // Functions

    float readPressure();
    float filterPressure(float pressIn);
    void sendRecvFloat(int pinSS, dataFloat *outData, dataFloat *inData);

};