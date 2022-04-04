#include "Arduino.h"
#define Thermistor false      //Do not change!
#define Thermocouple true  //Do not change!
#define DAC true               //Do not change!
#define PWM false             //Do not change!

//-----WARNING!!! Set the parameters below correctly before flashing this code------------------------------
//Either "DAC" or "PWM" can be used for controlling the SSR.
#define outputType PWM
//Set to "true" to use a simple Kalman Filter for the temperature probe input
#define filterInput false      
//Simply change "Thermistor" to "Thermocouple" to swap the type of temperature sensor used 
#define TemperatureProbeType Thermocouple
//-----------------------------------------------------------------------------------------------------------------------------

//--------------------Nothing to Change here----------------------------------------------------------------------------
#if (outputType)
#define DACout
#else
#define PWMout
#endif

#if (TemperatureProbeType)
#define MAX6675
#else
#define ADC
#endif

#if (filterInput)
#define KalmanFilter
#else
#define LinearAverage
#endif
//-----------------------------------------------------------------------------------------------------------------------------

//----------You can change some pin definations below if required------------------------------------------------
#ifdef DACout
#define DAC1 25                                         //DAC Channel 1 (can't be changed)
#define DAC2 26                                         //DAC Channel 2 (can't be changed)
#define SSR_Pin DAC1
#else
#define PWMPin 25                                     //PWM pin(can be changed to any pin from 0 to 34)
#define SSR_Pin PWMPin
#endif

#ifdef MAX6675
#define ThermocoupleSO 19
#define ThermocoupleCS 23
#define ThermocoupleSCK 5
#else
#define ThermistorPin 34
#define adcMax 4095
#define Vs 3.3
#endif

#define incPin 15                                     //Pin used to increase the output voltage of the DAC(can be changed to anyother input pin)
#define decPin 4                                      //Pin used to decrease the output voltage of the DAC(can be changed to anyother input pin)
#define supplyVoltage 3.3                         //Use 3.3 if directly measuring output from one of the DAC pins or set to whatever the supply/source voltage you use (for example, if you use a transitor connected to 5V then set to 5)
#define updateDelay_ms 100                    //This is simply the delay of the loop function in milliseconds. Increase/decrease to speed up the rate of change of voltage when increase or decrease buttons are pressed.

//-----------------------------------------------------------------------------------------------------------------------------

//----------You can change PWM parametes below if required-----------------------------------------------------
/* Setting PWM Properties */
const int PWMFreq = 5000; /* 5 KHz */
const int PWMChannel = 0;
const int PWMResolution = 10;
const int MAX_DUTY_CYCLE = (int)(pow(2, PWMResolution) - 1);

//-----------------------------------------------------------------------------------------------------------------------------


const float R1 = 9915;   // voltage divider resistor value
const float Beta = 3950;  // Beta value
const float To = 301.15;    // Temperature in Kelvin for 27 degree Celsius
const float Ro = 100000;   // Resistance of Thermistor at 25 degree Celsius

//-----------------------------------------------------------------------------------------------------------------------------
//Below values are for Kalman Filter
#ifdef KalmanFilter
const float e_mea = 2.0;
const float e_est = e_mea;
const float q = 0.01;
#else
const uint8_t Lavg_sample_size = 10;
#endif
