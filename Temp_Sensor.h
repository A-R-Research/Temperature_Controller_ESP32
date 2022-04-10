/******************************************************************
  @file    Controller.h
  @brief  Library for a temperature PID Controller
  @author  Rex Smith

  Code:        Rex Smith
  Version:     1.0.1
  Date:        03/04/2022

  1.0.0 Support for Thermistor sensor                       31/03/22
  1.0.1 Support for MAX6675                                   03/04/22

******************************************************************/

#ifndef Temp_Sensor_h
#define Temp_Sensor_h

#include "User_Config.h"
#include "src/QuickPID.h"
#include "src/SimpleKalmanFilter.h"
#include "Arduino.h"

class Temp_Sensor {
  public:
    Temp_Sensor(bool);
    float readTemp(void);

  private:
    byte getSPI(void);
    bool _sensorType = false;
    unsigned long timeNow = 0;
    float T = 0.0;
    float Tc = 0.0;
    float Tf = 0.0;

};

#endif
