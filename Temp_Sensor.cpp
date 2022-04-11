#include "Temp_Sensor.h"

//#if (filterInput)
//SimpleKalmanFilter kf = SimpleKalmanFilter(e_mea, e_est, q);
//#else
//do nothing
//#endif

Temp_Sensor::Temp_Sensor(bool sensorType)
{
  _sensorType = sensorType;
}

float Temp_Sensor::readTemp(void)
{
    float Vout = 0.0;
    float Rt = 0.0;
    float adc = 0.0;
    float adc_estimated_value = 0.0;
#ifdef KalmanFilter
    SimpleKalmanFilter kf = SimpleKalmanFilter(e_mea, e_est, q);
    adc_estimated_value = kf.updateEstimate(adc);
#else
    for  (byte n = 0; n < Lavg_sample_size; n++)
    {
      adc += analogRead(ThermistorPin);
      delay (2);
    }
    adc_estimated_value = adc / Lavg_sample_size;
#endif

    Vout = adc_estimated_value * Vs / adcMax;
    Rt = R1 * Vout / (Vs - Vout);
    T = 1 / (1 / To + log(Rt / Ro) / Beta); // Temperature in Kelvin
    Tc = T - 273.15;                   // Celsius
    Tf = Tc * 9 / 5 + 32;              // Fahrenheit
  return Tc;
}
