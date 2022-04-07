#include "Temp_Sensor.h"

//#if (filterInput)
//SimpleKalmanFilter kf = SimpleKalmanFilter(e_mea, e_est, q);
//#else
//do nothing
//#endif

Temp_Sensor::Temp_Sensor(bool sensorType)
{
  _sensorType = sensorType;
#ifdef MAX6675
  pinMode(ThermocoupleCS, OUTPUT);
  pinMode(ThermocoupleSCK, OUTPUT);
  pinMode(ThermocoupleSO, INPUT);
  digitalWrite(ThermocoupleCS, HIGH);
  Serial.println("Thermocouple Pins set!");
#else
  //do nothing
#endif
}

float Temp_Sensor::readTemp(void)
{
#ifdef MAX6675
  if (millis() >= timeNow + thermoReadDelay_ms)
  {
    uint16_t spi = 0;
    digitalWrite(ThermocoupleCS, LOW);
    delayMicroseconds(10);
    spi = getSPI();
    spi <<= 8;
    spi |= getSPI();
    digitalWrite(ThermocoupleCS, HIGH);
    if (spi & 0x4)  //Thermocouple not found!
    {
      return NAN;
    }
    spi >>= 3;
    Tc = spi * 0.25;
    T = Tc + 273.15;
    Tf = Tc * 9 / 5 + 32;
    timeNow += thermoReadDelay_ms;
  }
  return Tc;
#else
  float Vout, Rt = 0;
  float adc, adc_estimated_value = 0;
  //  adc = adc / 10;
#ifdef KalmanFilter
  SimpleKalmanFilter kf = SimpleKalmanFilter(e_mea, e_est, q);
  adc_estimated_value = kf.updateEstimate(adc);
#else
  for  (byte n = 0; n < Lavg_sample_size; n++)
  {
    adc += analogRead(ThermistorPin);
    delay (2);
  }
  adc_estimated_value = adc/Lavg_sample_size;
#endif
  Vout = adc_estimated_value * Vs / adcMax;
  Rt = R1 * Vout / (Vs - Vout);
  T = 1 / (1 / To + log(Rt / Ro) / Beta); // Temperature in Kelvin
  Tc = T - 273.15;                   // Celsius
  Tf = Tc * 9 / 5 + 32;              // Fahrenheit
  return Tc;
#endif
}

#ifdef MAX6675
byte Temp_Sensor::getSPI(void)
{
  byte spiData = 0;
  for (int i = 7; i >= 0; i--)
  {
    digitalWrite(ThermocoupleSCK, LOW);
    delayMicroseconds(10);
    if (digitalRead(ThermocoupleSO))
    {
      spiData |= (1 << i);
    }
    digitalWrite(ThermocoupleSCK, HIGH);
    delayMicroseconds(10);
  }
  return spiData;
}
#else
//do nothing
#endif
