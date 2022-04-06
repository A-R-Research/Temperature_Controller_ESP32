#include "Temp_Sensor.h"

float Input = 0.0;
float Output = 0.0;
float Setpoint = 90;    //Temperature Setpoint in (degC)

#ifdef DACout                 //Set these gains if DAC is selected
float Kp = 0.35;
float Ki = 0.6;
float Kd = 3;
#else                       //Else set these gains if PWM is selected
float Kp = 0.9;
float Ki = 1.6;
float Kd = 10;
#endif

QuickPID myPID(&Input, &Output, &Setpoint);

Temp_Sensor tempProbe = Temp_Sensor(TemperatureProbeType);

void setup()
{
  Serial.begin(115200);
  initOutput();
  Input = tempProbe.readTemp();
  
  myPID.SetTunings(Kp, Ki, Kd);
#ifdef DACout
  myPID.SetOutputLimits(51, 53);    //from the DAC pin, value 51 is 0.66v (0 VAC from SSR), 52 is 0.67v (120 VAC from SSR), 53 is 0.69v (240 VAC from SSR)
#else
  myPID.SetOutputLimits(141, 1023); //from PWM pin, value 141 is 0 VAC from SSR and 1023 is 240 VAC from SSR. The voltage is controlled mostly linearly in between (this needs to be tested!).
#endif

  myPID.SetMode(myPID.Control::automatic);
  delay(1000); //for stabality
}

void loop()
{
  Input = tempProbe.readTemp();
  myPID.Compute();
#ifdef DACout
  dacWrite(DAC1, Output);
#else
  ledcWrite(PWMChannel, Output);
#endif
  Serial.print("SetPoint_Temp(degC):"); Serial.print(Setpoint); Serial.print(",");
  Serial.print("Actual_Temp(degC):"); Serial.print(Input); Serial.print(",");

#ifdef DACout
  Serial.print("ControllerOutput(V):");  Serial.print((float)((supplyVoltage  / 255)*Output));
#else
  Serial.print("ControllerOutput(V):");  Serial.print((float)((supplyVoltage / MAX_DUTY_CYCLE )*Output));
#endif

  Serial.println();
}

void initOutput(void)
{
#ifdef DACout
  dacWrite(DAC1, 0);
#else
  ledcSetup(PWMChannel, PWMFreq, PWMResolution);
  ledcAttachPin(PWMPin, PWMChannel);
  ledcWrite(PWMChannel, 0);
#endif
}
