#include "Temp_Sensor.h"

float Input = 0.0;
float Output = 0.0;
float Setpoint = 90;    //Temperature Setpoint in (degC)

//-------------------------------------Set these gains if DAC is selected---------------------------------------------
#ifdef DACout                
float Kp = 0.35;
float Ki = 0.6;
float Kd = 3;
QuickPID myPID(&Input, &Output, &Setpoint);

//-------------------------------------Set these gains if PWM is selected--------------------------------------------
#elif defined(PWMout)                      
float Kp = 0.9;
float Ki = 1.6;
float Kd = 10;
QuickPID myPID(&Input, &Output, &Setpoint);

//-------------------------------------Set these gains if Digital is selected-------------------------------------------
#else
float Kp = 0.75;
float Ki = 0.9;
float Kd = 2;
const unsigned long windowSize = 100;
const byte debounce = 10;
unsigned long windowStartTime = 0, nextSwitchTime = 0;
boolean relayStatus = false;
QuickPID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd,
               myPID.pMode::pOnError,
               myPID.dMode::dOnError,
               myPID.iAwMode::iAwCondition,
               myPID.Action::direct);
#endif

Temp_Sensor tempProbe = Temp_Sensor(TemperatureProbeType);

void setup()
{
  Serial.begin(115200);
  initOutput();
  Input = tempProbe.readTemp();

  myPID.SetTunings(Kp, Ki, Kd);
#ifdef DACout
  myPID.SetOutputLimits(51, 53);    //from the DAC pin, value 51 is 0.66v (0 VAC from SSR), 52 is 0.67v (120 VAC from SSR), 53 is 0.69v (240 VAC from SSR)
#elif defined(PWMout)
  myPID.SetOutputLimits(141, 1023); //from PWM pin, value 141 is 0 VAC from SSR and 1023 is 240 VAC from SSR. The voltage is controlled mostly linearly in between (this needs to be tested!).
#else
  myPID.SetOutputLimits(0, windowSize);
  myPID.SetSampleTimeUs(windowSize * 1000);
#endif

  myPID.SetMode(myPID.Control::automatic);
  delay(1000); //for stabality
}

void loop()
{
  Input = tempProbe.readTemp();
  Serial.print("SetPoint_Temp(degC):"); Serial.print(Setpoint); Serial.print(",");
  Serial.print("Actual_Temp(degC):"); Serial.print(Input); Serial.print(",");

#ifdef DACout
  myPID.Compute();
  dacWrite(SSR_Pin, Output);
  Serial.print("ControllerOutput(V):");  Serial.print((float)((supplyVoltage  / 255)*Output));
#elif defined(PWMout)
  myPID.Compute();
  ledcWrite(PWMChannel, Output);
  Serial.print("ControllerOutput(V):");  Serial.print((float)((supplyVoltage / MAX_DUTY_CYCLE )*Output));
#else
  unsigned long msNow = millis();
  if (myPID.Compute())
  {
    windowStartTime = msNow;
  }
  if (!relayStatus && Output > (msNow - windowStartTime))
  {
    if (msNow > nextSwitchTime)
    {
      nextSwitchTime = msNow + debounce;
      relayStatus = true;
      digitalWrite(SSR_Pin, HIGH);
    }
  }
  else if (relayStatus && Output < (msNow - windowStartTime))
  {
    if (msNow > nextSwitchTime)
    {
      nextSwitchTime = msNow + debounce;
      relayStatus = false;
      digitalWrite(SSR_Pin, LOW);
    }
  }
  Serial.print("ControllerOutput(ms):"); Serial.print(Output);
#endif

  Serial.println();
}

void initOutput(void)
{
#ifdef DACout
  dacWrite(SSR_Pin, 0);
#elif defined(PWMout)
  ledcSetup(PWMChannel, PWMFreq, PWMResolution);
  ledcAttachPin(SSR_Pin, PWMChannel);
  ledcWrite(PWMChannel, 0);
#else
  pinMode(SSR_Pin, OUTPUT);
#endif
}
