#include "Temp_Sensor.h"

float Input = 0.0;
float Output = 0.0;
float Setpoint = 0.0;
float rampTemp = 70;    //Temperature Setpoint in (degC)
float rampTime = 60;    //Rate of chage of temperature
float inputOld = 0.0, inputNew = 0.0;
unsigned long sensorReadInterval = 0.0;
unsigned long msBefore = 0, msBefore1 = 0 , seconds = 0;

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
const int SetpointGap = 15;
const int PIDCutOff = 5;
float consKp = 0.5;
float consKi = 0.1;
float consKd = 0.9;
float aggKp = 1.1;
float aggKi = 0.95;
float aggKd = 9;
const unsigned long windowSize = 100;
const byte debounce = 0;
unsigned long windowStartTime = 0, nextSwitchTime = 0;
boolean relayStatus = false;
QuickPID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd,
               myPID.pMode::pOnErrorMeas,
               myPID.dMode::dOnMeas,
               myPID.iAwMode::iAwClamp,
               myPID.Action::direct);
#endif

Temp_Sensor tempProbe = Temp_Sensor(TemperatureProbeType);

void setup()
{
  Serial.begin(115200);
  initOutput();
  Input = tempProbe.readTemp();
  sensorReadInterval = windowSize;
  //myPID.SetTunings(Kp, Ki, Kd);
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
  unsigned long msNow = millis();
  if (msNow - msBefore >= sensorReadInterval)
  {
    inputNew = Input;
    Input = tempProbe.readTemp();
    msBefore = msNow;
  }

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
#ifdef ConstantSetpoint
  Setpoint = rampTemp;
#else
  if (Input < rampTemp || seconds <= rampTime)
  {
    Setpoint = seconds * (rampTemp / rampTime);                //Reach 90ºC till 60s (90/60=1.5)
    if (Setpoint >= rampTemp)
    {
      Setpoint = rampTemp;
    }
  }
#endif
  float gap = abs(Setpoint - Input); //distance away from setpoint
  if(gap <= PIDCutOff)
  {
    myPID.SetMode(myPID.Control::manual);
  }
  else
  {
    myPID.SetMode(myPID.Control::automatic);
  }
  if (abs(inputNew - inputOld) >= 0.3 || gap < SetpointGap)
  { //we're close to setpoint, use conservative tuning parameters
    myPID.SetTunings(consKp, consKi, consKd);
  }
  else
  {
    //we're far from setpoint, use aggressive tuning parameters
    myPID.SetTunings(aggKp, aggKi, aggKd);
  }
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
  Serial.print("ControllerOutput(ms):"); Serial.print(Output); Serial.print(",");
  Serial.print("Kp:"); Serial.print(myPID.GetKp()); Serial.print(",");
  Serial.print("Ki:"); Serial.print(myPID.GetKi()); Serial.print(",");
  Serial.print("Kd:"); Serial.print(myPID.GetKd());
#endif

  Serial.println();
  if (msNow - msBefore1 >= 1000)
  {
    inputOld = inputNew;
    msBefore1 = msNow;
    seconds++;
  }
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
