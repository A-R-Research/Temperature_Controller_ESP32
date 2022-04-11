#include "Temp_Sensor.h"

float Input = 0.0;
float Output = 0.0;
float Setpoint = 0.0;
float rampTemp = 70;    //Temperature Setpoint in (degC)
float rampTime = 60;    //Rate of chage of temperature
float inputOld = 0.0, inputNew = 0.0;
unsigned long sensorReadInterval = 0.0;
unsigned long msBefore = 0, msBefore1 = 0 , seconds = 0;

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

Temp_Sensor tempProbe = Temp_Sensor(TemperatureProbeType);

void setup()
{
  Serial.begin(115200);
  initOutput();
  Input = tempProbe.readTemp();
  sensorReadInterval = windowSize;
  //myPID.SetTunings(Kp, Ki, Kd);
  myPID.SetOutputLimits(0, windowSize);
  myPID.SetSampleTimeUs(windowSize * 1000);

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

  pinMode(SSR_Pin, OUTPUT);
}
