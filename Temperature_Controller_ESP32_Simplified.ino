#include "Temp_Sensor.h"
#include <String.h>
float Input = 0.0;
float Output = 0.0;
float Setpoint = 70;
float rampTemp = 70;    //Temperature Setpoint in (degC)
float rampTime = 60;    //Rate of chage of temperature
float inputOld = 0.0, inputNew = 0.0;
unsigned long sensorReadInterval = 0.0;
unsigned long msBefore = 0, msBefore1 = 0 , seconds = 0;
bool tempInc = true;
unsigned int fanSpeed = 0;

const int SetpointGap = 15;
const int PIDCutOff = 5;
float consKp = 1.1;      //0.2*Ku, w/ Ku = 5
float consKi = 0.095;    //0.4*Ku/Tu, w/ Tu = 80 sec
float consKd = 17;
float aggKp = 1.01;
float aggKi = 0.95;
float aggKd = 9;
char* tuningParam[] = {"0.0", "0.0", "0.0"};
const unsigned long windowSize = 1000;
const byte debounce = 0;
unsigned long windowStartTime = 0, nextSwitchTime = 0;
boolean relayStatus = false;
QuickPID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd,
               myPID.pMode::pOnError,
               myPID.dMode::dOnMeas,
               myPID.iAwMode::iAwCondition,
               myPID.Action::direct);

const int FanPWMFreq = 30000; /* 5 KHz */
const int FanPWMChannel = 0;
const int FanPWMResolution = 8;

Temp_Sensor tempProbe = Temp_Sensor(TemperatureProbeType);

void setup()
{
  Serial.begin(115200);
  initOutput();
  Input = tempProbe.readTemp();
  sensorReadInterval = windowSize;
  //myPID.SetTunings(Kp, Ki, Kd);
  myPID.SetOutputLimits(0, windowSize);
  myPID.SetSampleTimeUs((windowSize / 2) * 1000);

#ifdef MATLABout
  char str[50];
  bool tuningWait = true;
  while (tuningWait)
  {
    int c = 0;
    while (Serial.available() > 0)
    {
      str[c++] = Serial.read();
      tuningWait = false;
      str[c] = '\0';
    }
    if (!tuningWait)
    {
      //Serial.print("Recieved String:"); Serial.println(str);
      int i = 0;
      char * pch = strtok (str, "{,}");        // get first parameter
      //Serial.print("Token: "); Serial.print(pch); Serial.print(", Index: "); Serial.println(i);
      while (pch != NULL  && i < 3)
      {
        //Serial.print("Token: "); Serial.print(pch); Serial.print(", Index: "); Serial.println(i);
        tuningParam[i++] = pch;
        pch = strtok (NULL, "{,}");     // get next parameter
        delay(1);   //feed wdt
      }
    }
    delay(1);   //feed wdt
  }
  //Serial.print("Kp: "); Serial.println( atof(tuningParam[0])); Serial.print("Ki: "); Serial.println( atof(tuningParam[1])); Serial.print("Kd: "); Serial.println( atof(tuningParam[2]));
  float Kp = atof(tuningParam[0]);
  float Ki = atof(tuningParam[1]);
  float Kd = atof(tuningParam[2]);
  myPID.SetTunings(Kp, Ki, Kd);
  myPID.SetMode(myPID.Control::automatic);
#else
  delay(1000); //for stabality
  myPID.SetMode(myPID.Control::automatic);
#endif
}

void loop()
{
  unsigned long msNow = millis();
  Input = tempProbe.readTemp();
  //  if (msNow - msBefore >= sensorReadInterval+1000)
  //  {
  //    if (inputNew - Input > 0)
  //    {
  //      tempInc = true;
  //    }
  //    else
  //    {
  //      tempInc = false;
  //    }
  //    inputNew = Input;
  //    msBefore = msNow;
  //  }
#ifdef MATLABout
  if (Serial.available() > 0)
  {
    printDebug(Serial.read());
  }
#else
  Serial.print("SetPoint_Temp(degC):"); Serial.print(Setpoint); Serial.print(",");
  Serial.print("Actual_Temp(degC):"); Serial.print(Input); Serial.print(",");
  Serial.print("ControllerOutput(ms):"); Serial.print(Output); Serial.print(",");
  Serial.print("Kp:"); Serial.print(myPID.GetKp()); Serial.print(",");
  Serial.print("Ki:"); Serial.print(myPID.GetKi()); Serial.print(",");
  Serial.print("Kd:"); Serial.print(myPID.GetKd()); Serial.print(",");
  Serial.print("Fan:"); Serial.print(fanSpeed);

  Serial.println(";");
  Serial.write('\r');
#endif
  if (myPID.Compute())
  {
    windowStartTime = msNow;
  }
  if (Input < Setpoint)
  {
    if ((Setpoint - Input) <= 15 && (Setpoint - Input) > 5)
    {
      myPID.SetOutputLimits(0, 80);
      fanSpeed = 127;
    }
    else if ((Setpoint - Input) <= 5)
    {
      myPID.SetOutputLimits(0, 50);
      fanSpeed = 255;
    }
    else
    {
      myPID.SetOutputLimits(0, windowSize);
      fanSpeed = 0;
    }
  }
  else if (Input > Setpoint)
  {
    if ((Input - Setpoint) <= 1.5)
    {
      myPID.SetOutputLimits(0, 20);
      fanSpeed = 100;
    }
    else if ((Input - Setpoint) > 1.5 && (Input - Setpoint) <= 2)
    {
      myPID.SetOutputLimits(0, 20);
      fanSpeed = 127;
    }
    else if ((Input - Setpoint) > 2)
    {
      myPID.SetOutputLimits(0, 10);
      fanSpeed = 255;
    }
    else
    {
      // myPID.SetOutputLimits(0, windowSize);
      fanSpeed = 0;
    }
  }
  //    if ((Input >= Setpoint - 13) && (Input <= Setpoint + 5))
  //    {
  //      myPID.SetOutputLimits(0, 80);
  //    }
  //    if ((Input >= Setpoint - 5) && (Input <= Setpoint + 1.5))
  //    {
  //      myPID.SetOutputLimits(0, 20);
  //    }
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
  if (msNow - msBefore1 >= 1000)
  {
    inputOld = inputNew;
    msBefore1 = msNow;
    seconds++;
    fan(fanSpeed);
  }
}

void initOutput(void)
{
  pinMode(SSR_Pin, OUTPUT);
  ledcSetup(FanPWMChannel, FanPWMFreq, FanPWMResolution);
  ledcAttachPin(FanPWMPin, FanPWMChannel);
  ledcWrite(FanPWMChannel, 0);
}

void fan(int Speed)
{
  ledcWrite(PWMChannel, Speed);
}

void printDebug(char s)
{
  if (s == '1')
  {
    Serial.print("SetPoint_Temp(degC):"); Serial.print(Setpoint); Serial.print(",");
    Serial.print("Actual_Temp(degC):"); Serial.print(Input); Serial.print(",");
    Serial.print("ControllerOutput(ms):"); Serial.print(Output); Serial.print(",");
    Serial.print("Kp:"); Serial.print(myPID.GetKp()); Serial.print(",");
    Serial.print("Ki:"); Serial.print(myPID.GetKi()); Serial.print(",");
    Serial.print("Kd:"); Serial.print(myPID.GetKd()); Serial.print(",");
    Serial.print("Fan:"); Serial.print(fanSpeed);

    Serial.println(";");
    Serial.write('\r');
  }
  else if (s == 'r')
  {
    ESP.restart();
  }
  else
  {
    Serial.write('\r');
  }
}
