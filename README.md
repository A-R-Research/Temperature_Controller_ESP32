# Temperature_Controller_ESP32
 Code for temperature controller that will be used in the ARHot_Plate project.
# Installation
 Simply unzip the folder in the Arduino sketch directory (installation of external libraries are not required as they are already included in the \src directory).
# Usage
 After unzipping the folder. Open the "Temperature_Controller_ESP32.ino" file in Arduino IDE. Next, edit the User_Config.h file as per the instructions in it to ensure that all the parameters are set as per your hardware setup.
# Acknowledgements
 -PID controller is based on QuickPID Library by dlloydev (https://github.com/Dlloydev/QuickPID)
 -Kalman filter is based on SimpleKalmanFilter library by Denys Sene (https://github.com/denyssene/SimpleKalmanFilter)
 -Implementation of thermocouple code in Temp_Sensor.cpp is based on Adafruit's MAX6675 library (https://github.com/adafruit/MAX6675-library)
# Author
 -Rex Smith
