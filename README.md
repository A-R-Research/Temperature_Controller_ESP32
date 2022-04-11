# Temperature_Controller_ESP32
 Code for temperature controller that will be used in the ARHot_Plate project.
## Note
Forked from Temperature_Controller_ESP32 main branch. This is the simplified code which only allows for use of thermistor for temperature sensing, digital pin (software PWM) for SSR control, and constant PID tuning parameters.
# Installation
 Simply unzip the folder in the Arduino sketch directory (installation of external libraries are not required as they are already included in the \src directory).
# Usage
 After unzipping the folder. Open the "Temperature_Controller_ESP32.ino" file in Arduino IDE. Next, edit the User_Config.h file as per the instructions in it to ensure that all the parameters are set as per your hardware setup.
# Acknowledgements
 -PID controller is based on QuickPID Library by dlloydev (https://github.com/Dlloydev/QuickPID)
 -Kalman filter is based on SimpleKalmanFilter library by Denys Sene (https://github.com/denyssene/SimpleKalmanFilter)
# Author
 -Rex Smith
