# SD-IG42DB-Robot
Software for SuperDroid Programmable Mecanum Wheel Vectoring Robot - IG42 DB project

# Goal
Learn Arduino and NVidia Jetson Nano platforms using a SuperDroids IG42 DB platform as a base. 

# Current Status
Initial movement control via RadioLink AT10 RC controller completed. 

Several resources were helpful for this step:

     SparkFun's Arduino RC control tutorial: helped with my initial polled mode for reading the RC signals.
     https://www.youtube.com/watch?v=u0Ft8SB3pkw 
     
     
     Kamran Ahmad's PWM library is used to move from polled to interrupt RC signal reading.
     https://github.com/xkam1x/Arduino-PWM-Reader
     
     SuperDroid example code for methods of mixing RC signals for Mecanum wheel control and control of their DC motor drivers
     
# Hardware
  Superdroids IG42-DB tobot chassis
  
  Mega 2560 Arduino
  
  RadioLink AT10 II controller with R12DS Receiver
  
