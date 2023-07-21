# Voltiris_angle

## Set up the MPU-92500 for angle measurement

1) Wire the MPU-92500, Arduino Nano and the NRF24L01 module correctly together.
    - Use the Pinconfiguration_angledetection.pdf to get a picture.
    - Teh emiter module is with the MPU-92500 and the reciver module is the same but without the MPU-92500.

2) Calibrate the accelerometer with Accel_calib.ino.
    - Falsh the code onto the Arduino nano asembled in the final box.
    - Put the box on a perfectly flat surface (Use a Wasserwage to check if its flat)
    - Open the Serial monitor of the Arduino IDE
    - You shoud see every half secod a printout of the variables:
    - AccX_cal, AccY_cal, AccZ_cal, AccX, AccY, AccZ,   AccXcalib, AccYcalib, AccZcalib
    - AccN_cal  =The calibrated acceleration value int he Nth direction.
    - AccN      =The raw data directly out of the accelerometer of the Nth direction without any calibartion.
    - AccNcalib =The calibration value used to calibrate AccN_cal int he Nth direction.
    - Chose one line whre AccX_cal, AccY_cal, AccZ_cal are all 0 (or very close).
    - Set copy the calibration values AccXcalib, AccYcalib, AccZcalib of this line into Yaw_Pitch_Roll_emiter.ino.
3) Calibrate the Magnetometer
    - Is not working well fort the Yaw measurement therefore not explaind
4) Setup the reciver and emiter module for gatering the angle data toogether.
    - Flash the code of Yaw_Pitch_Roll_emiter.ino to the module witch is used as an emitter.
    - Flash the code of Yaw_Pitch_Roll_reciver.ino to the module witch is used as an reciver.
    - Open serial monitor on the Arduino IDE of the reciver to observe the recive angle data. 

