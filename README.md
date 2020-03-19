# RaspberryPi_Robot-Vehicle-with-Track-Simulation
Raspberry Pi and C programming: Learn SPI interface and the LSM9DS1 motion-sensing system-ina-chip, sensor input, autonomous robot car driving, I/O ports, PWM, timing, and user interface.

# Demo Link
https://youtu.be/Nhd-zg5ZOgE

# Instructions
tar -xvzf hw6wang.tgz\
cd hw6wang\
make\
sudo ./hw6wang\

# m1 – Manual Control Mode
 (1) Stop: ' s '\
 (2) Forward: ' f ' or ‘w’\
 (3) Backward: ' b ' or ‘x’\
 (4) Faster: ' i '\
 (5) Slower: ' j '\
 (4) Left: ' l ' or ‘a’\
 (5) Right: ' r ' or ‘d’\
 (6) Quit: ' q ' to quit all program\
 (7) Mode chg: ' m2 ' to change mode to m2\
 (8) display the IMU data with “p” command\
 (9) display the computed total distance and average speed the car traveled with “n” command\
 (10) display the map of the car traveled path with “m” command
 
# m2 – Line Tracing Self-Driving Mode
 (1) Stop: ' s ' to pause the line tracing until next Forward command\
 (2) Forward: ' f ' to start the line tracing\
 (3) Quit: ' q ' to quit all program\
 (4) Mode chg: ' m1 ' to change mode to m1\
 (5) display the IMU data with “p” command\
 (6) display the computed total distance and average speed the car traveled with “n” command\
 (7) display the map of the car traveled path with “mm” command
