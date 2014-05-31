Baseflight
========

STM32 F3 DEV version for the NAZE32Pro

Work :
* pwm 10 out/ pwm in 8
* serial rx : spket sat
* ppm rx
* usb cli/msp
* gps nmea (rx) 


Test Me:
* sbus 
* sumd
* gps ublox (rx/tx)

Fix Me :
* i2c on rc 3/4 pin 
* telemetry uart , dont work , but gps do work on both uart
* msp on core.telemport for bluez/433mhz
* something is wrong with baro reading .. stop working after calib ? 
* startup.s will reboot to dfu with gcc , keil startup.s  will reset/reboo
* systemReset , mag always work , mpu6050 work only 50% of time , can reproduce : always
* make target=NAZE make hex  , but hex crash .. worth fixing f103 build ?
* telemetry