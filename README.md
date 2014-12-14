#Baseflight -  STM32F3/F4 DEV version#


##Board##
* NazePro STM32f3
* Quanton STM32f4


##Polish me##
these things work and are tested

* external i2c (mag & baro)
* altitude hold
* headfree
* mixer : tri , quad , hexa, guimbal
* camstab 
* buzzer
* motor/servo pwm 14 out
* standard rx pwm 8 chan
* serial rx : spketrum sat
* ppm rx
* usb cli/msp
* bluetooth/433mhz msp on telemport 
* gps nmea (rx) on telemport, rc pin
* gps ublox (rx/tx) on telemport, rc pin
* baro5611 spi 
* mag5983 spi
* mpu6050 spi

##Test Me##
should work ,but untested
* sbus 
* sumd
* airplane


##My Todo##
* quanton mag and baro
* detect acc sensors range
* led1 is not present
* external i2c, need configuration to offest rcpin for pwmrx
* vbat monitor
* softserial
* non msp telemetry
* startup.s will reboot to dfu with gcc , keil startup.s  will reset/reboot
* build target=NAZE , remove thing ?
* all the things

