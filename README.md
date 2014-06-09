#Baseflight - STM32 F3 DEV version for the NAZE32Pro#


##Polish me##
these things work and are tested

* 1khz imu and default pid values
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
these things need to be tested

* sbus 
* sumd
* airplane


##Fix Me##
* detect acc sensors range
* led1 is not present
* i2c on rc 3/4 pin, need configuration for offest rcpin
* vbat monitor
* softserial
* non msp telemetry
* startup.s will reboot to dfu with gcc , keil startup.s  will reset/reboot
* build target=NAZE , remove thing ?
* all the things


## pins usage##
* af = feature af , alternate function for the pin  
* cam = feature servo_tilt, camera stabilisation

af|cam|type|mot1|mot2|mot3|mot4|mot5|mot6|rc1|rc2|rc3|rc4|rc5|rc6|rc7|rc8
--|-------|----|----|----|----|----|----|----|---|---|---|---|---|---|---|---
||multi pwmrx|mot1|mot2|mot3|mot4|mot5|mot6|rc1|rc2|rc3|rc4|rc5|rc6|rc7|rc8
|x|multi pwmrx|mot1|mot2|mot3|mot4|cam|cam|rc1|rc2|rc3|rc4|rc5|rc6|rc7|rc8
x|x|multi pwmrx|mot1|mot2|mot3|mot4|cam|cam|rc1|rc2|rc3|rc4|rc5|rc6|rc7|rc8
||Multi serialrx|mot1|mot2|mot3|mot4|mot5|mot6|mot7|mot8|mot9|mot10|mot11|mot12|mot13|mot14
|x|Multi serialrx|mot1|mot2|mot3|mot4|cam|cam|mot5|mot6|mot7|mot8|servo1|servo2|servo3|servo4
x|x|Multi serialrx|mot1|mot2|mot3|mot4|cam|cam|mot5|mot6|mot7|mot8|mot9|mot10|mot11|mot12
||Multi ppm|mot1|mot2|mot3|mot4|mot5|mot6|ppm|mot6|mot7|mot8|mot9|mot10|mot11|mot12
|x|Multi ppm|mot1|mot2|mot3|mot4|cam|cam|ppm|mot5|mot6|mot7|mot8|mot9|mot10|mot11
x|x|Multi ppm|mot1|mot2|mot3|mot4|cam|cam|ppm|mot6|mot7|mot8|servo1|servo2|servo3|servo4
||tri pwmrc|mot1|mot2|mot3|mot4|mot5|servo tail|rc1|rc2|rc3|rc4|rc5|rc6|rc7|rc8
|x|tri pwmrc|cam|cam|mot1|mot2|mot3|servo tail|rc1|rc2|rc3|rc4|rc5|rc6|rc7|rc8
x|x|tri pwmrc|cam|cam|mot1|mot2|mot3|servo tail|rc1|rc2|rc3|rc4|rc5|rc6|rc7|rc8
||tri serialrx|mot1|mot2|mot3|mot4|mot5|servo tail|mot6|mot7|mot8|mot9|mot10|mot11|mot12|mot13
|x|tri serialrx|cam|cam|mot1|mot2|mot3|servo tail|mot4|mot5|mot6|mot7|mot8|mot9|mot10|mot11
x|x|tri serialrx|cam|cam|mot1|mot2|mot3|servo tail|mot4|mot5|mot6|mot7|servo1|servo2|servo3|servo4
||tri ppm|mot1|mot2|mot3|mot4|mot5|servo tail|ppm|mot6|mot7|mot8|mot9|mot10|mot11|mot12
|x|tri ppm|cam|cam|mot1|mot2|mot3|servo tail|ppm|mot4|mot5|mot6|mot7|mot8|mot9|mot10
x|x|tri ppm|cam|cam|mot1|mot2|mot3|servo tail|ppm|mot4|mot5|mot6|servo1|servo2|servo3|servo4