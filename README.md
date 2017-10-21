# AltiDuo
Atmel ATmega328 firmware for the AltiDuo a dual deployment rocket altimeter board.
A standard dual altimeter that fire a charge at apogee and a second one 50, 100,150 or 200 meters before reaching the ground.
It also has some standard features such as charges continuity test and beeping the apogee altitude.
This is an Arduino compatible board so you can use the [Arduino](https://www.arduino.cc/) framework to modify the program and hence customize your altimeter.
<img src="/pictures/altiDuo_kit.jpg" width="49%">

# Building the code
You will need to download the Arduino ide from the [Arduino web site](https://www.arduino.cc/). 
You have to load the Arduino Uno boot loader to your ATmega328 micro controller. 
Make sure that you download the [support library](https://github.com/bdureau/AltimetersLibs) for the BMP085 sensor and copy them to the Arduino library folder. To compile it you need to choose the Arduino Uno board and the correct USB port.
You will need to use a USB/TTL adapter to connect the altimeter to your computer, refer to the documentation.
<img src="/pictures/ttl_cable.JPG" width="49%"> <img src="/pictures/cp2102-.jpg" width="49%">

# Modifying the firmware
Note that this kit is very convenient if you want to learn how the firmware works. It is very easy to upload the firmware. 

# Hardware
If you do not have the board, you could even run that code without any modifications on an Arduino Uno, nano or pro. You will just need to attach a BMP085 or BMP180 pressure sensor to your Arduino board and few other componants. 

