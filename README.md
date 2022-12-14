# RemoteLuxController
## Description
- Language: Arduino
- Target Board: Arduino Uno
- Shield: Ethernet Shield
- Sensor: TSL2561 I2C Light Lux Sensor
## I/O
- A4: SDL -> TSL2561
- A5: SCL -> TSL2561
- D3: PWM -> LED Driver (Inverted Logic)
## Developement
- Phase.1
To achieve LED controll
1. Create LED driver circuit. (2SC1815 NPN Tr, TK49N65W5 Nch MOSFET)
2. Test analog output and amplified output.
3. PWM parameter can be controlled by serial port
- Phase.2
To achieve fetching environment light lux from the sensor
1. Create sensor circuit. (Adafruit TSL2561 module)
2. Use sample sketch to get Lux value using I2C bus.
- Phase.3
1. Create system architecture and implement blanc functions.
2. Program dummy program to connect Lux sensor to LED driver. (anything to change LED power depending on Lux value)
- Phase.4
1. Program PID controller
2. Calcurate acceptable PID paramters.
3. Program order input using serial port.
3. Complete LED Controller using PID.
- Phase.5
1. Test Ethernet Shield.
2. Program Web server.
3. Program API over HTTP.
4. Test HTTP/API controll.
- Phase.6
1. Connect HTTP/API to Lux order input.
2. Complete Web controlled LED controller.
- Phase.7
TBD
## Note
- LED Driver
Our LED Driver is driven in Inverted Logic (Duty 10 -> Duty 90) because it has inverter.