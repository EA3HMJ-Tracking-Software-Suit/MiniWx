# MiniWx
Mini weather station for ControllerDish

Interface based on the ESP32 S2 Mini microcontroller and the ESP32 S2 Mini library [eModbus](https://github.com/eModbus/eModbus).

It behaves as a modbus-RTU client with addresses 3  at 57600 bauds and responding to function 3 and 6.

Registers:
- 0:  Sky temperature
- 1:  Ambient temperature
- 2:  Pressure			
- 3: Humidity
- 4:  Barometer temperature

All values are multiplied by 10 at the origin.
