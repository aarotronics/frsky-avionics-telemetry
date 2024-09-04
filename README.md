# FrSky Basic Avionics Telemetry

Open source Arduino based FrSky basic avionics telemetry system for RC
aircrafts. Arduino uses a GB-1803 GPS module, a BMP280 digital barometer and a
resistor divider filtered with a capacitor; to obtain navigation data, altitude
data and battery voltage data. Using [FrSkySportTelemetry](https://github.com/cody82/FrSkySportTelemetry.git) library, Arduino emulates
FrSky sensors and display all data in the transmitter LCD. [Adafruit_BMP280_Library](https://github.com/adafruit/Adafruit_BMP280_Library.git) and
[FrSkySportTelemetry](https://github.com/cody82/FrSkySportTelemetry.git) libraries need to be modificated in order to work properly with
BMP280 running through I2C.



## Wiring

![Voltage divider](/images/wiring.png)



## Code

This project uses following libraries:

 - [FrSkySportTelemetry](https://github.com/cody82/FrSkySportTelemetry.git) by cody82
 - [TinyGPSPlus](https://github.com/mikalhart/TinyGPSPlus) by mikalhart
 - [TimerOne](https://github.com/PaulStoffregen/TimerOne) by PaulStoffregen
 - [Adafruit_BMP280_Library](https://github.com/adafruit/Adafruit_BMP280_Library.git) by adafruit

In **FrSkySportSensorRpm.cpp** change:
```c++
void FrSkySportSensorRpm::setData(uint32_t rpm, float t1, float t2)
{
	rpmData = rpm*2;
	t1Data = (int32_t)round(t1);
	t2Data = (int32_t)round(t2);
}
```
to:
```c++
void FrSkySportSensorRpm::setData(uint32_t rpm, float t1, float t2)
{
	rpmData = rpm;
	t1Data = (int32_t)round(t1);
	t2Data = (int32_t)round(t2);
}
```

As we're going to use virtual rpm sensor to parse some other data.
