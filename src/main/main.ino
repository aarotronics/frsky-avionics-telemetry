/*

                                FRSKY AVIONICS TELEMETRY

   Open source Arduino based FrSky basic avionics telemetry system for RC aircrafts.
   Arduino uses a GB-1803 GPS module, a BMP280 digital barometer and a resistor divider
   filtered with a capacitor; to obtain navigation data, altitude data and battery
   voltage data. Using FrSkySportTelemetry library, Arduino emulates FrSky sensors and
   display all data in the transmitter LCD. Adafruit_BMP280_Library and TinyGPS libraries
   need to be modified in order to work properly with BMP280 running through I2C and
   GLONASS GNSS modules.



   Aaron G.
   Apr 2020
*/

// ======== GLOBAL LIBS ==========
#include <TimerOne.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <math.h>


// ========= LOCAL LIBS ==========
#include "TinyGPSPlus.h"
#include "Adafruit_BMP280.h"
#include "FrSkySportSensor.h"
#include "FrSkySportSensorFcs.h"
#include "FrSkySportSensorVario.h"
#include "FrSkySportSingleWireSerial.h"
#include "FrSkySportSensorGps.h"
#include "FrSkySportSensorRpm.h"
#include "FrSkySportTelemetry.h"

// ====== START USER CONFIG ======
#define BATT_PER_CELL               // Show battery voltage per cell instead of total voltage
//#define FROM_SEA_LEVEL              // Show altitude (from sea level, QNE) instead of height (from ground, QFE)
#define MAX_CELL_VOLTS        4.20  // V
#define MIN_CELL_VOLTS        3.30  // V
#define DIVIDER_UPPER_R       6.80  // KOhm
#define DIVIDER_LOWER_R       0.47  // KOhm
#define VOLTAGE_RATE          1.0
#define VOLTAGE_OFFSET        0


// ====== END USER CONFIG ======
#define VOLTAGE_PIN           A0    // Analog pin where voltage sensor is connected
#define EMA_FILTER_ALPHA      0.10  // Amount of the new value over 1.0 that will be added in each filter loop
#define EMA_FILTER_UPDATE     20    // ms filter loop time
#define MAX_ADC               1023  // 10 bit ADC
#define ADC_AREF              1.10  // V from ATMEGA328P internal AREF
#define GPS_SERIAL            Serial
#define VSPD_SAMPLES          40
#define VSPD_MAX_SAMPLES      50
#define UPDATE_DELAY          10000     // FrSky SmartPort update period (us)
#define LED_PIN               13        // Status LED, will turn ON after start-up when system is ready to go
#define SEA_PRESSURE          101325    // Default sea pressure for QNE operation (Pa)


Adafruit_BMP280         baroSensor;
TinyGPSPlus             gpsSensor;
FrSkySportSensorFcs     fcsFrSky;
FrSkySportSensorGps     gpsFrSky;
FrSkySportSensorVario   varioFrSky;
FrSkySportSensorRpm     rpmFrSky;
FrSkySportTelemetry     telemetry;


uint8_t cellNum;
uint16_t filteredADC = 0;
float batteryVoltage, cellVoltage, batteryPercent;
uint32_t lastBatteryRead = 0;
int hdopValue;
float gpsLatitude, gpsLongitude, gpsSpeed, gpsCourse, altitudeGPS;
float actualPressure, referencePressure, baroAltitude, verticalSpeed, baroTemp;
float tempo = millis();
float N1 = 0, N2 = 0, N3 = 0, D1 = 0, D2 = 0;
float alt[51];
float tim[51];


void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  delay(100);
  analogReference(INTERNAL); // Set analog reference to ATMega328P internal 1V1
  GPS_SERIAL.begin(57600);
  telemetry.begin(FrSkySportSingleWireSerial::SOFT_SERIAL_PIN_3, &fcsFrSky, &gpsFrSky, &varioFrSky, &rpmFrSky);
  baroSensor.begin(0x76);
  baroSensor.setSampling(Adafruit_BMP280::MODE_NORMAL,    // Operating Mode
                         Adafruit_BMP280::SAMPLING_X16,   // Temp oversampling
                         Adafruit_BMP280::SAMPLING_X16,   // Pressure oversampling
                         Adafruit_BMP280::FILTER_X16,     // Filtering
                         Adafruit_BMP280::STANDBY_MS_1);  // Standby time
  delay(1000);
  baroAltitude = 0;

#ifdef FROM_SEA_LEVEL
  referencePressure = SEA_PRESSURE;

#else
  referencePressure = 0;
  for (int i = 0; i < 200; i++) { // Read ground pressure for QFE operation
    referencePressure += baroSensor.readPressure();
    delayMicroseconds(1000);
  }
  referencePressure = referencePressure / 200.0;
#endif

  delay(1000);
  batteryVoltage = (((((float)analogRead(VOLTAGE_PIN) / (float)MAX_ADC * ADC_AREF) * ((float)DIVIDER_UPPER_R + (float)DIVIDER_LOWER_R)) / (float)DIVIDER_LOWER_R) * VOLTAGE_RATE) + VOLTAGE_OFFSET;
  if (batteryVoltage <= 17.50) // Get number of cells from total voltage
    cellNum = 4;
  if (batteryVoltage <= 12.70)
    cellNum = 3;
  if (batteryVoltage <= 8.50)
    cellNum = 2;

  Timer1.initialize(UPDATE_DELAY);        // Interruption used to send data to FrSky SmartPort
  Timer1.attachInterrupt(SmartPort_ISR);
  interrupts();
  digitalWrite(LED_PIN, HIGH);            // Turn LED ON when ready to go
}


void loop() {


  // Voltage
  if (millis() >= (lastBatteryRead + EMA_FILTER_UPDATE)) {
    filteredADC = (uint16_t)((EMA_FILTER_ALPHA * analogRead(VOLTAGE_PIN)) + ((1.0 - EMA_FILTER_ALPHA) * filteredADC));
    batteryVoltage = (((((float)filteredADC / (float)MAX_ADC * ADC_AREF) * ((float)DIVIDER_UPPER_R + (float)DIVIDER_LOWER_R)) / (float)DIVIDER_LOWER_R) * VOLTAGE_RATE) + VOLTAGE_OFFSET;
    cellVoltage = batteryVoltage / (float)cellNum;
    batteryPercent = constrain((((cellVoltage - MIN_CELL_VOLTS) / (MAX_CELL_VOLTS - MIN_CELL_VOLTS)) * 100.0), 0.0, 100.0);
    lastBatteryRead = millis();
  }


  // GPS
  while (GPS_SERIAL.available()) {
    gpsSensor.encode(GPS_SERIAL.read());
  }

  if (gpsSensor.location.isValid()) {
    gpsLatitude = gpsSensor.location.lat();
    gpsLongitude = gpsSensor.location.lng();
  } else {
    gpsLatitude = 0.0000000;
    gpsLongitude = 0.0000000;
  }

  if (gpsSensor.speed.isValid()) {
    gpsSpeed = gpsSensor.speed.mps();
  } else {
    gpsSpeed = 0.0;
  }

  if (gpsSensor.altitude.isValid()) {
    altitudeGPS = gpsSensor.altitude.meters();
  } else {
    altitudeGPS = 0.0;
  }

  if (gpsSensor.course.isValid()) {
    gpsCourse = gpsSensor.course.deg();
  } else {
    gpsCourse = 0.0;
  }

  hdopValue = gpsSensor.hdop.hdop();


  // Barometer
  actualPressure = baroSensor.readPressure();
  baroAltitude = 44330 * (1.0 - pow(actualPressure / referencePressure, 0.190284));
  //baroAltitudeFt = 145366.45 * (1.0 - pow(actualPressure / referencePressure, 0.190284));
  baroTemp = baroSensor.readTemperature();
  tempo = millis();
  N1 = 0;
  N2 = 0;
  N3 = 0;
  D1 = 0;
  D2 = 0;
  verticalSpeed = 0.0;
  for (int cc = 1; cc <= VSPD_MAX_SAMPLES; cc++) {
    alt[(cc - 1)] = alt[cc];
    tim[(cc - 1)] = tim[cc];
  }
  alt[VSPD_MAX_SAMPLES] = baroAltitude;
  tim[VSPD_MAX_SAMPLES] = tempo;
  float stime = tim[VSPD_MAX_SAMPLES - VSPD_SAMPLES];
  for (int cc = (VSPD_MAX_SAMPLES - VSPD_SAMPLES); cc < VSPD_MAX_SAMPLES; cc++) {
    N1 += (tim[cc] - stime) * alt[cc];
    N2 += (tim[cc] - stime);
    N3 += (alt[cc]);
    D1 += (tim[cc] - stime) * (tim[cc] - stime);
    D2 += (tim[cc] - stime);
  }
  verticalSpeed = 1000 * ((VSPD_SAMPLES * N1) - N2 * N3) / (VSPD_SAMPLES * D1 - D2 * D2);


#ifdef BATT_PER_CELL
  fcsFrSky.setData(0, cellVoltage);
#else
  fcsFrSky.setData(0, batteryVoltage);
#endif
  gpsFrSky.setData(gpsLatitude, gpsLongitude, altitudeGPS, gpsSpeed, gpsCourse, 0, 0, 0, 0, 0, 0);
  varioFrSky.setData(baroAltitude, verticalSpeed);
  rpmFrSky.setData(hdopValue, baroTemp, batteryPercent);

}


// Send data to RX SmartPort
void SmartPort_ISR(void) {
  telemetry.send();
}
