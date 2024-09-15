/*

                                FRSKY AVIONICS TELEMETRY

   Open source Arduino based FrSky basic avionics telemetry system for RC aircrafts.
   Arduino uses a GB-1803 GPS module, a BMP280 digital barometer and a resistor divider
   filtered with a capacitor; to obtain navigation data, altitude data and battery
   voltage data. Using FrSkySportTelemetry library, Arduino emulates FrSky sensors and
   display all data in the transmitter LCD.



   Aaron G.
   Apr 2020 (Updated Sep 2024)
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
#include "FrSkySportPollingDynamic.h"

// ====== START USER CONFIG ======
//#define FROM_SEA_LEVEL              // Show altitude (from mean sea level, QNE) instead of height (from ground, QFE)
//#define ALTITUDE_IN_FEET            // Show altitude/height in feet instead of meters
#define BATT_PER_CELL               // Show battery voltage per cell instead of total voltage
#define MAX_CELL_VOLTS        4200  // mV
#define MIN_CELL_VOLTS        3300  // mV
#define DIVIDER_UPPER_R       6800  // Ohm
#define DIVIDER_LOWER_R       390   // Ohm
#define VOLTAGE_RATE          1.0
#define VOLTAGE_OFFSET        0.0


// ====== END USER CONFIG ======
#define LED_PIN               13    // Status LED, will turn ON after start-up when system is ready to go
#define VOLTAGE_PIN           A6    // Analog pin where voltage sensor is connected
#define EMA_ALPHA_BAT         0.10  // Amount of the new value over 1.0 that will be added in each filter loop
#define EMA_PERIOD_BAT        50    // ms filter loop time
#define MAX_ADC               1023  // 10 bit ADC
#define ADC_AREF              1100  // mV from ATMEGA328P internal 1V1 AREF
#define GPS_SERIAL            Serial
#define VSPD_SAMPLES          40
#define VSPD_MAX_SAMPLES      50
#define EMA_ALPHA_VARIO       0.25
#define EMA_PERIOD_VARIO      20
#define SMARTPORT_UPDATE      2000      // FrSky SmartPort update period (us)
#define SEA_PRESSURE          101325    // Default sea pressure for QNE operation (Pa)


Adafruit_BMP280         baroSensor;
TinyGPSPlus             gpsSensor;
FrSkySportSensorFcs     fcsFrSky;
FrSkySportSensorGps     gpsFrSky;
FrSkySportSensorVario   varioFrSky;
FrSkySportSensorRpm     rpmFrSky;
FrSkySportTelemetry     telemetry(new FrSkySportPollingDynamic());


uint8_t cellNum;
float adcToVoltsConversionRate;
float filteredADC = 0, batteryVoltage, cellVoltage, batteryPercent;
uint32_t lastBatFilterTime = 0, lastVarioFilterTime = 0;
int gpsHDOP = 99;
float gpsLatitude, gpsLongitude, gpsSpeed, gpsCourse, gpsAltitude;
float actualPressure, referencePressure, baroAltitude, instantVSpd, filteredVSpd = 0, baroTemp;
float lastBaroAltitude = 0;
float tempo = millis();
float N1 = 0, N2 = 0, N3 = 0, D1 = 0, D2 = 0;
float alt[51];
float tim[51];


void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  delay(100);
  analogReference(INTERNAL); // Set analog reference to ATMEGA328P internal 1V1
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
    delayMicroseconds(5000);
  }
  referencePressure /= 200.0;
#endif

  delay(2000);
  adcToVoltsConversionRate = ((float)ADC_AREF / (float)MAX_ADC) * (((float)DIVIDER_UPPER_R + (float)DIVIDER_LOWER_R) / (float)DIVIDER_LOWER_R);
  batteryVoltage = (analogRead(VOLTAGE_PIN) * adcToVoltsConversionRate * VOLTAGE_RATE) + VOLTAGE_OFFSET;
  if (batteryVoltage <= 17000) // Get number of cells from total voltage
    cellNum = 4;
  if (batteryVoltage <= 12800)
    cellNum = 3;
  if (batteryVoltage <= 8600)
    cellNum = 2;
  if (batteryVoltage <= 4400)
    cellNum = 1;

  Timer1.initialize(SMARTPORT_UPDATE);    // Interruption used to send data to FrSky SmartPort
  Timer1.attachInterrupt(SmartPort_ISR);
  interrupts();
  digitalWrite(LED_PIN, HIGH);            // Turn LED ON when ready to go
}


void loop() {

  // Voltage
  if (millis() >= (lastBatFilterTime + EMA_PERIOD_BAT)) {
    filteredADC = (analogRead(VOLTAGE_PIN) * EMA_ALPHA_BAT) + (filteredADC * (1.0 - EMA_ALPHA_BAT));
    batteryVoltage = (filteredADC * adcToVoltsConversionRate * VOLTAGE_RATE) + VOLTAGE_OFFSET;
    cellVoltage = batteryVoltage / (float)cellNum;
    batteryPercent = constrain(((cellVoltage - MIN_CELL_VOLTS) * 100.0 / (MAX_CELL_VOLTS - MIN_CELL_VOLTS)) , 0.0, 100.0);
#ifdef BATT_PER_CELL
    fcsFrSky.setData(0, (float)cellVoltage / 1000.0);
#else
    fcsFrSky.setData(0, (float)batteryVoltage / 1000.0);
#endif
    rpmFrSky.setData(gpsHDOP, baroTemp, batteryPercent);
    lastBatFilterTime = millis();
  }


  // GPS
  if (GPS_SERIAL.available()) {

    while (GPS_SERIAL.available()) {
      gpsSensor.encode(GPS_SERIAL.read());
    }
    /*
        if (gpsSensor.location.isValid()) {
          gpsLatitude = gpsSensor.location.lat();
          gpsLongitude = gpsSensor.location.lng();
          gpsHDOP = gpsSensor.hdop.hdop();
        } else {
          gpsLatitude = 0.000000;
          gpsLongitude = 0.000000;
          gpsHDOP = 20;
        }

        if (gpsSensor.speed.isValid()) {
          gpsSpeed = gpsSensor.speed.mps();
        } else {
          gpsSpeed = 0.0;
        }

        if (gpsSensor.altitude.isValid()) {
          gpsAltitude = gpsSensor.altitude.meters();
        } else {
          gpsAltitude = 0.0;
        }

        if (gpsSensor.course.isValid()) {
          gpsCourse = gpsSensor.course.deg();
        } else {
          gpsCourse = 0.0;
        }
    */
    gpsHDOP = gpsSensor.hdop.hdop();
    gpsFrSky.setData(gpsSensor.location.lat(),
                     gpsSensor.location.lng(),
                     gpsSensor.altitude.meters(),
                     gpsSensor.speed.mps(),
                     gpsSensor.course.deg(),
                     gpsSensor.date.year() - 2000,
                     gpsSensor.date.month(),
                     gpsSensor.date.day(),
                     gpsSensor.time.hour(),
                     gpsSensor.time.minute(),
                     gpsSensor.time.second()
                    );
  }


  // Barometer
  if (millis() >= (lastVarioFilterTime + EMA_PERIOD_VARIO)) {
    actualPressure = baroSensor.readPressure();
    baroTemp = baroSensor.readTemperature();
    baroAltitude = 44330 * (1.0 - pow(actualPressure / referencePressure, 0.190284));
    //instantVSpd = (baroAltitude - lastBaroAltitude) * 1000 / (millis() - lastVarioFilterTime);
    //filteredVSpd = (instantVSpd * EMA_ALPHA_VARIO) + (filteredVSpd * (1.0 - EMA_ALPHA_VARIO));
    //instantVSpd = filteredVSpd;
    //lastBaroAltitude = baroAltitude;


    // Vario algorithm derived from https://www.instructables.com/DIY-Arduino-Variometer-for-Paragliding/
    tempo = millis();
    N1 = 0;
    N2 = 0;
    N3 = 0;
    D1 = 0;
    D2 = 0;
    instantVSpd = 0;
    for (int j = 0; j < VSPD_MAX_SAMPLES; j++) {
      alt[j] = alt[(j + 1)];
      tim[j] = tim[(j + 1)];
    }
    alt[VSPD_MAX_SAMPLES] = baroAltitude;
    tim[VSPD_MAX_SAMPLES] = tempo;
    float stime = tim[VSPD_MAX_SAMPLES - VSPD_SAMPLES];
    for (int k = (VSPD_MAX_SAMPLES - VSPD_SAMPLES); k < VSPD_MAX_SAMPLES; k++) {
      N1 += (tim[k] - stime) * alt[k];
      N2 += (tim[k] - stime);
      N3 += (alt[k]);
      D1 += (tim[k] - stime) * (tim[k] - stime);
      D2 += (tim[k] - stime);
    }
    instantVSpd = 1000 * ((VSPD_SAMPLES * N1) - N2 * N3) / (VSPD_SAMPLES * D1 - D2 * D2);


#ifdef ALTITUDE_IN_FEET
    baroAltitude *= 3.2808;    // Convert altitude from m to ft
    instantVSpd *= 196.8504;   // Convert VSpd from m/s to ft/min
#endif

    varioFrSky.setData(baroAltitude, instantVSpd);
    lastVarioFilterTime = millis();
  }

}


// Send data to RX SmartPort
void SmartPort_ISR(void) {
  telemetry.send();
}
