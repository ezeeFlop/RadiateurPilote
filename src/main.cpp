#include <Arduino.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <ld2410.h>

#include "HomeSpan.h"  // Always start by including the HomeSpan library

#define DEBUG 0

#define OPTO_1 22  // Pilotage des optocoupleurs GPIO12 et 13
#define OPTO_2 21
#define ONE_WIRE_BUS 18

#define TEMP_THRESHOLD 0.1

// alpha for exponential averaging of
// temperature readings, more is smoother
#define TEMP_EXP_ALPHA 0.99

// period to scan for state changes in milliseconds
#define STATUS_PERIOD 5000

// period to scan for temperature in milliseconds
#define SENSE_PERIOD 50

double current_temp = 0;
boolean presence = false;

// enums to define some constants
enum TargetHeaterStates {
  OFF = 0,
  HORS_GEL = 1,
  ECO = 2,  // we disable this state since we don't have an AC
  CONFORT = 3
};

enum CurrentHeaterStates {
  IDLE = 0,
  HEATING = 1,
  COOLING = 2  // we disable this state since we don't have an AC (in essence this is just a bool)
};

enum TemperatureDisplayUnits {
  CELSIUS = 0,
  FAHRENHEIT = 1  // we disable this state since we are not heathens
};

class Heater {
  int opto1;
  int opto2;
  TargetHeaterStates state;

  public:
  Heater(int opto1, int opto2) : opto1(opto1), opto2(opto2) {
    pinMode(opto1, OUTPUT);
    pinMode(opto2, OUTPUT);
  }

  void setState(TargetHeaterStates targetState) {
    WEBLOG("Heater::setState(%d)\n", targetState);

    if (targetState == OFF)  // Mode arret: demi alternance positive
    {
      digitalWrite(opto1, HIGH);
      digitalWrite(opto2, LOW);
    }
    if (targetState == HORS_GEL)  // Mode hors gel: demi alternance négative
    {
      digitalWrite(opto1, LOW);
      digitalWrite(opto2, HIGH);
    }
    if (targetState == ECO)  // Mode éco: pleine alternance
    {
      digitalWrite(opto1, LOW);
      digitalWrite(opto2, LOW);
    }
    if (targetState == CONFORT)  // Mode confort: pas de signal
    {
      digitalWrite(opto1, HIGH);
      digitalWrite(opto2, HIGH);
    }
    state = targetState;
  }
  TargetHeaterStates getState() {
    return state;
  }
};

Heater heater(OPTO_1, OPTO_2);

// class ties the heater relay operation to a characteristic
class CurrentHeaterStatus : public Characteristic::CurrentHeatingCoolingState {
  private:

  public:
  // constructor for the class
  CurrentHeaterStatus() : Characteristic::CurrentHeatingCoolingState(IDLE) {}

  void setVal(uint8_t value, bool notify = true) {
    heater.setState((TargetHeaterStates) value);
    Characteristic::CurrentHeatingCoolingState::setVal(value, notify);
  }
};

// the class that contains the logic for running the
// thermostat service
class Thermostat : public Service::Thermostat {
  private:


  const float tempThreshold;

  // alpha for exponential averaging temperature
  const float tempExpAlpha;

  // period in milliseconds before the state is
  // updated and temperature is sensed respectively
  const unsigned long statusUpdatePeriod;
  const unsigned long temperatureSensePeriod;

  // setup the characteristic objects
  CurrentHeaterStatus *currentHeaterState;
  SpanCharacteristic *targetHeaterState;
  SpanCharacteristic *targetTemperature;
  SpanCharacteristic *currentTemperature;
  SpanCharacteristic *temperatureDisplayUnits;
  SpanCharacteristic *heatingThresholdTemperature;
  SpanCharacteristic *coolingThresholdTemperature;

  // track the state of the thermostat
  unsigned long lastUpdateTemperature;
  unsigned long lastSenseTemperature;
  unsigned long lastUpdateState;
  float currentExpAvgTemp;
  bool wasUpdated;

  public:
  // constructor for the thermostat
  Thermostat(
      float alpha,
      float threshold,
      unsigned long statePeriod,
      unsigned long sensePeriod) : tempExpAlpha(alpha),
                                   tempThreshold(threshold),
                                   statusUpdatePeriod(statePeriod),
                                   temperatureSensePeriod(sensePeriod) {
    // get an initial reading for current temperature
    currentExpAvgTemp = current_temp;

    // initialise the characteristics
    currentHeaterState = new CurrentHeaterStatus();
    currentTemperature = new Characteristic::CurrentTemperature(currentExpAvgTemp);
    targetHeaterState = new Characteristic::TargetHeatingCoolingState(OFF, true);
    targetTemperature = new Characteristic::TargetTemperature(20, true);
    temperatureDisplayUnits = new Characteristic::TemperatureDisplayUnits(CELSIUS);
    coolingThresholdTemperature = new Characteristic::CoolingThresholdTemperature(24, true);
    heatingThresholdTemperature = new Characteristic::HeatingThresholdTemperature(18, true);

    // setup the valid values for characteristics
    currentHeaterState->setValidValues(2, IDLE, HEATING);
    targetHeaterState->setValidValues(3, OFF, HORS_GEL, ECO, CONFORT);
    temperatureDisplayUnits->setValidValues(1, CELSIUS);
    temperatureDisplayUnits->removePerms(PW);

    // set the ranges and step values of temperatures
    targetTemperature->setRange(10, 38, 1);
    currentTemperature->setRange(-270, 100, 0.1);
    coolingThresholdTemperature->setRange(10, 35, 1);
    heatingThresholdTemperature->setRange(0, 25, 1);

    // setup the state of the thermostat
    lastUpdateTemperature = millis();
    lastSenseTemperature = millis();
    lastUpdateState = millis();
    wasUpdated = false;
  }

  // mark the state to be updated
  // so the next time loop runs it
  // doesn't wait for timer period
  virtual bool update() override {
    wasUpdated = targetHeaterState->updated() ||
                 targetTemperature->updated() ||
                 coolingThresholdTemperature->updated() ||
                 heatingThresholdTemperature->updated();
    return true;
  }

  // update the state of the thermostat
  virtual void loop() override {
    // sense temperature every given duration
    if ((millis() - lastSenseTemperature) > temperatureSensePeriod) {
      updateTempReading();
      lastSenseTemperature = millis();
    }

    // update current temperature every given duration
    // but only when when you have enough readings
    if ((millis() - lastUpdateTemperature) > statusUpdatePeriod) {
      updateCurrentTemp();
      lastUpdateTemperature = millis();
    }

    // update state every given duration
    if ((millis() - lastUpdateState) > statusUpdatePeriod || wasUpdated) {
      updateState();
      wasUpdated = false;
      lastUpdateState = millis();
    }
  }

  private:
  // update the state of the system given parameters
  void updateState() {
    // get the current state of the system
    uint8_t targetState = targetHeaterState->getVal();
    uint8_t heaterState = currentHeaterState->getVal();
    float currentTemp = currentExpAvgTemp,
          targetTemp = targetTemperature->getVal<float>(),
          minTemp = heatingThresholdTemperature->getVal<float>(),
          maxTemp = coolingThresholdTemperature->getVal<float>();

    if (targetState == HEATING) {
      if (heaterState == CONFORT || heaterState == ECO || heaterState == HORS_GEL) {
        if (currentTemp > targetTemp) {
          targetState = OFF;
        } else {
          if (presence) {
            targetState = CONFORT;
          } else {
            targetState = ECO;
          }
        }
      } else if (heaterState == OFF) {
        if (currentTemp < targetTemp) {
          if (presence) {
            targetState = CONFORT;
          } else {
            targetState = ECO;
          }
        } else {
          targetState = OFF;
        }
      }
    } else if (targetState == OFF || targetState == COOLING) {
      targetState = OFF;
    }

    WEBLOG("Target Heater State: %d", targetState);
    WEBLOG("Current Heater State: %d", heaterState);
    WEBLOG("Current Temperature: %f %f", currentTemp, current_temp);
    WEBLOG("Target Temperature: %f", targetTemp);
    WEBLOG("Min Temperature: %f", minTemp);
    WEBLOG("Max Temperature: %f", maxTemp);

    currentHeaterState->setVal(targetState);
  }

  // accumulate a new temperature reading
  // using exponential averaging
  void updateTempReading() {
    // read the current temperature
    double reading = current_temp;

    // validate for correct seaming reading
    // and accumulate if so
    if (-10 <= reading && reading <= 40) {
      currentExpAvgTemp *= tempExpAlpha;
      currentExpAvgTemp += (1 - tempExpAlpha) * reading;
    }
  }

  // update the current temperature from accumulated readings
  void updateCurrentTemp() {
    currentTemperature->setVal(currentExpAvgTemp);
  }

};

struct TempSensor : Service::TemperatureSensor {  // A standalone Temperature sensor

  SpanCharacteristic *temp;  // reference to the Current Temperature Characteristic
  int bus;                  // I2C address of temperature sensor
  uint32_t timer = 0;        // keep track of time since last update
  // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
  OneWire *oneWire;
  DallasTemperature *sensors;
  // Pass our oneWire reference to Dallas Temperature.
  int count = 0;

  TempSensor(int bus) : Service::TemperatureSensor() {  // constructor() method
    oneWire = new OneWire(bus);
    sensors = new DallasTemperature(oneWire);

    this->bus = bus;
    sensors->begin();
    double tempC = getTemp();

    temp = new Characteristic::CurrentTemperature(tempC);  // instantiate the Current Temperature Characteristic
    temp->setRange(-50, 100);                              // expand the range from the HAP default of 0-100 to -50 to 100 to allow for negative temperatures

  }  // end constructor

  void loop() {
    if (millis() - timer > 5000) {  // only sample every 5 seconds
      timer = millis();

      double tempC = getTemp();

      if (abs(temp->getVal<double>() - tempC) > 0.5) {  // only update temperature if change is more than 0.5C
        temp->setVal(tempC);                            // set the new temperature; this generates an Event Notification and also resets the elapsed time
        WEBLOG("ADT7410-%02X Temperature Update: %g\n", bus, tempC);
      }
    }

  }  // loop

  float getTemp(void) {
    WEBLOG("Requesting temperatures...");
    sensors->requestTemperatures();  // Send the command to get temperatures
    WEBLOG("DONE");
    float temp = sensors->getTempCByIndex(0);
    WEBLOG("Temperature for the device 1 (index 0) is: %f ", temp);
    count++;
    sensors->setUserDataByIndex(0, count);
    int x = sensors->getUserDataByIndex(0);
    if (temp == -127)
      temp = 0;

    current_temp = temp;

    return temp;
  }
};

struct MotionSensor : Service::MotionSensor {  // Motion sensor

  SpanCharacteristic *movement;  // reference to the MotionDetected Characteristic
  ld2410 radar;
  uint32_t lastReading = 0;

  MotionSensor() : Service::MotionSensor() {
    movement = new Characteristic::MotionDetected(false);  // instantiate the MotionDetected Characteristic
                                                           // radar.debug(Serial); //Uncomment to show debug information from the library on the Serial Monitor. By default this does not show sensor reads as they are very frequent.
    if (DEBUG == 0) {
      Serial1.begin(256000, SERIAL_8N1, TX, RX);              // UART for monitoring the radar
    }

    delay(500);
    WEBLOG("\nLD2410 radar sensor initialising: ");
    if (radar.begin(Serial1)) {
      WEBLOG("OK\n");
    } else {
      WEBLOG("not connected\n");
    }
  }  // end constructor

  void loop() {
    radar.read();

    if (radar.isConnected() && millis() - lastReading > 2000) {
      boolean motion = false;

      lastReading = millis();
      if (radar.presenceDetected()) {
        if (radar.stationaryTargetDetected()) {
          WEBLOG("Stationary target: %d\n", radar.stationaryTargetDistance());
          WEBLOG("cm energy: %d\n", radar.stationaryTargetEnergy());
          motion = true;
        }
        if (radar.movingTargetDetected()) {
          WEBLOG("Moving target: %d\n", radar.movingTargetDistance());
          WEBLOG("cm energy: %d\n", radar.movingTargetEnergy());
          motion = true;
        }
      } else {
        WEBLOG("No target detected\n");
      }

      if (motion != movement->getVal()) {
        presence = motion;
        movement->setVal(motion);
        if (motion == true) {
          WEBLOG("Motion was detected\n");
        }
      }
    }
  }
};

void setup(void) {
  if (DEBUG) {
      Serial.begin(115200);  // Feedback over Serial Monitor
  }
  delay(500);            // Give a while for Serial Monitor to wake up

  homeSpan.enableWebLog(100, "pool.ntp.org", "UTC", "myLog");  // creates a web log on the URL /HomeSpan-[DEVICE-ID].local:[TCP-PORT]/myLog
  homeSpan.setQRID("THER");
  homeSpan.setPairingCode("08443299");
  homeSpan.enableOTA(false, true);
  homeSpan.setHostNameSuffix("v1");
  homeSpan.setStatusPin(LED_BUILTIN);
  if (DEBUG == 0) {
     homeSpan.setLogLevel(-1);
  }

  homeSpan.begin(Category::Bridges, "Thermostat", "Thermostat");

  SPAN_ACCESSORY("Motion Sensor");
  new MotionSensor();
  
 
  SPAN_ACCESSORY("Thermostat");
  (new Thermostat(
       TEMP_EXP_ALPHA,
       TEMP_THRESHOLD,
       STATUS_PERIOD,
       SENSE_PERIOD));

  SPAN_ACCESSORY("Temperature Sensor");
  new TempSensor(ONE_WIRE_BUS);
}

void loop() {
  homeSpan.poll();
}