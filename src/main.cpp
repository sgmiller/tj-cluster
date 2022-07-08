/*
 * CCDWrite.ino (https://github.com/laszlodaniel/CCDLibrary)
 * Copyright (C) 2021, Daniel Laszlo
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * Example: two custom CCD-bus messages are repeatedly written 
 * to the CCD-bus while other messages (including these) are displayed 
 * in the Arduino serial monitor. 
 * Multiple messages can be sent one after another by adding 
 * new byte arrays and changing the "msgCount" value accordingly. 
 * The CCD.write() function automatically updates the source array with 
 * the correct checksum.
 */

#include <CCDLibrary.h>
#include <FreqMeasureMulti.h>
#include <FreqCount.h>
#include <Watchdog.h>
#include "main.h"
#include "instruments.h"

WDT_T4<WDT1> wdt;
uint32_t currentMillis = 0; // ms
uint32_t lastMillis = 0; // ms
uint16_t writeInterval = 200; //ms
uint8_t msgCount = 4;
uint8_t counter = 0;
bool activity, speedoOn;
FreqMeasureMulti speedoMeasure;

#define CCD1_IDLE PIN_A4
#define CCD2_IDLE 10
#define REFRESH_INTERVAL 2000 // ms
#define USING_SPEED_SENSOR false
#define SELF_TEST_MODE true
#define SPEEDO_SENSOR_IN 7
#define SPEEDOMETER_RATIO 1.59
#define DISABLE_AIRBAG_LAMP true
#define INSTRUMENT_COUNT 11
#define SPEED_SENSOR_SAMPLES 4
#define ACTIVITY_ON_MS 25
#define SELF_TEST_STAGE_COUNT 8
#define SELF_TEST_STAGE_DURATION 3000
#define VBAT_RELAY_TURN_ON_MAX 2 // ms
#define VBAT_VD_R1 2200.0 // VBAT voltage divider R1 value (ohms)
#define VBAT_VD_R2 13000.0 // VBAT voltage divider R2 value (ohms)
#define VBAT_MEASUREMENT_RATIO 1.0/(VBAT_VD_R1/(VBAT_VD_R1+VBAT_VD_R2)) // Set voltage divider resistor values here
//#define VBAT_VOLTAGE_DIVIDER_RATIO 10000000.0/1500000.0 // Set voltage divider resistor values here
#define MCU_VOLTAGE 3.3
#define PULSES_PER_AXLE_REVOLUTION 8
#define GEAR_RATIO 3.07
#define TIRE_DIAMETER 28.86
#define TIRE_CIRCUMFERENCE 3.14159*TIRE_DIAMETER
#define PULSES_PER_MILE (5280/TIRE_CIRCUMFERENCE)*PULSES_PER_AXLE_REVOLUTION

BatteryAndOil battOil;
SingleLamp skimLamp(messageSkim, 3);
SingleLamp checkGaugesLamp(messageCheckGauges, 4);
SingleLamp checkEngineLamp(messageCheckEngine, 4);
Instrument fuel(messageFuel, 3, 1, 254);
Speedometer speedo;
Tachometer tach;
FeatureStatus featureStatus;
Odometer odometer;
Instrument airbagOk(messageAirbagOk, 3, 0, 255);
Instrument airbagBad(messageAirbagBad, 3, 0, 255);
Instrument* instruments[INSTRUMENT_COUNT]= {&fuel, &speedo, &tach, &checkEngineLamp, &checkGaugesLamp, &skimLamp, &featureStatus, &battOil, &odometer, &airbagOk, &airbagBad};
InstrumentWriter _writer(instruments, INSTRUMENT_COUNT);

IntervalTimer selfTestTimer;
uint8_t speedoSignal;
bool selfTestMode = true;
double speedoFreqSum;
int speedoFreqCount;
int selfTestPhaseStart;
int loopCount;
int refreshCount;
int selfTestStage=0;
float speedSensorFrequency;
int speedSensorPulses;
elapsedMillis lastActivity;
elapsedMillis lastCCDLoop;
elapsedMillis lastRefresh;


void resetGauges() {
    elapsedMillis();
    speedo.SetMPH(0);
    tach.SetRPM(0);
    battOil.SetBatteryVoltage(14);
    battOil.SetOilPressure(20);
    battOil.SetOilTemperature(155);
    if (!selfTestMode) {
        fuel.SetPercentage(1, 0.0, 1, 254);
    }
    featureStatus.SetCruiseEnabled(false);
    featureStatus.SetUpShift(false);
    checkEngineLamp.SetLamp(false);
    checkGaugesLamp.SetLamp(true);
}

void selfTest() {
    selfTestPhaseStart = millis();
    resetGauges();
    selfTestStage++;
    //featureStatus.SetByte(1,selfTestStage);
    Serial.print("Self test stage ");
    Serial.println(selfTestStage);
    switch (selfTestStage) {
    case 1:
        featureStatus.SetCruiseEnabled(true);
        break;
    case 2:
        featureStatus.SetUpShift(true);
        break;
    case 3:
        checkEngineLamp.SetLamp(true);
        break;
    case 4:
        checkGaugesLamp.SetLamp(true);
        break;
    case 5:
        battOil.SetBatteryVoltage(16);
        break;
    case 6:
        fuel.SetPercentage(1, 0.5, 1, 254);
        break;
    case 7:
        battOil.SetOilPressure(40);
        break;
    case 8:
        battOil.SetOilTemperature(210);
        break;
    case SELF_TEST_STAGE_COUNT+1:
        selfTestStage = 0;
        break;
    }
}

void handleSpeedSensor() {
     if (speedoMeasure.available()>0) {
        // average several reading together
        int pulses = speedoMeasure.read();
        speedSensorPulses += pulses;
        speedoFreqSum = speedoFreqSum + pulses;
        speedoFreqCount = speedoFreqCount + 1;
        if (speedoFreqCount > SPEED_SENSOR_SAMPLES) {
            speedSensorFrequency = speedoMeasure.countToFrequency(speedoFreqSum / speedoFreqCount);
            speedoFreqSum = 0;
            speedoFreqCount = 0;
            speedo.SetSpeedSensorFrequency(speedSensorFrequency);
        }
        if (speedSensorPulses >= PULSES_PER_UPDATE) {
            // send an odometer increment
            odometer.AddMiles(speedSensorPulses / PULSES_PER_MILE);
            speedSensorPulses = 0;
        }
    }     
}

void CCDMessageReceived(uint8_t* message, uint8_t messageLength)
{
    activity=true;
    for (uint8_t i = 0; i < messageLength; i++)
    {
        if (message[i] < 16) Serial.print("0"); // print leading zero
        Serial.print(message[i], HEX); // print message byte in hexadecimal format on the serial monitor
        Serial.print(" "); // insert whitespace between bytes
    }

    Serial.println(); // add new line
}

void CCDHandleError(CCD_Operations op, CCD_Errors err)
{
    if (err == CCD_OK) return;

    String s = op == CCD_Read ? "READ " : "WRITE ";

    switch (err)
    {
        case CCD_ERR_BUS_IS_BUSY:
        {
            Serial.println(s + "CCD_ERR_BUS_IS_BUSY");
            break;
        }
        case CCD_ERR_BUS_ERROR:
        {
            Serial.println(s + "CCD_ERR_BUS_ERROR");
            break;
        }
        case CCD_ERR_ARBITRATION_LOST:
        {
            Serial.println(s + "CCD_ERR_ARBITRATION_LOST");
            break;
        }
        case CCD_ERR_CHECKSUM:
        {
            Serial.println(s + "CCD_ERR_CHECKSUM");
            break;
        }
        default: // unknown error
        {
            Serial.println(s + "ERR: " + String(err, HEX));
            break;
        }
    }
}

float measureBattery() {
    // Relay on
    digitalWrite(VBAT_MEASURE_CTL, HIGH);
    // Let relay settle
    delay(VBAT_RELAY_TURN_ON_MAX);
    int battery=analogRead(VBAT_MEASURE_SIG);
    // Relay off
    digitalWrite(VBAT_MEASURE_CTL, LOW);
    float analogVoltage = battery * (MCU_VOLTAGE/1023.0);
    float batVoltage = analogVoltage * VBAT_MEASUREMENT_RATIO;
    return batVoltage;
}

void clusterRefresh() {
    refreshCount++;
    tach.Refresh();
    if (DISABLE_AIRBAG_LAMP) {
        airbagOk.Refresh();
    }

    if (!SELF_TEST_MODE) {
        battOil.SetBatteryVoltage(measureBattery());
    }
}

void watchdogReset() {
    Serial.println("Resetting in 5s...");
}


void onIdleChange() {
        Serial.print("CCD1 IDLE: ");
        Serial.println(digitalRead(CCD1_IDLE));
    
}

void setup()
{
    // Watchdog
    WDT_timings_t config;
    config.trigger = 5; /* in seconds, 0->128 */
    config.timeout = 10; /* in seconds, 0->128 */
    config.callback = watchdogReset;
    wdt.begin(config);
    
    //Give the cluster time to boot
    delay(3000);

    pinMode(CCD1_IDLE, INPUT);
    pinMode(CCD2_IDLE, INPUT);
      //attachInterrupt(digitalPinToInterrupt(CCD1_IDLE), onIdleChange, CHANGE);  

    Serial.begin(115200);

    // Don't update certain instruments on start
    battOil.SetOilPressure(12);
    battOil.Quiesce();
    
    // Did we reset due to watchdog?  Alert on this
    // Save copy of Reset Status Register
    int lastResetCause = SRC_SRSR;
    // Clear all Reset Status Register bits
    SRC_SRSR = (uint32_t)0x1FF;
    if (lastResetCause == SRC_SRSR_WDOG_RST_B) {
        // Yes.  Alert via the SKIM light
        skimLamp.SetLamp(true);    
    }

    // Set pins
    pinMode(VBAT_MEASURE_CTL, OUTPUT);
    pinMode(VBAT_MEASURE_SIG, INPUT);
    // Nothing we're reading moves quickly, so do read averaging
    //analogReadAveraging(8);

    CCD1->onMessageReceived(CCDMessageReceived); // subscribe to the message received event and call this function when a CCD-bus message is received
    CCD1->onError(CCDHandleError); // subscribe to the error event and call this function when an error occurs
    CCD1->begin(); // CDP68HC68S1
    CCD2->onMessageReceived(CCDMessageReceived); // subscribe to the message received event and call this function when a CCD-bus message is received
    CCD2->onError(CCDHandleError); // subscribe to the error event and call this function when an error occurs
    CCD2->begin(); // CDP68HC68S1

    if (USING_SPEED_SENSOR) {
        speedoOn=speedoMeasure.begin(SPEEDO_SENSOR_IN,FREQMEASUREMULTI_RAISING);
        pinMode(SPEEDO_SENSOR_IN, INPUT);
    }
    lastActivity=millis();
    if (SELF_TEST_MODE) {
        selfTestTimer.begin(selfTest, (1000+SELF_TEST_STAGE_DURATION)*1000);
    }
    
    // Start the CCD writer
    _writer.Setup(&_writer);
    Serial.println("Setup complete");
}

uint8_t lastIdle=1;


void loop()
{
    loopCount++;     

/*    CCD1SERIAL.write(0xaf);
    CCD1SERIAL.flush();
    delay(10);*/
//    for (int i=0; i<10; i++) {
        //Serial.println(analogRead(11));
  //      delay(100);
   //     wdt.feed();
   // }
    //return;
    if (SELF_TEST_MODE) {
        Serial.println(digitalRead(11));
        float t = constrain(float(millis() - selfTestPhaseStart) / SELF_TEST_STAGE_DURATION, 0.0, 1.0);
        if (selfTestStage == 3) {
            speedo.SetKPH(160 * t);
        } else if (selfTestStage == 4) {
            tach.SetRPM(6000 * t);
        } 
    } else {
        if (USING_SPEED_SENSOR) {
            // If using a square wave speed sensor, check for pulses
            handleSpeedSensor();
        }

        if (lastRefresh > REFRESH_INTERVAL) {
            lastRefresh = 0;
            clusterRefresh();
            CCD2->write(messageAirbagOk, 4);
        }
    }

    if (lastCCDLoop > INTERWRITE_DELAY) {
        lastCCDLoop = 0;
        bool newActivity = _writer.Loop();
        activity = activity || newActivity;
    }

    // DAS BLINKENLIGHTS! .. but seriously, blink the builtin LED on activity
    if (lastActivity >= ACTIVITY_ON_MS) {
        lastActivity=0;
        activity=false;
        digitalWrite(LED_BUILTIN, LOW);
    } else if (activity) {
        lastActivity=0;
        // Feed on activity, since there should be some pretty regularly
        wdt.feed();
        digitalWrite(LED_BUILTIN, HIGH);
        activity=false;
    } 
}
