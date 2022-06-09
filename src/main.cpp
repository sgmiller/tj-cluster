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
uint32_t heartbeat = 0;
uint32_t currentMillis = 0; // ms
uint32_t lastMillis = 0; // ms
uint16_t refreshInterval = 1000; // ms
uint16_t writeInterval = 200; //ms
uint16_t heartbeatInterval = 500; // ms
uint8_t msgCount = 4;
uint8_t counter = 0;
IntervalTimer refreshTimer;
IntervalTimer heartbeatTimer;
bool ccdReceived, ccdTransmitted, canReceived, canTransmitted, speedoOn;
FreqMeasureMulti speedoMeasure;

#define POT 15
#define SPEEDO_SENSOR_OUT 18
#define SPEEDO_SENSOR_IN 22
#define SPEEDOMETER_RATIO 1.59
#define DISABLE_AIRBAG_LAMP true
#define INSTRUMENT_COUNT 9
#define SPEED_SENSOR_SAMPLES 4


BatteryAndOil battOil;
SingleLamp skimLamp(messageSkim, 3);
SingleLamp checkGaugesLamp(messageCheckGauges, 4);
SingleLamp checkEngineLamp(messageCheckEngine, 4);
Instrument fuel(messageFuel, 3);
Speedometer speedo;
Instrument rpm(messageEngineSpeed, 4);
FeatureStatus featureStatus;
Instrument incrementOdometer(messageIncrementOdometer, 4);

Instrument* instruments[INSTRUMENT_COUNT]={&fuel, &speedo, &rpm, &checkEngineLamp, &checkGaugesLamp, &skimLamp, &featureStatus, &battOil, &incrementOdometer};
IntervalTimer outPWM;
IntervalTimer writeTimer;
uint8_t speedoSignal;
double speedoFreqSum;
int speedoFreqCount;
bool airbagOk = DISABLE_AIRBAG_LAMP;
int loopCount;
int breathDir = 4;
int breathPWM;
float speedSensorFrequency;
int speedSensorPulses;

void loop()
{
    loopCount++;     
    handleSpeedSensor();

    // DAS BLINKENLIGHTS! .. but seriously, blink I/O indicator pins
    digitalWrite(CCD_TX_LED_PIN, ccdTransmitted);
    digitalWrite(CCD_RX_LED_PIN, ccdReceived);
    ccdTransmitted=false;
    ccdReceived=false;

    digitalWrite(CAN_TX_LED_PIN, ccdTransmitted);
    digitalWrite(CAN_RX_LED_PIN, ccdReceived);
    canTransmitted=false;
    canReceived=false;

    // Liveness indicator.  "Breathing" being controlled in the main
    // loop provides a decent indicator that things are still working.
    if (loopCount%40000 == 0) {
        breathPWM = breathPWM + breathDir;
        if (breathPWM < 0) {
            breathDir = 4;
            breathPWM = 0;
        } else if (breathPWM > 255) {
            breathDir = -4;
            breathPWM = 255;
        }
        analogWrite(LED_BUILTIN, breathPWM);
    }
}

void watchdogFeed() {
    wdt.feed();
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
            speedSensorPulses = speedSensorPulses - PULSES_PER_UPDATE;
            incrementOdometer.Refresh();
        }
    }     
}

void handleHeartbeat() {
    heartbeat++;
    //Heartbeat LED    
}

void clusterWrite() {  
    if (DISABLE_AIRBAG_LAMP || airbagOk) {
        // Write the airbag every time
        CCD.write(messageAirbagOk, sizeof(messageAirbagOk));
    } else {
        CCD.write(messageAirbagBad, sizeof(messageAirbagBad));
    }
    delay(50);

    // Now write just those instruments that have changed
    for  (uint8_t i=0; i<INSTRUMENT_COUNT; i++) {
        if (instruments[i]->MaybeWrite(CCD)) {
            ccdTransmitted=true;
        }
    }
}

void CCDMessageReceived(uint8_t* message, uint8_t messageLength)
{
    ccdReceived=true;
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

void speedoPWM() {
    speedoSignal = (speedoSignal + 1) % 2;
    digitalWrite(SPEEDO_SENSOR_OUT, speedoSignal);
}

void clusterRefresh() {
    rpm.Refresh();
}

void watchdogReset() {
    Serial.println("Resetting in 5s...");
}

void setup()
{
    WDT_timings_t config;
    config.trigger = 5; /* in seconds, 0->128 */
    config.timeout = 10; /* in seconds, 0->128 */
    config.callback = watchdogReset;
    wdt.begin(config);

    // Don't increment the odometer on start
    incrementOdometer.Quiesce();
    
    // Did we reset due to watchdog?  Alert on this
    // Save copy of Reset Status Register
    int lastResetCause = SRC_SRSR;
    // Clear all Reset Status Register bits
    SRC_SRSR = (uint32_t)0x1FF;
    if (lastResetCause == SRC_SRSR_WDOG_RST_B) {
        // Yes.  Alert via the SKIM light
        skimLamp.SetLamp(true);    
    }

    // Register instruments
    pinMode(CCD_TX_LED_PIN, OUTPUT);
    pinMode(CCD_RX_LED_PIN, OUTPUT);
    pinMode(CAN_TX_LED_PIN, OUTPUT);
    pinMode(CAN_RX_LED_PIN, OUTPUT);
    pinMode(SPEEDO_SENSOR_OUT, OUTPUT);
    pinMode(SPEEDO_SENSOR_IN, INPUT);
    pinMode(POT, INPUT);
    CCD.onMessageReceived(CCDMessageReceived); // subscribe to the message received event and call this function when a CCD-bus message is received
    CCD.onError(CCDHandleError); // subscribe to the error event and call this function when an error occurs
    CCD.begin(); // CDP68HC68S1
    writeTimer.begin(clusterWrite, writeInterval*1000);
    refreshTimer.begin(clusterRefresh, refreshInterval*1000);
    heartbeatTimer.begin(handleHeartbeat, heartbeatInterval*1000);
    //outPWM.begin(speedoPWM, 100000);
    speedoOn=speedoMeasure.begin(SPEEDO_SENSOR_IN,FREQMEASUREMULTI_RAISING);
    Serial.println("Setup complete");
}

