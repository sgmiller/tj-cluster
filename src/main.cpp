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
#include <FlexCAN_T4.h>
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

// CAN bus
#define CAN1_TX 1
#define CAN1_RX 0
#define CAN2_TX 22
#define CAN2_RX 23 
#define NUM_CAN_TX_MAILBOXES 1
#define NUM_CAN_RX_MAILBOXES 2

#define REFRESH_INTERVAL 2000 // ms
#define USING_SPEED_SENSOR false
#define SELF_TEST_MODE true
#define SPEEDO_SENSOR_IN 7
#define SPEEDOMETER_RATIO 1.59
#define DISABLE_AIRBAG_LAMP true
#define INSTRUMENT_COUNT 11
#define SPEED_SENSOR_SAMPLES 4
#define ACTIVITY_ON_MS 25
#define SELF_TEST_STAGE_COUNT 10
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

// All instruments
BatteryAndOil battOil;
SingleLamp skimLamp(messageSkim, 3, 10000);
SingleLamp checkGaugesLamp(messageCheckGauges, 4, 500);
SingleLamp checkEngineLamp(messageCheckEngine, 4, 500);
Fuel fuel;
Speedometer speedo;
Tachometer tach;
FeatureStatus featureStatus;
Odometer odometer;
Instrument airbagOk(messageAirbagOk, 3, 0, 255, -1);
Instrument airbagBad(messageAirbagBad, 3, 0, 255, -1);
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

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;

void resetGauges() {
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

    if (selfTestStage == 3) {
        featureStatus.SetCruiseEnabled(true);
    } else if (selfTestStage == 4) {
        featureStatus.SetUpShift(true);
    } else if (selfTestStage == 5) {
        checkEngineLamp.SetLamp(true);
    } else if (selfTestStage == 6) {
        checkGaugesLamp.SetLamp(true);
    } else if (selfTestStage == 7) {
        battOil.SetBatteryVoltage(16);
    } else if (selfTestStage == 8) {
        float nv = (fuel.percent + 0.1);
        if (nv > 1.0) {
            nv = 0;
        }
        fuel.SetFuelPercentage(nv);
    } else if (selfTestStage == 9) {
        battOil.SetOilPressure(40);
    } else if (selfTestStage == 10) {
        battOil.SetOilTemperature(210);
    } else if (selfTestStage > 10){
        selfTestStage = 0;
        
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

void canSniff(const CAN_message_t &msg) {
  Serial.print("MB "); Serial.print(msg.mb);
  Serial.print("  OVERRUN: "); Serial.print(msg.flags.overrun);
  Serial.print("  LEN: "); Serial.print(msg.len);
  Serial.print(" EXT: "); Serial.print(msg.flags.extended);
  Serial.print(" TS: "); Serial.print(msg.timestamp);
  Serial.print(" ID: "); Serial.print(msg.id, HEX);
  Serial.print(" Buffer: ");
  for ( uint8_t i = 0; i < msg.len; i++ ) {
    Serial.print(msg.buf[i], HEX); Serial.print(" ");
  } Serial.println();
}

void onVCUVehicleInputs3(const CAN_message_t &msg) {
    uint8_t mph = msg.buf[1];
    speedo.SetMPH(mph);

    // Tell power steering to change
    CAN_message_t leafEPS;
    leafEPS.id=0x12345;
    leafEPS.buf[0]=128;
    can2.write(MB1, leafEPS);
}

void onVCUFaultStates(const CAN_message_t &msg) {
    // Decode fault states, probably don't want to alert on all of them
    for (int i=0; i<8; i++) {
        if (msg.buf[i] != 0) {
            // Oh no!  Set and forget, won't clear without a restart
            checkEngineLamp.SetLamp(true);
        }
    }
}

void setupCAN() {
  pinMode(6, OUTPUT); digitalWrite(6, LOW);
  can1.begin();
  can2.begin();

  can1.setBaudRate(500000);
  can2.setBaudRate(500000);
  can1.mailboxStatus();
  can2.mailboxStatus();

  can1.setMaxMB(NUM_CAN_TX_MAILBOXES + NUM_CAN_RX_MAILBOXES);
  for (int i = 0; i<NUM_CAN_RX_MAILBOXES; i++){
    can1.setMB((FLEXCAN_MAILBOX)i,RX,EXT);
  }
  for (int i = NUM_CAN_RX_MAILBOXES; i<(NUM_CAN_TX_MAILBOXES + NUM_CAN_RX_MAILBOXES); i++){
    can1.setMB((FLEXCAN_MAILBOX)i,TX,EXT);
  }
  can1.setMBFilter(REJECT_ALL);
  can1.enableMBInterrupts();
  can1.onReceive(MB0,onVCUVehicleInputs3);
  can1.setMBFilter(MB0,0x2F0A014);
  can1.onReceive(MB1, onVCUFaultStates);
  can1.setMBFilter(MB1, 0x2F0A044);
}

void setup() {
    Serial.begin(115200);
    // Watchdog
    WDT_timings_t config;
    config.trigger = 5; /* in seconds, 0->128 */
    config.timeout = 10; /* in seconds, 0->128 */
    config.callback = watchdogReset;
    
    //Give the cluster time to boot
    delay(3000);
    Serial.println("Entered setup");
    wdt.begin(config);

    Serial.println("A");

    // Don't update certain instruments on start
    battOil.SetOilPressure(12);
    battOil.Quiesce();
    Serial.println("B");
    
    // Did we reset due to watchdog?  Alert on this
    // Save copy of Reset Status Register
    int lastResetCause = SRC_SRSR;
    // Clear all Reset Status Register bits
    SRC_SRSR = (uint32_t)0x1FF;
    if (lastResetCause == SRC_SRSR_WDOG_RST_B) {
        // Yes.  Alert via the SKIM light
        skimLamp.SetLamp(true);    
    }
    Serial.println("C");

    // Set pins
    pinMode(VBAT_MEASURE_CTL, OUTPUT);
    pinMode(VBAT_MEASURE_SIG, INPUT);
    // Nothing we're reading moves quickly, so do read averaging
    analogReadAveraging(8);
    Serial.println("D");

    CCD.onMessageReceived(CCDMessageReceived); // subscribe to the message received event and call this function when a CCD-bus message is received
    CCD.onError(CCDHandleError); // subscribe to the error event and call this function when an error occurs
    CCD.begin(); // CDP68HC68S1
    Serial.println("E");

    if (USING_SPEED_SENSOR) {
        speedoOn=speedoMeasure.begin(SPEEDO_SENSOR_IN,FREQMEASUREMULTI_RAISING);
        pinMode(SPEEDO_SENSOR_IN, INPUT);
    }
    lastActivity=millis();
    if (SELF_TEST_MODE) {
        selfTestTimer.begin(selfTest, (1000+SELF_TEST_STAGE_DURATION)*1000);
    }
    Serial.println("F");
    
    pinMode(LED_BUILTIN, OUTPUT);
    // Start the CCD writer
    _writer.Setup(&_writer);
    //setupCAN();
    Serial.println("G");
    Serial.println("Setup complete");
}

CAN_message_t msg;
void loop()
{
  loopCount++;  
  /*//can1.events();
  static uint32_t timeout = millis();
  if ( millis() - timeout > 200 ) {
    CAN_message_t msg;
    msg.id = random(0x1,0x7FE);
    for ( uint8_t i = 0; i < 8; i++ ) msg.buf[i] = i + 1;
    can2.write(msg);
    timeout = millis();
    Serial.print(".");
    Serial.flush();
  }

  if (can1.read(msg)) {
    canSniff(msg);
  }*/
    //handleSpeedSensor();

    if (SELF_TEST_MODE) {
        float t = constrain(float(millis() - selfTestPhaseStart) / SELF_TEST_STAGE_DURATION, 0.0, 1.0);
        if (selfTestStage == 1) {
            speedo.SetKPH(160 * t);
        } else if (selfTestStage == 2) {
            tach.SetRPM(6000 * t);
        }
    }

    if (lastRefresh > REFRESH_INTERVAL) {
        lastRefresh = 0;
        clusterRefresh();
    }
    if (lastCCDLoop > INTERWRITE_DELAY) {
        lastCCDLoop = 0;
        bool newActivity = _writer.Loop();
        activity = activity || newActivity;
    }

    // DAS BLINKENLIGHTS! .. but seriously, blink the builtin LED on activity
    if (activity) {
        // Feed on activity, since there should be some pretty regularly
        wdt.feed();
        digitalWrite(LED_BUILTIN, HIGH);
        activity=false;
    } else if (lastActivity >= ACTIVITY_ON_MS) {
        lastActivity=0;
        activity=false;
        digitalWrite(LED_BUILTIN, LOW);
    }

    wdt.feed();
}

