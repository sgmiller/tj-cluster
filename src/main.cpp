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
#include "main.h"
#include "instruments.h"
#include <FreqMeasureMulti.h>


uint32_t heartbeat = 0;
uint32_t currentMillis = 0; // ms
uint32_t lastMillis = 0; // ms
uint16_t writeInterval = 200; // ms
uint16_t heartbeatInterval = 500; // ms
uint8_t msgCount = 4;
uint8_t counter = 0;
IntervalTimer writeTimer;
IntervalTimer heartbeatTimer;
bool ccdReceived, ccdTransmitted, canReceived, canTransmitted, speedoOn;
FreqMeasureMulti speedoMeasure;

#define REVS_PER_MILE 2737
#define PULSES_PER_REV 8

#define POT 15
#define SPEEDO_SENSOR_OUT 18
#define SPEEDO_SENSOR_IN 22
#define SPEEDOMETER_RATIO 1.59


Instrument fuel(messageFuel, 3);
Instrument battery(messageBatteryCharge, 5);
Instrument speed(vehicleSpeed, 5);
Instrument rpm(engineSpeed, 4);
BatteryAndOil battOil;
Instrument* instruments[2]={&fuel, &battery};
int instrumentCount = 2;
IntervalTimer outPWM;
uint8_t speedoSignal;
double speedoSum;
int speedoCount;
int loopCount;


void loop()
{
    loopCount++;     //Serial.print("Transmit allowed: ");
    if (speedoMeasure.available()>0) {
        // average several reading together
        speedoSum = speedoSum + speedoMeasure.read();
        speedoCount = speedoCount + 1;
        if (speedoCount > 4) {
            float frequency = speedoMeasure.countToFrequency(speedoSum / speedoCount);
            /*Serial.print("Measured hz:");
            Serial.println(frequency);*/
            speedoSum = 0;
            speedoCount = 0;
            //freq= (simulatedSpeed/3600.0)*REVS_PER_MILE*3
        
            int spv=frequency*3600/(3*REVS_PER_MILE);
            /*
            Serial.print("SPV:");
            Serial.println(spv);*/
            vehicleSpeed[2]=constrain(spv*SPEEDOMETER_RATIO, 0, 200);
        }
    }
    int pot = analogRead(POT);
//        Serial.println(pot);

   /*int simulatedSpeed=(pot/1024.0)*100;
    float speedoSensorFreq = (simulatedSpeed/3600.0)*REVS_PER_MILE*3; //hz
    analogWrite(SPEEDO_SENSOR_OUT, 128);
    analogWriteFrequency(SPEEDO_SENSOR_OUT, speedoSensorFreq);
    // DAS BLINKENLIGHTS!
    digitalWrite(CCD_TX_LED_PIN, ccdTransmitted);
    digitalWrite(CCD_RX_LED_PIN, ccdReceived);*/
    int val = 127;
/*    if (CCDSERIAL.available()>0) {
        Serial.print("Out of the blue! ");
        Serial.println(CCDSERIAL.read());
    }*/

        int i = loopCount%10;
        int j = 1<<(loopCount%10);
        int volts = loopCount;
        battOil.SetBatteryVoltage((loopCount/10.0)+9);
        battOil.SetOilPressure((loopCount*5)%80);
        battOil.SetOilTemperature(100 + (loopCount*5)%240);
        Serial.print("OilPressure:"); Serial.println(battOil.oilPressure);
        Serial.print("BattVoltage:"); Serial.println(battOil.batteryVoltage);
        Serial.print("OilTemperat:"); Serial.println(battOil.oilTemperature);
        Serial.print(battOil.GetByte(1)); Serial.print(",");
        Serial.print(battOil.GetByte(2)); Serial.print(",");
        Serial.print(battOil.GetByte(3)); Serial.println();
        battOil.MaybeWrite(CCD);
        delay(1000);
}

void handleHeartbeat() {
    heartbeat++;
    //Heartbeat LED
    
    digitalWrite(LED_BUILTIN, heartbeat%2);
//    Serial.println("Heartbeat\n");
    ccdTransmitted=false;
    ccdReceived=false;
    /*if ((heartbeat%20)==0) {
        fuel.SetPercentage(heartbeat%10 * 10,1,254);
    }*/
    
}

void clusterWrite() {  
    // Write the airbag every time
    //CCD.write(airbagOk, sizeof(airbagOk));
    //delay(200);
    // Same with RPM/MAP
    //CCD.write(vehicleSpeed, sizeof(vehicleSpeed));
    //delay(50);

    // Now write just those instruments that have changed
    for  (uint8_t i=0; i<instrumentCount; i++) {
        /*if (instruments[i]->MaybeWrite(CCD)) {
            ccdTransmitted=true;
        }*/
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

void setup()
{
    // Register instruments
    Serial.begin(9600);
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
    heartbeatTimer.begin(handleHeartbeat, heartbeatInterval*1000);
    //outPWM.begin(speedoPWM, 100000);
    speedoOn=speedoMeasure.begin(SPEEDO_SENSOR_IN,FREQMEASUREMULTI_RAISING);
}

