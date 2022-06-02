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


uint8_t speed;
uint32_t heartbeat = 0;
uint32_t currentMillis = 0; // ms
uint32_t lastMillis = 0; // ms
uint16_t writeInterval = 200; // ms
uint16_t heartbeatInterval = 500; // ms
uint8_t msgCount = 4;
uint8_t counter = 0;
IntervalTimer writeTimer;
IntervalTimer heartbeatTimer;
bool ccdReceived, ccdTransmitted, canReceived, canTransmitted;

Instrument* instruments[1]={&fuel};
int instrumentCount = 1;

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

void setup()
{
    Serial.begin(9600);
    pinMode(CCD_TX_LED_PIN, OUTPUT);
    pinMode(CCD_RX_LED_PIN, OUTPUT);
    pinMode(CAN_TX_LED_PIN, OUTPUT);
    pinMode(CAN_RX_LED_PIN, OUTPUT);
   
    CCD.onMessageReceived(CCDMessageReceived); // subscribe to the message received event and call this function when a CCD-bus message is received
    CCD.onError(CCDHandleError); // subscribe to the error event and call this function when an error occurs
    CCD.begin(); // CDP68HC68S1
    writeTimer.begin(clusterWrite, writeInterval*1000);
    heartbeatTimer.begin(handleHeartbeat, heartbeatInterval*1000);
}

void loop()
{
    speed = (speed + 5) % 100;
    delay(2000);
}

void handleHeartbeat() {
    heartbeat++;
    //Heartbeat LED
    
    digitalWrite(LED_BUILTIN, heartbeat%2);

    // DAS BLINKENLIGHTS!
    digitalWrite(CCD_TX_LED_PIN, ccdTransmitted);
    digitalWrite(CCD_RX_LED_PIN, ccdReceived);
    Serial.println("Heartbeat\n");
    ccdTransmitted=false;
    ccdReceived=false;
    Serial.print("Transmit allowed: ");
    Serial.println(digitalRead(IDLE_PIN)==0);
    if ((heartbeat%20)==0) {
        fuel.SetPercentage(heartbeat%10 * 10,1,254);
    }
}

void clusterWrite() {  
    CCD.write(airbagOk, sizeof(airbagOk));
    delay(50);
    for  (uint8_t i=0; i<instrumentCount; i++) {
        if (instruments[i]->MaybeWrite(CCD)) {
            delay(50);
            ccdTransmitted=true;
        }
    }
}
