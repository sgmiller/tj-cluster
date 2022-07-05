/*
 * CCDLibrary (https://github.com/laszlodaniel/CCDLibrary)
 * Copyright (C) 2020-2021, Daniel Laszlo
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
 */

#include <CCDLibrary.h>
#include <util/atomic.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/io.h> 

CCDLibrary *CCD1 = new CCDLibrary(&CCD1SERIAL);
CCDLibrary *CCD2 = new CCDLibrary(&CCD2SERIAL);

CCDLibrary::CCDLibrary(HardwareSerial *port)
{
    ccdPort = port;
}

CCDLibrary::~CCDLibrary()
{
    // Empty class destructor.
}

static void isrCCDActiveByte()
{
    CCD1->activeByteInterruptHandler();
}

static void globalTransmitDelayHandler() {
    CCD1->transmitDelayHandler();
}

static void globalBusIdleChange() {
    CCD1->busIdleChange();
}

void CCDLibrary::begin(float baudrate, bool dedicatedTransceiver, uint8_t busIdleBits, bool verifyRxChecksum, bool calculateTxChecksum)
{
    // baudrate:
    //   CCD_DEFAULT_SPEED: one and only speed for CCD-bus is 7812.5 baud
    // dedicatedTransceiver:
    //   CDP68HC68S1: enables 1 MHz clock signal on D11/PB5 pin for the CDP68HC68S1 CCD-bus transceiver IC. 
    //                Bus-idle and arbitration detection is handled by this IC and signaled as external interrupts:
    //                  IDLE_PIN - Arduino pin connected to CDP68HC68S1's IDLE-pin.
    //                  CTRL_PIN - Arduino pin connected to CDP68HC68S1's CTRL-pin.
    //   CUSTOM_TRANSCEIVER: disables 1 MHz clock signal on D11/PB5 pin. The library handles bus-idle and arbitration detection based on timing and bit-manipulation.
    // busIdleBits:
    //   IDLE_BITS_XX: sets the number of consecutive 1-bits sensed as CCD-bus idle condition.
    //   IDLE_BITS_10: default idle bits is 10 according to the CDP68HC68S1 datasheet. It should be changed if messages are not coming through properly.
    // verifyRxChecksum:
    //   ENABLE_RX_CHECKSUM: verifies received messages against their last checksum byte and ignores them if broken.
    //   DISABLE_RX_CHECKSUM: accepts received messages without verification.
    // calculateTxChecksum:
    //   ENABLE_TX_CHECKSUM: calculates the checksum of outgoing messages and overwrites the last message byte with it.
    //   DISABLE_TX_CHECKSUM: sends messages as they are, no checksum calculation is perfomed.

    _baudrate = baudrate;
    _dedicatedTransceiver = dedicatedTransceiver;
    _busIdleBits = busIdleBits;
    _verifyRxChecksum = verifyRxChecksum;
    _calculateTxChecksum = true;
    _messageLength = 0;
    _busIdle = true;
    _transmitAllowed = true;
    transmitting = false;

    serialInit();
    //transmitDelayTimerInit();
    listenAll();

    if (_dedicatedTransceiver)
    {
        // CDP68HC68S1 transceiver chip.
        // Setup external interrupt for bus-idle detection on the IDLE pin.
        // Detach active byte interrupt.
        pinMode(IDLE_PIN, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(IDLE_PIN), globalBusIdleChange, CHANGE);
        //detachInterrupt(digitalPinToInterrupt(CTRL_PIN));

        // Enable 1 MHz clock generator on Timer 1 and disable bus-idle timer at the same time.
        clockGeneratorInit();
    }
    else
    {
        // Custom transceiver circuits.
        // Setup external interrupt for active byte detection on the "CTRL" pin. RX1 pin is spliced and routed here.
        // Detach bus-idle interrupt. The active byte interrupt takes over the role of detecting bus-idle condition.
        pinMode(CTRL_PIN, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(CTRL_PIN), isrCCDActiveByte, FALLING);
        //detachInterrupt(digitalPinToInterrupt(IDLE_PIN));

        // Enable bus-idle timer on Timer 1 and disable 1 MHz clock generator at the same time.
        //busIdleTimerInit();
    }
}

void CCDLibrary::serialInit()
{
   ccdPort->begin(CLOCK_SPEED/128);
}

void serialEvent() {
    Serial.println("Serial event");
    if (!CCD1->transmitting && CCD1SERIAL.available()) {
        CCD1->receiveByte();
    }
    if (!CCD1->transmitting && CCD2SERIAL.available()) {
        CCD2->receiveByte();
    }
    // Swallow debug input
    while (Serial.available()) {
        Serial.read();
    }
}


void CCDLibrary::receiveByte()
{
    Serial.println("receiveByte");
    // Get error bits from status register.
    uint8_t lastRxError = ccdPort->getReadError();

    // Save byte in serial receive buffer.
    if (_serialRxBufferPos < 16)
    {
        _serialRxBuffer[_serialRxBufferPos] = ccdPort->read();
        _serialRxBufferPos++;
    }
    else
    {
        lastRxError |= UART_BUFFER_OVERFLOW; // error: buffer overflow
    }

    // Save last serial error.
    _lastSerialError = lastRxError;

    // Re-enable active byte interrupt.
 //   if (!_dedicatedTransceiver) attachInterrupt(digitalPinToInterrupt(CTRL_PIN), isrCCDActiveByte, FALLING);
}

void CCDLibrary::transmitDelayTimerStart()
{
    transmitDelayTimer.begin(globalTransmitDelayHandler, 256.0);
}

void CCDLibrary::transmitDelayHandler()
{
    Serial.println("Transmit now allowed");
    transmitDelayTimer.end();
    _transmitAllowed = true; // set flag
}

void CCDLibrary::clockGeneratorInit()
{
  analogWriteResolution(15);
  analogWrite(CLOCK_PIN, 16383);
  analogWriteFrequency(CLOCK_PIN, CLOCK_SPEED);
}

void CCDLibrary::activeByteInterruptHandler()
{
    //busIdleTimer.begin(); // start bus-idle timer right after UART RX1 pin goes low (start bit)
    detachInterrupt(digitalPinToInterrupt(CTRL_PIN)); // disable interrupt until next byte's start bit
    _busIdle = false; // clear flag
    _transmitAllowed = false; // clear flag, interrupt controlled message transmission is not affected by this flag
}

void CCDLibrary::timer1Handler()
{
    busIdleTimerStop(); // stop bus idle timer
    _busIdle = true; // set flag
    //transmitDelayTimerStart(); // start counting 256 microseconds for the next message transmission opportunity
    attachInterrupt(digitalPinToInterrupt(CTRL_PIN), isrCCDActiveByte, FALLING);
    processMessage(); // process received message, if any
}

void CCDLibrary::busIdleChange() {
    Serial.println("bus idle change");
    if (digitalRead(IDLE_PIN) == 1) {
        _busIdle = false; // clear flag
        _transmitAllowed = false; // clear flag, interrupt controlled message transmission is not affected by this flag
    } else {
        // IDLE pin transitioned from high to low = CCD-bus idle
        transmitDelayTimerStart(); // start counting 256 microseconds for the next message transmission opportunity
        _busIdle = true; // set flag
    }
}


uint8_t CCDLibrary::write(uint8_t* buffer, uint8_t bufferLength)
{
    // Return values:
    //   0: ok
    //   1: zero buffer length
    //   2: timeout
    //   3: data collision
    if (bufferLength == 0) return 1;

    if (_calculateTxChecksum && (bufferLength > 1)) // calculate message checksum if needed, minimum message length is 2 bytes
    {
        uint8_t checksum = 0;
        uint8_t checksumLocation = bufferLength - 1;
        for (uint8_t i = 0; i < checksumLocation ; i++) checksum += buffer[i];
        buffer[checksumLocation] = checksum; // overwrite checksum in the source array too
    }
    
    bool timeout = false;
    uint32_t timeoutStart = millis();

    transmitting = true;
    // CDP68HC68S1 handles arbitration, just wait for idle
    while (!_busIdle && !timeout)
    {
        if ((uint32_t)(millis() - timeoutStart) >= 1000) timeout = true;
    }

    if (timeout) return 2;

    Serial.print("CCD <- ");
    for (int i=0; i<bufferLength; i++) {
        Serial.print(buffer[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
    ccdPort->write(buffer, bufferLength);
    return 0;
}

bool CCDLibrary::shouldIgnore(uint8_t idByte) {
    uint8_t *pByte, bit;

    pByte = getBit(idByte, &bit); // check if ID-byte is on ignore list
    return pByte && (*pByte & (1 << bit)) != 0; // ID-byte is not on ignore list
}

void CCDLibrary::processMessage()
{
    if (_serialRxBufferPos > 0)
    {
        if (!shouldIgnore(_serialRxBuffer[0]))
        {
            if (_verifyRxChecksum && (_serialRxBufferPos > 1)) // verify checksum
            {
                uint8_t checksum = 0;
                uint8_t checksumLocation = _serialRxBufferPos - 1;
                for (uint8_t i = 0; i < checksumLocation ; i++) checksum += _serialRxBuffer[i];

                if (checksum == _serialRxBuffer[checksumLocation])
                {
                    for (uint8_t i = 0; i < _serialRxBufferPos; i++) _message[i] = _serialRxBuffer[i];

                    _messageLength = _serialRxBufferPos;
                    handleMessagesInternal(_message, _messageLength);
                }
                else
                {
                    handleErrorsInternal(CCD_Read, CCD_ERR_CHECKSUM);
                }
            }
            else // ignore checksum
            {
                for (uint8_t i = 0; i < _serialRxBufferPos; i++) _message[i] = _serialRxBuffer[i];

                _messageLength = _serialRxBufferPos;
                handleMessagesInternal(_message, _messageLength);
            }
        }

        _serialRxBufferPos = 0;
    }
}

void CCDLibrary::listenAll()
{
    memset(_ignoreList, 0, sizeof(_ignoreList));
}

void CCDLibrary::listen(uint8_t* ids)
{
    uint8_t *pByte, bit;

    while (*ids)
    {
        pByte = getBit(*ids, &bit);

        if (pByte)
        {
            *pByte &= ~(1 << bit);
        }

        ids++;
    }
}

void CCDLibrary::ignoreAll()
{
    memset(_ignoreList, 0xFF, sizeof(_ignoreList));
}

void CCDLibrary::ignore(uint8_t* ids)
{
    uint8_t *pByte, bit;

    while (*ids)
    {
        pByte = getBit(*ids, &bit);

        if (pByte)
        {
            *pByte |= 1 << bit;
        }

        ids++;
    }
}

uint8_t* CCDLibrary::getBit(uint8_t _id, uint8_t* _pBit)
{
    if (!_id)
    {
        *_pBit = 0xFF;
        return NULL;
    }

    *_pBit = _id % 8;
    return &(_ignoreList[_id / 8]);
}

void CCDLibrary::handleMessagesInternal(uint8_t* _message, uint8_t _messageLength)
{
    onCCDMessageReceivedHandler msgHandler = _msgHandler;

    if (msgHandler)
    {
        msgHandler(_message, _messageLength); // raise event
    }
}

void CCDLibrary::onMessageReceived(onCCDMessageReceivedHandler msgHandler)
{
    _msgHandler = msgHandler;
}

void CCDLibrary::handleErrorsInternal(CCD_Operations _op, CCD_Errors _err)
{
    if (_err != CCD_OK)
    {
        onCCDErrorHandler errHandler = _errHandler;

        if (errHandler)
        {
            errHandler(_op, _err); // raise event
        }
    }
}

void CCDLibrary::onError(onCCDErrorHandler errHandler)
{
    _errHandler = errHandler;
}