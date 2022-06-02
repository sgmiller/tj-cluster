#ifndef INSTRUMENTS_H
#define INSTRUMENTS_H
#include <Arduino.h>
#include <CCDLibrary.h>

class Instrument
{

    public:
        Instrument(uint8_t *baseMessage, int messageLen);
        uint8_t GetMessage();
        bool MaybeWrite(CCDLibrary ccd);
        void SetPercentage(int pct, int min, int max);
        

    protected:
        uint8_t _message[4];
        uint8_t _messageLen;
        bool _needsUpdate;
};

static uint8_t engineSpeed[4] = { 0xE4, 0x02, 0x84 };
//static uint8_t vehicleSpeed[4] = { 0x24, 0x02, 0x00 };
static uint8_t airbagOk[3] = { 0x50, 0x00, 0x00 };
static uint8_t messageFuel[3] = { 0x25, 0x00, 0x00 };

static Instrument fuel(messageFuel, 3);
#endif