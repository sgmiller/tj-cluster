#ifndef INSTRUMENTS_H
#define INSTRUMENTS_H
#include <Arduino.h>
#include <CCDLibrary.h>

static uint8_t engineSpeed[4] = { 0xE4, 0x02, 0x84 };
static uint8_t vehicleSpeed[4] = { 0x24, 0x02, 0x00, 0x00 };
static uint8_t airbagOk[3] = { 0x50, 0x00, 0x00 };
static uint8_t messageFuel[3] = { 0x25, 0x00, 0x00 };
static uint8_t messageBatteryCharge[5] = { 0xd4, 0x1f, 0x1f, 0x00, 0x00 };
static uint8_t messageBattOil[5] = { 0x0c, 0x1f, 0x1f, 0x1f, 0x00 };

class Instrument
{
    public:
        Instrument(uint8_t *baseMessage, int messageLen);
        bool MaybeWrite(CCDLibrary ccd);
        void SetPercentage(int bpos, float pct, int min, int max);
        void SetByte(int bpos, uint8_t val);
        uint8_t GetByte(int bpos);
        

    protected:
        uint8_t _message[5];
        int _messageLen;
        bool _needsUpdate = true;
};

class BatteryAndOil : public Instrument {
    public:
        float batteryVoltage;
        int oilPressure;
        int oilTemperature;

        BatteryAndOil() : Instrument(messageBattOil, 5) {}
        void SetBatteryVoltage(float volts);
        void SetOilPressure(int psi);
        void SetOilTemperature(int degreesF);

};

 
#endif