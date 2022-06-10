#ifndef INSTRUMENTS_H
#define INSTRUMENTS_H
#include <Arduino.h>
#include <CCDLibrary.h>

#define REVS_PER_MILE 2737
#define PULSES_PER_REV 8
#define PULSES_PER_UPDATE 800
#define POST_WRITE_DELAY 50*1000 // 50ms
#define INTERWRITE_DELAY 200*1000 // 200ms

static uint8_t messageEngineSpeed[4] = { 0xE4, 0x02, 0x84 };
static uint8_t messageAirbagOk[3] = { 0x50, 0x00, 0x00 };
static uint8_t messageAirbagBad[3] = { 0x51, 0x00, 0x00 };
static uint8_t messageCheckGauges[4] = { 0xec, 0x00, 0x00, 0x00 };
static uint8_t messageCheckEngine[4] = { 0xf5, 0x00, 0x00, 0x00 };
static uint8_t messageSkim[3] = { 0x0b, 0x00, 0x00 };
static uint8_t messageFuel[3] = { 0x25, 0x00, 0x00 };
static uint8_t messageBattOil[5] = { 0x0c, 0x1f, 0x1f, 0x1f, 0x00 };
static uint8_t messageFeatureStatus[4] = { 0xa4, 0x00, 0x00, 0x00 };
static uint8_t messageVehicleSpeed[4] = { 0x24, 0x02, 0x00, 0x00 };
static uint8_t messageIncrementOdometer[4] = { 0x84, 0x00, 0x00, 0x00 };

class InstrumentWriter;

class Instrument
{
    public:
        Instrument(uint8_t *baseMessage, int messageLen);
        bool MaybeWrite(CCDLibrary ccd);
        void SetPercentage(int bpos, float pct, int min, int max);
        void SetByte(int bpos, uint8_t val);
        uint8_t GetByte(int bpos);
        // Forces a write even if values haven't changed
        void Refresh();
        // Unset the need to update
        void Quiesce();
        void setWriter(InstrumentWriter* writer);

    protected:
        InstrumentWriter* _writer;
        uint8_t _message[5];
        int _messageLen;
        bool _needsUpdate = true;
};

class InstrumentWriter {
    public:
        InstrumentWriter(Instrument** instruments, int instrumentCount);
        void Loop();  
        void Wake();  
        void Begin(InstrumentWriter* writer, void (*funct)(), void (*activity)());
    private:
        IntervalTimer _writeDelay;
        int _instrumentCount;
        Instrument** _instruments;
        int _currentInstrument;
        bool _writing;
        void (*_writeCallback)();
        void (*_activity)();
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

class Fuel : public Instrument {
    public:
        float fuelPercent;
        
        Fuel() : Instrument(messageFuel, 3) {}
        void SetFuelPercentage(float pct);
};

class SingleLamp : public Instrument {
    public:
        bool on;

        SingleLamp(uint8_t *baseMessage, int messageLen) : Instrument(baseMessage, messageLen) {}
        void SetLamp(bool on);
};

class FeatureStatus : public Instrument {
    public:
        bool upshift;
        bool cruiseEnabled;
        FeatureStatus() : Instrument(messageFeatureStatus, 4) {}

        void SetUpShift(bool on);
        void SetCruiseEnabled(bool enabled);
};

class Speedometer : public Instrument {
    public:
        int mph;
        Speedometer() : Instrument(messageVehicleSpeed, 4) {}
        void SetSpeedSensorFrequency(int pulseHz);
};
 
#endif