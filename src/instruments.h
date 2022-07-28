#ifndef INSTRUMENTS_H
#define INSTRUMENTS_H
#include <Arduino.h>
#include <CCDLibrary.h>

#define REVS_PER_MILE 2737
#define PULSES_PER_REV 8
#define PULSES_PER_UPDATE 800
#define ODOMETER_INCREMENT_UNITS_PER_MILE 8000
#define INTERWRITE_DELAY 50 // 50ms
#define EOM_DELAY 1280 // 1.28 ms
#define UPSHIFT 144
#define CRUISE_ENABLED 132
#define DEFAULT_REFRESH_INTERVAL 2000

// In bus priority order 
static uint8_t messageCheckEngine[4] = { 0xf5, 0x00, 0x00, 0x00 };
static uint8_t messageCheckGauges[4] = { 0xec, 0x00, 0x00, 0x00 };
static uint8_t messageEngineSpeed[4] = { 0xe4, 0x02, 0x84 };
static uint8_t messageFeatureStatus[4] = { 0xa4, 0x02, 0x00, 0x00};
static uint8_t messageIncrementOdometer[4] = { 0x84, 0x00, 0x00, 0x00 };
static uint8_t messageAirbagBad[3] = { 0x51, 0x00, 0x00 };
static uint8_t messageAirbagOk[3] = { 0x50, 0x00, 0x00 };
static uint8_t messageFuel[3] = { 0x25, 0x01, 0x01 };
static uint8_t messageVehicleSpeed[4] = { 0x24, 0x02, 0x00, 0x00 };
static uint8_t messageSkim[3] = { 0x0b, 0x00, 0x00 };
static uint8_t messageBattOil[6] = { 0x0c, 0,0,0,0x1f, 0 };

class InstrumentWriter;

class Instrument
{
    public:
        Instrument(uint8_t *baseMessage, int messageLen, uint8_t min, uint8_t max, int refreshInterval);
        bool NeedsUpdate();
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
        uint8_t _min,_max;
        InstrumentWriter* _writer;
        uint8_t _message[5];
        elapsedMillis _sinceLastWrite;
        int _refreshInterval;
        int _messageLen;
        bool _needsUpdate;
};

class InstrumentWriter {
    public:
        InstrumentWriter(Instrument** instruments, int instrumentCount);
        bool Loop();  
        void Setup(InstrumentWriter* writer);
    private:
        bool _writing;
        int _instrumentCount;
        Instrument** _instruments;
        int _currentInstrument;
        void (*_writeCallback)();
        void (*_activity)();
};

class BatteryAndOil : public Instrument {
    public:
        float batteryVoltage;
        int oilPressure;
        int oilTemperature;

        BatteryAndOil() : Instrument(messageBattOil, 6, 0, 255, 5000) {}
        void SetBatteryVoltage(float volts);
        void SetOilPressure(int psi);
        void SetOilTemperature(int degreesF);

};

class Fuel : public Instrument {
    public:
        float percent;
        
        Fuel() : Instrument(messageFuel, 3,1,255,30000) {}
        void SetFuelPercentage(float pct);
};

class SingleLamp : public Instrument {
    public:
        bool on;

        SingleLamp(uint8_t *baseMessage, int messageLen, int refreshInterval) : Instrument(baseMessage, messageLen,0,255,refreshInterval) {}
        void SetLamp(bool on);
};

class FeatureStatus : public Instrument {
    public:
        bool upshift;
        bool cruiseEnabled;
        FeatureStatus() : Instrument(messageFeatureStatus, 4, 0, 255, 0) {}

        void SetUpShift(bool on);
        void SetCruiseEnabled(bool enabled);
};

class Speedometer : public Instrument {
    public:
        int mph;
        int kph;
        Speedometer() : Instrument(messageVehicleSpeed, 4, 0, 160,  0) {}
        void SetSpeedSensorFrequency(int pulseHz);
        void SetMPH(int mph);
        void SetKPH(int kph);
};

class Tachometer : public Instrument {
    public:
        int rpm;
        Tachometer() : Instrument(messageEngineSpeed, 4, 0, 255, 0) {}
        void SetRPM(int rpm);
};

class Odometer : public Instrument {
    public:
        float trip;
        Odometer() : Instrument(messageIncrementOdometer, 4, 0, 255, -1) {}
        void AddFeet(int feet);
        void AddMiles(float miles);
};

#endif