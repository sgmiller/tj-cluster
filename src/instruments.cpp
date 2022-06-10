#include "instruments.h"
#include "main.h"

Instrument::Instrument(uint8_t *baseMessage, int messageLen) {
    for (int i=0; i<messageLen; i++) {
        _message[i]=baseMessage[i];
    }
    _messageLen = messageLen;
    _writer = NULL;
}

uint8_t Instrument::GetByte(int bpos) {
    return _message[bpos];
}

bool Instrument::MaybeWrite(CCDLibrary ccd) {
    if (_needsUpdate) {
        watchdogFeed();
        ccd.write(_message, _messageLen);
        // May need to delay even if something went wrong
        _needsUpdate=false;
        return true;
    }
    return false;
}

void Instrument::SetPercentage(int bpos, float pct, int min, int max) {
    uint8_t nextByte = constrain((max-min)*pct, min, max);
    _needsUpdate = nextByte != _message[bpos];
    _message[bpos]=nextByte;
}

void Instrument::SetByte(int bpos, uint8_t val) {
    if (_message[bpos] != val) {
        _message[bpos] = val;
        Refresh();
    }
}

void Instrument::Refresh() {
    _needsUpdate = true;
    if (_writer != NULL) {
        _writer->Wake();
    }
}

void Instrument::Quiesce() {
    _needsUpdate=false;
}

void Instrument::setWriter(InstrumentWriter* writer) {
    _writer=writer;
}

InstrumentWriter::InstrumentWriter(Instrument** instruments, int instrumentCount) {
    _instruments = instruments;
    _instrumentCount = instrumentCount;
}

void InstrumentWriter::Begin(InstrumentWriter* self, void (*funct)(),void (*activity)()) {
    
    _activity = activity;
    _writeCallback = funct;
    Serial.println("Distributing writer");
    Serial.println(self == NULL);
    for (int i=0; i<_instrumentCount; i++) {
        _instruments[i]->setWriter(self);
    }
    Serial.println("Starting writeloop");
    _writeDelay.begin(funct, POST_WRITE_DELAY);
}

void InstrumentWriter::Loop() {
    if (_writing) {
        Serial.println("Skipping loop");
        return;
    }
    Serial.println("Big loop");

    int start = _currentInstrument;
    do {
        if (_instruments[_currentInstrument]->MaybeWrite(CCD)) {
            // Delay after this, but only as long as necessary
            _writing++;
            _writeDelay.begin(_writeCallback, POST_WRITE_DELAY);
            _currentInstrument = (_currentInstrument + 1) % _instrumentCount;
            _activity();
            break;
        } else {
            _currentInstrument = (_currentInstrument + 1) % _instrumentCount;
        }
    } while (_currentInstrument != start);
    _writing=false;
    _writeDelay.update(INTERWRITE_DELAY);
}

void InstrumentWriter::Wake() {
    Serial.println("Wake");
    if (!_writing) {
        _writeDelay.end();
        Loop();
    }
}

void BatteryAndOil::SetBatteryVoltage(float volts) {
    batteryVoltage = volts;
    if (volts > 20) {
        volts = 20;
    }
    SetByte(1, round(constrain(volts * 8,0,255)));
}

void BatteryAndOil::SetOilPressure(int psi) {
    oilPressure = psi;
    // Odd
    // Seem to be three regimes here
    if (psi <= 40) {
        SetByte(2, psi*2);
    } else if (psi <= 60) {
        SetByte(2, 80+(psi-40)*3);
    } else {
        SetByte(2, 140+(psi-60));
    }
}

void BatteryAndOil::SetOilTemperature(int tempF) {
    oilTemperature = tempF;
    int hex = 0xC1;
    if (tempF < 100) {           
        hex = 0x00;
    } else if (tempF >= 100 && tempF <= 210) {
        hex = ((tempF - 100) * 0.54545455) + 105;
    } else if (tempF > 210 && tempF < 223) {
        hex = ((tempF - 210) * 1.6) + 165;
    } else if (tempF >= 223 && tempF < 248) {
        hex = 0xBC; //188
    }
    int newTemp = round(constrain(hex, 0, 255));

     SetByte(3, newTemp);
}

void Fuel::SetFuelPercentage(float pct) {
    fuelPercent = pct;
    SetByte(1,round(constrain(254 * (fuelPercent / 100), 0, 254)));
}

uint8_t lampBool(bool b) {
    if (b) {
        return 255;
    }
    return 0;
}

void SingleLamp::SetLamp(bool on) {
    SetByte(1, lampBool(on));
}

void FeatureStatus::SetCruiseEnabled(bool enabled) {
    cruiseEnabled=enabled;
    SetByte(3, lampBool(enabled));
}

void FeatureStatus::SetUpShift(bool enabled) {
    upshift=enabled;
    SetByte(2, lampBool(enabled));
}

void Speedometer::SetSpeedSensorFrequency(int pulseHz) {
    int spv=pulseHz*3600/(8*REVS_PER_MILE);
    mph=spv;
    SetByte(1, mph);
}
