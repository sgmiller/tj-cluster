#include "instruments.h"
#include "main.h"

Instrument::Instrument(uint8_t *baseMessage, int messageLen, uint8_t min, uint8_t max, int refreshInterval) {
    for (int i=0; i<messageLen; i++) {
        _message[i]=baseMessage[i];
    }
    _messageLen = messageLen;
    _writer = NULL;
    _min = min;
    _max = max;
    if (refreshInterval == 0) {
        _refreshInterval = DEFAULT_REFRESH_INTERVAL;
    } else {
        _refreshInterval = refreshInterval;
    }
    _sinceLastWrite=300000;
}

uint8_t Instrument::GetByte(int bpos) {
    return _message[bpos];
}

bool Instrument::NeedsUpdate() {
    return _needsUpdate;
}

bool Instrument::MaybeWrite(CCDLibrary ccd) {
    if (_needsUpdate || (_refreshInterval > -1 && _sinceLastWrite >= _refreshInterval)) {
        ccd.write(_message, _messageLen);
        // May need to delay even if something went wrong
        _needsUpdate=false;
        _sinceLastWrite=0;
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
    uint8_t newVal = constrain(val, _min, _max);
    if (_message[bpos] != newVal) {
        _message[bpos] = newVal;
        Refresh();
    }
}

void Instrument::Refresh() {
    _needsUpdate = true;
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

void InstrumentWriter::Setup(InstrumentWriter* self) {
    for (int i=0; i<_instrumentCount; i++) {
        Serial.print("Initial write of ");
        Serial.println(i);
        _instruments[i]->setWriter(self);
        if (_instruments[i]->MaybeWrite(CCD)) {
            delay(INTERWRITE_DELAY);    
            Serial.println("Delay complete");
        }
    }
    Serial.println("InstrumentWriter setup complete.");
}

bool InstrumentWriter::Loop() {
    if (_writing) {
        return false;
    }
    _writing=true;
    int startInstrument = _currentInstrument;
    do {
        if (_instruments[_currentInstrument]->NeedsUpdate()) {
            if (_instruments[_currentInstrument]->MaybeWrite(CCD)) {
                _currentInstrument = (_currentInstrument + 1) % _instrumentCount;
                _writing=false;
                // Return, so we delay until next loop
                return true;
            }
        } else { 
          _currentInstrument = (_currentInstrument + 1) % _instrumentCount;
        }
    } while (_currentInstrument != startInstrument);
    _writing=false;
    return false;
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
    // Seem to be three regimes here.  Don't count on this being
    // true for your cluster.
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
    percent= pct;
    SetByte(1,round(constrain(254 * percent, 0, 254)));
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
    if (enabled) {
        SetByte(2, GetByte(2) | CRUISE_ENABLED);
    } else {
        SetByte(2, GetByte(2) & !CRUISE_ENABLED);
    }
}

void FeatureStatus::SetUpShift(bool enabled) {
    upshift=enabled;
    if (enabled) {
        SetByte(1, GetByte(2) | UPSHIFT);
    } else {
        SetByte(1, GetByte(2) & !UPSHIFT);
    }
}

void Speedometer::SetSpeedSensorFrequency(int pulseHz) {
    int spv=pulseHz*3600/(8*REVS_PER_MILE);
    SetMPH(spv);
}

void Speedometer::SetMPH(int newMph) {
    mph=newMph;
    SetKPH(round(newMph * 1.609344));
    SetByte(2, newMph);
}

void Speedometer::SetKPH(int newKPH) {
    kph = newKPH;
    mph = round(kph * 0.62137119);
    SetByte(2, kph);
}

void Tachometer::SetRPM(int newRPM) {
    rpm = newRPM;
    SetByte(1, rpm/32);
}

void Odometer::AddFeet(int feet) {
    AddMiles(feet/5280.0);
}

void Odometer::AddMiles(float miles) {
    trip = trip + miles;
    float val = ODOMETER_INCREMENT_UNITS_PER_MILE*miles;
    if (val > 255) {
        Serial.println("Warning: Overflow in odometer addition");
    }
    SetByte(2, uint8_t(val));
    // This may be the same value, so refresh anyway
    Refresh();
}