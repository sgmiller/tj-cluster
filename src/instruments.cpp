#include "instruments.h"

Instrument::Instrument(uint8_t *baseMessage, int messageLen) {
    for (int i=0; i<messageLen; i++) {
        _message[i]=baseMessage[i];
    }
    _messageLen = messageLen;
}

bool Instrument::MaybeWrite(CCDLibrary ccd) {
    if (_needsUpdate) {
        ccd.write(_message, _messageLen);
        // May need to delay even if something went wrong
        return true;
    }
    return false;
}


void Instrument::SetPercentage(int pct, int min, int max) {
    float pctFloat = (float)pct;
    uint8_t nextByte = constrain((max-min)*(pctFloat/100), min, max);
    _needsUpdate = nextByte != _message[1];
}