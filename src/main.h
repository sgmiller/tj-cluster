#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>

#define VBAT_MEASURE_CTL PIN_A5
#define VBAT_MEASURE_SIG PIN_A0

void clusterWrite();
void handleHeartbeat();
void handleSpeedSensor();
void watchdogFeed();

#endif