#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>

#define VBAT_MEASURE_SIG 33

void clusterWrite();
void handleHeartbeat();
void handleSpeedSensor();
void watchdogFeed();
void setupSpeedo();

#endif