#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>

#define VBAT_MEASURE_SIG 15

void clusterWrite();
void handleHeartbeat();
void handleSpeedSensor();
void watchdogFeed();

#endif