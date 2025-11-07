#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>

void clusterWrite();
void handleHeartbeat();
void handleSpeedSensor();
void watchdogFeed();
void setupSpeedo();

#endif