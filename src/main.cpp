/*
    XJ/TJ CCD/CAN bus arbiter
    (C) 2022 N9 Works/ Scott Miller

    This code is rather specific to driving a Jeep XJ or TJ cluster dash
    with values from onboard sensors and/or the CAN bus to convert to CCD
    bus signaling in the message formats expected for those vehicles'
    instrument clusters.

    This code is MIT licensed, however the dependent CCDLibrary is GPL, which
    carries extra requirements, including publication of the source code if
    used in a commercial project.

    Copyright (c) 2022 Scott G Miller

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to
   deal in the Software without restriction, including without limitation the
   rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
   sell copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
   all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
   FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
   IN THE SOFTWARE.
 */

#include "main.h"
#include "instruments.h"
#include <CCDLibrary.h>
#include "FreqCountESP.h"
#include <Watchdog.h>
#include <elapsedMillis.h>
#include <ESP32_CAN.h>

Watchdog wdt(5);
bool activity, speedoOn;

// Serial port, swap to external pins when not debugging
#define Stdout Serial

// CAN bus
#define CAN1_TX 26
#define CAN1_RX 27
#define CAN2_TX 31
#define CAN2_RX 30
#define NUM_CAN_TX_MAILBOXES 1
#define NUM_CAN_RX_MAILBOXES 2

#define SELF_TEST_MODE false
#define USING_SPEED_SENSOR false
#define SPEEDO_SENSOR_IN 27
#define SPEED_SENSOR_WINDOW 100 //ms
#define SPEEDOMETER_RATIO 1.59
#define DISABLE_AIRBAG_LAMP true
#define INSTRUMENT_COUNT 11
#define ACTIVITY_ON_MS 25
#define SELF_TEST_STAGE_COUNT 10
#define SELF_TEST_STAGE_DURATION 3000
#define VBAT_RELAY_TURN_ON_MAX 2 // ms
#define VBAT_VD_R1 100000.0 // VBAT voltage divider R1 value (ohms)
#define VBAT_VD_R2 4700.0 // VBAT voltage divider R2 value (ohms)
#define VBAT_C 0.000001 // Farads
#define VBAT_MEASUREMENT_RATIO 1.0/(VBAT_VD_R2/(VBAT_VD_R1+VBAT_VD_R2)) // Set voltage divider resistor values here
#define MCU_VOLTAGE 3.3
#define PULSES_PER_AXLE_REVOLUTION 8
#define GEAR_RATIO 3.07
#define TIRE_DIAMETER 28.86
#define TIRE_CIRCUMFERENCE 3.14159 * TIRE_DIAMETER
#define PULSES_PER_MILE (5280 / TIRE_CIRCUMFERENCE) * PULSES_PER_AXLE_REVOLUTION
#define BATTERY_MEASURE_INTERVAL 2000*(VBAT_VD_R1*VBAT_C) // ms, allowing 2xtime for the capacitor to charge between reads, T=RC
#define AIRBAG_OK_INTERVAL 1000      // ms

// All instruments
BatteryAndOil battOil;
SingleLamp skimLamp(messageSkim, 3, 10000);
SingleLamp checkGaugesLamp(messageCheckGauges, 4, 500);
SingleLamp checkEngineLamp(messageCheckEngine, 4, 500);
Fuel fuel;
Speedometer speedo;
Tachometer tach;
FeatureStatus featureStatus;
Odometer odometer;
Instrument airbagOk(messageAirbagOk, 3, 0, 255, -1);
Instrument airbagBad(messageAirbagBad, 3, 0, 255, -1);
Instrument *instruments[INSTRUMENT_COUNT] = {
    &fuel, &speedo, &tach, &checkEngineLamp, &checkGaugesLamp,
    &skimLamp, &featureStatus, &battOil, &odometer, &airbagOk,
    &airbagBad};
InstrumentWriter _writer(instruments, INSTRUMENT_COUNT);

IntervalTimer selfTestTimer;
int selfTestPhaseStart;
int selfTestStage = 0;

uint8_t speedoSignal;
double speedoFreqSum;
int speedoFreqCount;

int loopCount;
float speedSensorFrequency;
int speedSensorPulses;
elapsedMillis lastActivity;
elapsedMillis lastCCDLoop;
elapsedMillis lastRefresh;
elapsedMillis lastBattMeasure;
elapsedMillis lastAirbagOkXmt;

ESP32_CAN<RX_SIZE_256, TX_SIZE_16> can1;
ESP32_CAN<RX_SIZE_256, TX_SIZE_16> can2;

void resetGauges()
{
  speedo.SetMPH(0);
  tach.SetRPM(0);
  battOil.SetBatteryVoltage(14);
  battOil.SetOilPressure(20);
  battOil.SetOilTemperature(155);
  if (!SELF_TEST_MODE)
  {
    fuel.SetPercentage(1, 0.0, 1, 254);
  }

  featureStatus.SetCruiseEnabled(false);
  featureStatus.SetUpShift(false);
  checkEngineLamp.SetLamp(false);
  checkGaugesLamp.SetLamp(false);
}

void selfTest()
{
  selfTestPhaseStart = millis();
  selfTestStage++;
  resetGauges();
  Stdout.print("Self test stage ");
  Stdout.println(selfTestStage);

  if (selfTestStage == 3)
  {
    featureStatus.SetCruiseEnabled(true);
  }
  else if (selfTestStage == 4)
  {
    featureStatus.SetUpShift(true);
  }
  else if (selfTestStage == 5)
  {
    checkGaugesLamp.SetLamp(true);
  }
  else if (selfTestStage == 6)
  {
    checkEngineLamp.SetLamp(true);
  }
  else if (selfTestStage == 7)
  {
    battOil.SetBatteryVoltage(16);
  }
  else if (selfTestStage == 8)
  {
    battOil.SetBatteryVoltage(12);
    fuel.SetFuelPercentage(0.75);
  }
  else if (selfTestStage == 9)
  {
    battOil.SetOilPressure(70);
    fuel.SetFuelPercentage(0.25);
  }
  else if (selfTestStage == 10)
  {
    battOil.SetOilTemperature(210);
  }
  else if (selfTestStage > 10)
  {
    selfTestStage = 0;
  }
}

void handleSpeedSensor()
{
  if (FreqCountESP.available() > 0)
  {
    int pulses = FreqCountESP.read();
    speedSensorPulses += pulses;
    speedoFreqSum = speedoFreqSum + pulses;
    speedoFreqCount = speedoFreqCount + 1;
    if (speedoFreqCount > SPEED_SENSOR_SAMPLES)
    {
      // average several readings together
      speedSensorFrequency =
          speedoMeasure.countToFrequency(speedoFreqSum / speedoFreqCount);
      speedoFreqSum = 0;
      speedoFreqCount = 0;
      speedo.SetSpeedSensorFrequency(speedSensorFrequency);
    }
    // Either way, actual pulses are accounted for, and can be used to update
    // the odometer
    if (speedSensorPulses >= PULSES_PER_UPDATE)
    {
      // send an odometer increment
      odometer.AddMiles(speedSensorPulses / PULSES_PER_MILE);
      speedSensorPulses = 0;
    }
  }
}

void CCDHandleError(CCD_Operations op, CCD_Errors err)
{
  if (err == CCD_OK)
    return;

  String s = op == CCD_Read ? "READ " : "WRITE ";

  switch (err)
  {
  case CCD_ERR_BUS_IS_BUSY:
  {
    Stdout.println(s + "CCD_ERR_BUS_IS_BUSY");
    break;
  }
  case CCD_ERR_BUS_ERROR:
  {
    Stdout.println(s + "CCD_ERR_BUS_ERROR");
    break;
  }
  case CCD_ERR_ARBITRATION_LOST:
  {
    Stdout.println(s + "CCD_ERR_ARBITRATION_LOST");
    break;
  }
  case CCD_ERR_CHECKSUM:
  {
    Stdout.println(s + "CCD_ERR_CHECKSUM");
    break;
  }
  default: // unknown error
  {
    Stdout.println(s + "ERR: " + String(err, HEX));
    break;
  }
  }
}

float measureBattery() {
    // TODO: Use ADC
    int battery=analogRead(VBAT_MEASURE_SIG);
    float analogVoltage = battery * (MCU_VOLTAGE/1023.0);
    float batVoltage = analogVoltage * VBAT_MEASUREMENT_RATIO;
    Stdout.print("Calculated VBAT=");
    Stdout.println(batVoltage);
    return batVoltage;
}

void watchdogReset() { Stdout.println("Resetting in 5s..."); }

void canSniff(const CAN_message_t &msg)
{
  Stdout.print("MB ");
  Stdout.print(msg.mb);
  Stdout.print("  OVERRUN: ");
  Stdout.print(msg.flags.overrun);
  Stdout.print("  LEN: ");
  Stdout.print(msg.len);
  Stdout.print(" EXT: ");
  Stdout.print(msg.flags.extended);
  Stdout.print(" TS: ");
  Stdout.print(msg.timestamp);
  Stdout.print(" ID: ");
  Stdout.print(msg.id, HEX);
  Stdout.print(" Buffer: ");
  for (uint8_t i = 0; i < msg.len; i++)
  {
    Stdout.print(msg.buf[i], HEX);
    Stdout.print(" ");
  }
  Stdout.println();
}

void onVCUVehicleInputs3(const CAN_message_t &msg)
{
  uint8_t mph = msg.buf[1];
  speedo.SetMPH(mph);
/*
  // Tell power steering to change
  CAN_message_t leafEPS;
  leafEPS.id = 0x12345;
  leafEPS.buf[0] = 128;
  can2.write(MB1, leafEPS);
*/
}

void onVCUFaultStates(const CAN_message_t &msg)
{
  // Decode fault states, probably don't want to alert on all of them
  for (int i = 0; i < 8; i++)
  {
    if (msg.buf[i] != 0)
    {
      // Oh no!  Set and forget, won't clear without a restart
      checkEngineLamp.SetLamp(true);
    }
  }
}

void setupCAN()
{
  pinMode(6, OUTPUT);
  digitalWrite(6, LOW);
  can1.setTX(CAN1_TX);
  can1.setRX(CAN1_RX);
  can1.begin();

  can1.setBaudRate(500000);
  can2.setBaudRate(500000);

  can1.onReceive = onVCUVehicleInputs3
}

void setup()
{
  Stdout.begin(115200);
  
  // Setup unused GPIOs as pulldowns to GND
  pinMode(19, INPUT_PULLDOWN);
  pinMode(23, INPUT_PULLDOWN);
  
  while (!Stdout);  
  delay (1000);
    
  // Watchdog
  WDT_timings_t config;
  config.trigger = 5;  /* in seconds, 0->128 */
  config.timeout = 10; /* in seconds, 0->128 */
  config.callback = watchdogReset;

  // Give the cluster time to boot
  delay(3000);
  Stdout.println("Entered setup");
  wdt.begin(config);

  // Don't update certain instruments on start
  battOil.SetOilPressure(40);
  battOil.SetBatteryVoltage(14);
  fuel.Quiesce();
  skimLamp.Quiesce();
  airbagBad.Quiesce();
  airbagOk.Quiesce();

  // Did we reset due to watchdog?  Alert on this.
  // Save copy of Reset Status Register
  int lastResetCause = SRC_SRSR;
  // Clear all Reset Status Register bits
  SRC_SRSR = (uint32_t)0x1FF;
  if (lastResetCause == SRC_SRSR_WDOG_RST_B)
  {
    // Yes.  Alert via the SKIM light
    Stdout.println("Lighting SKIM");
    skimLamp.SetLamp(true);
  }

  // Set pins
  pinMode(VBAT_MEASURE_SIG, INPUT);

  CCD.onError(CCDHandleError); // subscribe to the error event and call this
                               // function when an error occurs
  CCD.begin();                 // CDP68HC68S1

  if (USING_SPEED_SENSOR)
  {
    FreqCountESP.begin(SPEEDO_SENSOR_IN, SPEED_SENSOR_WINDOW);
  }
  lastActivity = millis();
  if (SELF_TEST_MODE)
  {
    selfTestTimer.begin(selfTest, (1000 + SELF_TEST_STAGE_DURATION) * 1000);
  }

  pinMode(LED_BUILTIN, OUTPUT);
  // Start the CCD writer
  _writer.Setup(&_writer);
  // setupCAN();
  Stdout.println("Setup complete");
}

CAN_message_t msg;
void loop()
{
  loopCount++;

  // This code expects to use a physical speed sensor to measure actual
  // driveshaft revolutions, and as such keep the odometer up to date.  Tampering
  // with the odometer is a federal crime, so if you don't have the speed sensor,
  // it's up to you to derive distance from speed or other measurements to
  // update the odometer.  Good luck.
  handleSpeedSensor();

  if (SELF_TEST_MODE)
  {
    float t = constrain(float(millis() - selfTestPhaseStart) /
                            SELF_TEST_STAGE_DURATION,
                        0.0, 1.0);
    if (selfTestStage == 1)
    {
      speedo.SetKPH(160 * t);
    }
    else if (selfTestStage == 2)
    {
      tach.SetRPM(6000 * t);
    }
  }
  if (!SELF_TEST_MODE && lastBattMeasure > BATTERY_MEASURE_INTERVAL)
  {
    battOil.SetBatteryVoltage(measureBattery());
    lastBattMeasure = 0;
  }
  if (DISABLE_AIRBAG_LAMP && lastAirbagOkXmt > AIRBAG_OK_INTERVAL)
  {
    airbagOk.Refresh();
    lastAirbagOkXmt = 0;
  }

  if (lastCCDLoop > INTERWRITE_DELAY)
  {
    lastCCDLoop = 0;
    bool newActivity = _writer.Loop();
    activity = activity || newActivity;
  }

  // DAS BLINKENLIGHTS! .. but seriously, blink the builtin LED on activity
  if (activity)
  {
    // Feed the watchdog on activity, since there should be some pretty
    // regularly or something is wrong.
    wdt.feed();
    digitalWrite(LED_BUILTIN, HIGH);
    activity = false;
  }
  else if (lastActivity >= ACTIVITY_ON_MS)
  {
    lastActivity = 0;
    activity = false;
    digitalWrite(LED_BUILTIN, LOW);
  }
}
