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
    AUTHORS OR COPYRIGHTC;        HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
   FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
   IN THE SOFTWARE.
 */

#include "main.h"
#include "instruments.h"
#include <CCDLibrary.h>
#include <Watchdog.h>
#include <elapsedMillis.h>
#include <ESP32_CAN.h>
#include <Arduino.h>
#include <TinyPICO.h>
#include "driver/pcnt.h"
#include <DallasTemperature.h>


//#define BLUETOOTH_CONSOLE
// CAN bus
#define CAN1_TX GPIO_NUM_19
#define CAN1_RX GPIO_NUM_23

#define POST_BOOT_CPU_MHZ 10

#define SELF_TEST_MODE

//#define USING_SPEED_SENSOR
#define SPEEDO_SENSOR_IN 27
#define SPEED_SENSOR_WINDOW 100 //ms
#define SPEEDOMETER_RATIO 1.59
#define SPEEDO_INTERVAL 200 // ms

#define DISABLE_AIRBAG_LAMP // Define if you don't have a working airbag module on the CCD bus
#define INSTRUMENT_COUNT 11
#define ACTIVITY_ON_MS 25 // ms
#define SELF_TEST_STAGE_COUNT 10
#define SELF_TEST_STAGE_DURATION 3000
#define VBAT_VD_R1 4300000.0 // VBAT voltage divider R1 value (ohms)
#define VBAT_VD_R2 820000.0 // VBAT voltage divider R2 value (ohms)
#define VBAT_MEASUREMENT_RATIO 1.0/(VBAT_VD_R2/(VBAT_VD_R1+VBAT_VD_R2)) 
#define MCU_VOLTAGE 3.3
#define PULSES_PER_AXLE_REVOLUTION 8
#define GEAR_RATIO 3.07
#define TIRE_DIAMETER 28.86
#define TIRE_CIRCUMFERENCE 3.14159 * TIRE_DIAMETER
#define PULSES_PER_MILE (5280 / TIRE_CIRCUMFERENCE) * PULSES_PER_AXLE_REVOLUTION
#define BATTERY_MEASURE_INTERVAL 200 // ms
#define VBAT_MEASURE_SIG GPIO_NUM_33
#define AIRBAG_OK_INTERVAL 1000      // ms
#define LOOP_DELAY 25 //ms

// Temperature management
#define THERMAL_LIMIT 100 // degC
#define THERMAL_SLEEP_TIME 300 // seconds
#define TEMP_CHECK_INTERVAL 1000 // ms

// Internal temp
// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS GPIO_NUM_5
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);
// arrays to hold device address
DeviceAddress internalTemp;
bool thermoPresent;

TinyPICO tp = TinyPICO();
Watchdog wdt(5);
bool activity, speedoOn;


//#ifdef BLUETOOTH_CONSOLE
#include <BluetoothSerial.h>
// Bluetooth
#define SERVICE_UUID "3ea24ab1-256b-4baf-ab04-a98f32993856"
volatile bool bluetoothBegan;
elapsedSeconds lastBluetooth;
BluetoothSerial bt;
uint8_t unitMACAddress[6];  // Use MAC address in BT broadcast and display
char deviceName[20];        // The serial string that is broadcast.
//#else 
// Serial port, swap to external pins when not debugging
#define Stdout Serial
//#endif

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

int selfTestPhaseStart;
int selfTestStage = 0;

uint8_t speedoSignal;
//mutex for hardware pulse counters
portMUX_TYPE pcntMux0 = portMUX_INITIALIZER_UNLOCKED;
hw_timer_t *speedoTimer;

int loopCount;
float speedSensorFrequency;
volatile int speedSensorPulses;

elapsedMillis lastActivity;
elapsedMillis lastCCDLoop;
elapsedMillis lastRefresh;
elapsedMillis lastBattMeasure;
elapsedMillis lastAirbagOkXmt;
elapsedMillis lastTempCheck;
elapsedMillis sinceLastStage;

ESP32_CAN<RX_SIZE_256, TX_SIZE_16> can1;


void resetGauges()
{
  speedo.SetMPH(0);
  tach.SetRPM(0);
  battOil.SetBatteryVoltage(14);
  battOil.SetOilPressure(20);
  battOil.SetOilTemperature(155);
  #ifndef SELF_TEST_MODE
  fuel.SetPercentage(1, 0.0, 1, 254);
  #endif

  featureStatus.SetCruiseEnabled(false);
  featureStatus.SetUpShift(false);
  checkEngineLamp.SetLamp(false);
  checkGaugesLamp.SetLamp(false);
}

#ifdef SELF_TEST_MODE
void selfTest()
{
  selfTestPhaseStart = millis();
  selfTestStage++;
  resetGauges();
  Stdout.print("Self test stage ");
  Stdout.println(selfTestStage);
  Stdout.flush();
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
    fuel.SetFuelPercentage(0.75);
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
#endif

#ifdef USING_SPEED_SENSOR
void IRAM_ATTR handleSpeedSensor()
{
  int16_t pulses = 0;
  pcnt_get_counter_value(PCNT_UNIT_0, &pulses);
  if (pulses > 0)
  {
    speedSensorPulses += pulses;
    speedo.SetSpeedSensorFrequency(pulses / 0.200);
  
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
#endif

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
  Stdout.print(" LEN: ");
  Stdout.print(msg.len);
  Stdout.print(" EXT: ");
  Stdout.print(msg.flags.extended);
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

static void onVCUVehicleInputs3(const CAN_message_t &msg)
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

  can1.onReceive(onVCUVehicleInputs3);
}

void setupSpeedo()
{
  #ifdef USING_SPEED_SENSOR
  pinMode(SPEEDO_SENSOR_IN, INPUT_PULLUP);
  pcnt_config_t pcnt_config;

  pcnt_config.pulse_gpio_num = SPEEDO_SENSOR_IN;
  pcnt_config.ctrl_gpio_num = PCNT_PIN_NOT_USED;
  pcnt_config.channel = PCNT_CHANNEL_0;
  pcnt_config.unit = PCNT_UNIT_0;
  pcnt_config.pos_mode = PCNT_COUNT_INC;
  pcnt_config.neg_mode = PCNT_COUNT_DIS;
  pcnt_config.lctrl_mode = PCNT_MODE_KEEP;
  pcnt_config.hctrl_mode = PCNT_MODE_KEEP;
  pcnt_config.counter_h_lim = 10000;
  pcnt_config.counter_l_lim = 0;
    
  pcnt_unit_config(&pcnt_config);
  pcnt_counter_pause(PCNT_UNIT_0);
  pcnt_counter_clear(PCNT_UNIT_0);
  pcnt_counter_resume(PCNT_UNIT_0);

  speedoTimer = timerBegin(0,80,true);  

  // Attach interrupt
  timerAttachInterrupt(speedoTimer, handleSpeedSensor, true);

  timerAlarmWrite(speedoTimer, SPEEDO_INTERVAL * 1000, true);  
  #endif
}

// function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}

//#ifdef BLUETOOTH_CONSOLE
void setupBluetooth() {
  // Get unit MAC address
  esp_read_mac(unitMACAddress, ESP_MAC_WIFI_STA);
  
  // Convert MAC address to Bluetooth MAC (add 2): https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/system.html#mac-address
  unitMACAddress[5] += 2;                                                          
  
  //Create device name
  sprintf(deviceName, "BleBridge-%02X%02X", unitMACAddress[4], unitMACAddress[5]); 
  
  bt.begin(deviceName);
  bluetoothBegan=true;
  //Stdout.setTimeout(10);
}
//#endif

void setup()
{
  tp.DotStar_SetPixelColor(255,255,0);

  //#ifdef BLUETOOTH_CONSOLE
  setupBluetooth();
  //#else
  Stdout.begin(115200);
  while (!Stdout);  
  //#endif
  delay (1000);
  tp.DotStar_SetPixelColor(255,128,0);
    
  /*
  // Watchdog
  WDT_timings_t config;
  config.trigger = 5;  // in seconds, 0->128 
  config.timeout = 10; // in seconds, 0->128 
  config.callback = watchdogReset;
*/
  Stdout.println("Entered setup");

  // locate temp devices on the bus
  Serial.print("Locating temp devices...");
  sensors.begin();
  Serial.print("Found ");
  Serial.print(sensors.getDeviceCount(), DEC);
  Serial.println(" devices.");

  // report parasite power requirements
  Serial.print("Parasite power is: "); 
  if (sensors.isParasitePowerMode()) Serial.println("ON");
  else Serial.println("OFF");

  oneWire.reset_search();
  // assigns the first address found to insideThermometer
  if (!oneWire.search(internalTemp)) {
    Serial.println("Unable to find address for insideThermometer");
  } else {
  // show the addresses we found on the bus
    Serial.print("Device 0 Address: ");
    printAddress(internalTemp);
    Serial.println();
    thermoPresent=true;
  }

  // Setup unused GPIOs as pulldowns to GND
  pinMode(19, INPUT_PULLDOWN);
  pinMode(23, INPUT_PULLDOWN);
  
  // Give the cluster time to boot
  delay(3000);
  //wdt.begin(config);
  Serial.println("A");
  // Don't update certain instruments on start
  battOil.SetOilPressure(40);
  battOil.SetBatteryVoltage(14);
  fuel.Quiesce();
  skimLamp.Quiesce();
  airbagBad.Quiesce();
  airbagOk.Quiesce();

  /*
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
*/

  // Set pins
  pinMode(VBAT_MEASURE_SIG, INPUT);
  Serial.println("B");
  
  CCD.onError(CCDHandleError); // subscribe to the error event and call this
  Serial.println("B2");
                               // function when an error occurs
  CCD.begin(&Stdout);      
           // CDP68HC68S1
  Serial.println("C");

  setupSpeedo();
  Serial.println("D");

  lastActivity = millis();
  tp.DotStar_SetPixelColor(0,128,128);

  // Start the CCD writer
  _writer.Setup(&_writer);
  Serial.println("E");

  //setupCAN();
  Stdout.println("Setup complete");
  tp.DotStar_SetPixelColor(0,128,0);
  tp.DotStar_Clear();
  pinMode(DOTSTAR_PWR, OUTPUT);
  digitalWrite(DOTSTAR_PWR, 0);
  setCpuFrequencyMhz(POST_BOOT_CPU_MHZ);
}

void tempCheck() {
  float tempC = sensors.getTempC(internalTemp);
  if(tempC == DEVICE_DISCONNECTED_C) 
  {
    Serial.println("Error: Could not read temperature data");
    return;
  }
  Serial.print("Temp C: ");
  Serial.print(tempC);

  if (tempC > THERMAL_LIMIT) {
    // Go into deep sleep for a while and hope we cool off
    Serial.println("Thermal limit reached, going to sleep.");
    esp_deep_sleep_start();
    esp_sleep_enable_timer_wakeup(THERMAL_SLEEP_TIME * 1000000);
  }
}

CAN_message_t msg;
void loop()
{
  loopCount++;


  #ifdef SELF_TEST_MODE
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
    if (sinceLastStage > SELF_TEST_STAGE_DURATION) {
      sinceLastStage = 0;
      selfTest();
    } 
  #else
    if (lastBattMeasure > BATTERY_MEASURE_INTERVAL)
    {
      battOil.SetBatteryVoltage(measureBattery());
      lastBattMeasure = 0;
    }
    #ifdef DISABLE_AIRBAG_LAMP
      if (lastAirbagOkXmt > AIRBAG_OK_INTERVAL)
      {
        airbagOk.Refresh();
        lastAirbagOkXmt = 0;
      }
    #endif

    if (lastCCDLoop > INTERWRITE_DELAY)
    {
      lastCCDLoop = 0;
      bool newActivity = _writer.Loop();
      activity = activity || newActivity;
    }

    if (lastTempCheck > TEMP_CHECK_INTERVAL) {
      lastTempCheck = 0;
      tempCheck();
    }

    // DAS BLINKEN LIGHTS! .. but seriously, blink the builtin LED on activity
    if (activity)
    {
      // Feed the watchdog on activity, since there should be some pretty
      // regularly or something is wrong.
      //wdt.feed();
      //digitalWrite(LED_BUILTIN, HIGH);
      activity = false;
    }
    else if (lastActivity >= ACTIVITY_ON_MS)
    {
      lastActivity = 0;
      activity = false;
      //digitalWrite(LED_BUILTIN, LOW);
    }
  #endif

  //#ifdef BLUETOOTH_CONSOLE
  // Disable bluetooth after 2 minutes disconnected to save power
  if (bluetoothBegan && lastBluetooth > 60) {
    if (bt.connected()) {
      lastBluetooth = 0;
    } else {
      Stdout.println("No connection since boot, disabling Bluetooth");
      bt.end();
      bluetoothBegan=false;
    }
  }
  //#endif
}
