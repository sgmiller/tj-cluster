/*
    XJ/TJ CCD/CAN bus arbiter
    (C) 2025 N9 Works/ Scott Miller

    This code is rather specific to driving a Jeep XJ or TJ cluster dash
    with values from onboard sensors and/or the CAN bus to convert to CCD
    bus signaling in the message formats expected for those vehicles'
    instrument clusters.

    This code is MIT licensed, however the dependent CCDLibrary is GPL, which
    carries extra requirements, including publication of the source code if
    used in a commercial project.

    Copyright (c) 2025 Scott G Miller

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
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include <DallasTemperature.h>


//#define BLUETOOTH_CONSOLE
// CAN bus
#define CAN1_TX GPIO_NUM_23
#define CAN1_RX GPIO_NUM_19

#define POST_BOOT_CPU_MHZ 20

//#define SELF_TEST_MODE

#define BATT_DEBUG
#define BENCH_BATT
#define SPEEDO_DEBUG
#define USING_SPEED_SENSOR
#define INTERNAL_THERMOMETER
#define SPEEDO_SENSOR_IN 26
#define SPEED_SENSOR_WINDOW 100 //ms
#define SPEEDOMETER_RATIO 1.59
#define SPEEDO_INTERVAL 200 // ms
#define SPEEDO_SAMPLE_COUNT 5

#define DISABLE_AIRBAG_LAMP // Define if you don't have a working airbag module on the CCD bus
#define INSTRUMENT_COUNT 10
#define ACTIVITY_ON_MS 25 // ms
#define SELF_TEST_STAGE_COUNT 10
#define SELF_TEST_STAGE_DURATION 3000
#define VBAT_VD_R1 10040000.0 // VBAT voltage divider R1 value (ohms), ideally measured
#define VBAT_VD_R2 1492000.0 // VBAT voltage divider R2 value (ohms), ideally measured
#define VBAT_MEASUREMENT_RATIO 1.0/(VBAT_VD_R2/(VBAT_VD_R1+VBAT_VD_R2)) 
#define MCU_VOLTAGE 3.3
#define PULSES_PER_AXLE_REVOLUTION 8
#define BATTERY_WINDOW_LENGTH 10
#define BATTERY_SAMPLE_COUNT 20
#define VBAT_MEASURE_SIG ADC1_CHANNEL_5  
#define BATTERY_MEASURE_INTERVAL 1000 / (BATTERY_WINDOW_LENGTH * BATTERY_SAMPLE_COUNT) // ms
esp_adc_cal_characteristics_t adc1_chars;

#define AIRBAG_OK_INTERVAL 1000      // ms
#define LOOP_DELAY 25 //ms

// Temperature management
#define THERMAL_LIMIT 100 // degC
#define THERMAL_SLEEP_TIME 300 // seconds
#define TEMP_CHECK_INTERVAL 1000 // ms

#ifdef INTERNAL_THERMOMETER
// Internal temp
// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS GPIO_NUM_5
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature thermometer(&oneWire);

// arrays to hold device address
DeviceAddress internalTemp;
bool thermoPresent;
#endif

TinyPICO tp = TinyPICO();
Watchdog wdt(5);
bool activity, speedoOn;
volatile float speedoFreq;
float lastSpeedo;

#ifdef BLUETOOTH_CONSOLE
#include <BluetoothSerial.h>
// Bluetooth
#define SERVICE_UUID "3ea24ab1-256b-4baf-ab04-a98f32993856"
volatile bool bluetoothBegan;
elapsedSeconds lastBluetooth;
BluetoothSerial Stdout;
uint8_t unitMACAddress[6];  // Use MAC address in BT broadcast and display
char deviceName[20];        // The serial string that is broadcast.
#else 
// Serial port, swap to external pins when not debugging
#define Stdout Serial
#endif

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
// In priority order
Instrument *instruments[INSTRUMENT_COUNT] = {
    &checkGaugesLamp, &skimLamp, &battOil, &fuel, &speedo, &tach, 
    &featureStatus, &odometer, &airbagOk, &airbagBad};
InstrumentWriter _writer(instruments, INSTRUMENT_COUNT);

int selfTestPhaseStart;
int selfTestStage = 0;

uint32_t loopCount;
float speedSensorFrequency;
volatile int speedSensorPulses;

u_int16_t batteryWindow[BATTERY_WINDOW_LENGTH];
u_int16_t batterySamples[BATTERY_SAMPLE_COUNT];
u_int8_t batterySamplePtr;
u_int8_t batteryWindowPtr;

elapsedMillis lastActivity;
elapsedMillis lastCCDLoop;
elapsedMillis lastRefresh;
elapsedMillis lastBattMeasure;
elapsedMillis lastAirbagOkXmt;
elapsedMillis lastTempCheck;
elapsedMillis sinceLastStage;
elapsedMicros sinceLastSpeedoPulse;

float speedoSamples[SPEEDO_SAMPLE_COUNT];
int speedoSamplePtr;

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
    featureStatus.SetCruiseEnabled(false);
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
void ICACHE_RAM_ATTR handleSpeedSensor()
{
    speedSensorPulses ++;
    speedoFreq=1000000.0 / sinceLastSpeedoPulse;
    sinceLastSpeedoPulse = 0;
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
    u_int32_t bat_mv = esp_adc_cal_raw_to_voltage(adc1_get_raw(VBAT_MEASURE_SIG), &adc1_chars);
    batterySamples[batterySamplePtr] = bat_mv;
    batterySamplePtr = (batterySamplePtr + 1) % BATTERY_SAMPLE_COUNT;

    if (batterySamplePtr == 0) {
      float batTot = 0;
      for (int i = 0; i< BATTERY_SAMPLE_COUNT; i++) {
        batTot += batterySamples[i];
      }
      batteryWindow[batteryWindowPtr] = batTot / BATTERY_SAMPLE_COUNT;

      batTot = 0;
      for (int i=0; i< BATTERY_WINDOW_LENGTH; i++) {
        batTot += batteryWindow[i];
      }
      batteryWindowPtr = (batteryWindowPtr + 1) % BATTERY_WINDOW_LENGTH;
      float windowAvg = batTot / BATTERY_WINDOW_LENGTH;
      float battV = VBAT_MEASUREMENT_RATIO * windowAvg / 1000.0;
      #ifdef BATT_DEBUG
      Stdout.print("Measured battery = ");
      Stdout.println(battV);
      #endif
      #ifdef BENCH_BATT
      battV = battV + 9;
      #endif
      return battV;
    }

    return -1;
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
  pinMode(SPEEDO_SENSOR_IN, INPUT);
  attachInterrupt(digitalPinToInterrupt(SPEEDO_SENSOR_IN), handleSpeedSensor, FALLING);
  #endif
}

// function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) Stdout.print("0");
    Stdout.print(deviceAddress[i], HEX);
  }
}

void setupBluetooth() {
#ifdef BLUETOOTH_CONSOLE
  // Get unit MAC address
  esp_read_mac(unitMACAddress, ESP_MAC_WIFI_STA);
  
  // Convert MAC address to Bluetooth MAC (add 2): https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/system.html#mac-address
  unitMACAddress[5] += 2;                                                          
  
  //Create device name
  sprintf(deviceName, "BleBridge-%02X%02X", unitMACAddress[4], unitMACAddress[5]); 
  
  Serial.print("Starting Bluetooth on device ");
  Serial.println(deviceName);
  Stdout.begin(deviceName);
  bluetoothBegan=true;
  //Stdout.setTimeout(10);
#endif
}

void setupADC() {
  adc1_config_width(adc_bits_width_t(ADC_WIDTH_BIT_DEFAULT));
  adc1_config_channel_atten(ADC1_CHANNEL_5,ADC_ATTEN_0db);
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_0db, adc_bits_width_t(ADC_WIDTH_BIT_DEFAULT), 0, &adc1_chars);
}

void setup()
{
  tp.DotStar_SetPixelColor(255,165,0);
  tp.DotStar_SetBrightness(96);
  setCpuFrequencyMhz(80);
  setupADC();
 
  #ifdef BLUETOOTH_CONSOLE
  setupBluetooth();
  #else
  Stdout.begin(115200);
  while (!Stdout);
  delay(1000);
  #endif
  tp.DotStar_SetPixelColor(255,128,0);
    
  /*
  // Watchdog
  WDT_timings_t config;
  config.trigger = 5;  // in seconds, 0->128 
  config.timeout = 10; // in seconds, 0->128 
  config.callback = watchdogReset;
*/
  Stdout.println("Entered setup");

  #ifdef INTERNAL_THERMOMETER
  // locate temp devices on the bus
  Stdout.print("Locating temp devices...");
  thermometer.begin();
  Stdout.print("Found ");
  Stdout.print(thermometer.getDeviceCount(), DEC);
  Stdout.println(" devices.");

  // report parasite power requirements
  Stdout.print("Parasitic power is: "); 
  if (thermometer.isParasitePowerMode()) Stdout.println("ON");
  else Stdout.println("OFF");

  oneWire.reset_search();
  // assigns the first address found to insideThermometer
  if (!oneWire.search(internalTemp)) {
    Stdout.println("Unable to find address for insideThermometer");
  } else {
  // show the addresses we found on the bus
    Stdout.print("Device 0 Address: ");
    printAddress(internalTemp);
    Stdout.println();
    thermoPresent=true;
  }
  #endif

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
  Stdout.println("B");
  
  CCD.onError(CCDHandleError); // subscribe to the error event and call this
  Stdout.println("B2");
                               // function when an error occurs
  Serial.println("C");

  setupSpeedo();
  Stdout.println("D");

  lastActivity = millis();
  tp.DotStar_SetPixelColor(0,128,128);

  Stdout.println("E");

  //setupCAN();
  
  #ifndef BLUETOOTH_CONSOLE
  // Reset serial to accommodate new CPU speed
//  Serial.end();
  //Serial.begin(115200);
  #endif
  // Start the CCD writer
  CCD.begin(&Stdout);    
  Serial.println("F");
  _writer.Setup(&_writer);
  Serial.println("Setup complete");
  tp.DotStar_SetPixelColor(0,128,0);
  tp.DotStar_Clear();
  pinMode(DOTSTAR_PWR, OUTPUT);
  digitalWrite(DOTSTAR_PWR, 0);
  Serial.flush();
}

void tempCheck() {
  #ifdef INTERNAL_THERMOMETER
  if (thermoPresent) {
    thermometer.requestTemperatures();
    float tempC = thermometer.getTempC(internalTemp);
    if(tempC == DEVICE_DISCONNECTED_C) 
    {
      Stdout.println("Error: Could not read temperature data");
      return;
    }
    #ifdef THERMO_DEBUG
    Stdout.print("Temp C: ");
    Stdout.println(tempC);
    #endif
    battOil.SetOilPressure(int(tempC));

    if (tempC > THERMAL_LIMIT) {
      // Go into deep sleep for a while and hope we cool off
      Stdout.println("Thermal limit reached, going to sleep.");
      esp_sleep_enable_timer_wakeup(THERMAL_SLEEP_TIME * 1000000);
      esp_deep_sleep_start();
    }
  }
  #endif
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
      float battery = measureBattery();
      if (battery > -1) {
        battOil.SetBatteryVoltage(battery);
      }
      lastBattMeasure = 0;
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
    if (sinceLastSpeedoPulse > 1000000 && speedSensorPulses == 0) {
      speedoFreq = 0;
      if (speedoFreq > 0) {
        speedo.SetSpeedSensorFrequency(0);
      }
    } else if (speedSensorPulses > 0) {
      speedoSamples[speedoSamplePtr] = speedoFreq;
      float speedoSum = 0;
      for (int i=0; i<SPEEDO_SAMPLE_COUNT; i++) {
        speedoSum += speedoSamples[i];
      }
      float speedoAvg = speedoSum / SPEEDO_SAMPLE_COUNT;
      speedo.SetSpeedSensorFrequency(speedoAvg);
    
      // Either way, actual pulses are accounted for, and can be used to update
      // the odometer
      if (speedSensorPulses >= PULSES_PER_UPDATE)
      {
        // send an odometer increment
        //odometer.AddMiles(speedSensorPulses / PULSES_PER_MILE);
        speedSensorPulses = 0;
      }
  #ifdef SPEEDO_DEBUG
      Stdout.print("Speedo freq now ");
      Stdout.println(speedoFreq);
  #endif
    }
  }



  #ifdef BLUETOOTH_CONSOLE
  // Disable bluetooth after 2 minutes disconnected to save power
  if (false && bluetoothBegan && lastBluetooth > 60) {
    if (Stdout.connected()) {
      lastBluetooth = 0;
      while (Stdout.available()) {
        Stdout.read();
      }
    } else {
      Stdout.println("No connection since boot, disabling Bluetooth");
      Stdout.end();
      bluetoothBegan=false;
    }
  }
  #endif
}
