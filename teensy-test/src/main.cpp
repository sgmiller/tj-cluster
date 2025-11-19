#include "main.h"

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> myCan;

void canSniff(const CAN_message_t &msg) {
  Serial.print("MB "); Serial.print(msg.mb);
  Serial.print("  OVERRUN: "); Serial.print(msg.flags.overrun);
  Serial.print("  LEN: "); Serial.print(msg.len);
  Serial.print(" EXT: "); Serial.print(msg.flags.extended);
  Serial.print(" TS: "); Serial.print(msg.timestamp);
  Serial.print(" ID: "); Serial.print(msg.id, HEX);
  Serial.print(" Buffer: ");
  for ( uint8_t i = 0; i < msg.len; i++ ) {
    Serial.print(msg.buf[i], HEX); Serial.print(" ");
  } Serial.println();
}

CAN_message_t msg;
uint64_t loopCount;


#define LED 13

void setup() {
      pinMode(LED, OUTPUT);

  delay(3000);
  Serial.begin(115200);
    Serial.println("Beginning");
    myCan.begin();
    //myCan.setMB(MB0, RX, STD);
    //myCan.setMB(MB1, TX);
    Serial.println("Set mailboxes");
    myCan.setBaudRate(1000000);
    Serial.println("Added interrupt");
    //myCan.enableMBInterrupt(MB1);
    Serial.println("Enabled interrupt");

    myCan.setMaxMB(16);
    myCan.enableFIFO();
    myCan.enableFIFOInterrupt();
    myCan.onReceive(canSniff);
    myCan.mailboxStatus();
}


static uint32_t timeout = millis();


void loop() {
  myCan.events();
  if ( millis() - timeout > 200 ) {
    msg.id = random(0x1,0x7FE);
    loopCount++;
    for ( uint8_t i = 0; i < 8; i++ ) msg.buf[i] = loopCount >> (8*i) & 0xff;
    myCan.write(msg);
    timeout = millis();
    Serial.print(".");
    Serial.flush();
  }
}