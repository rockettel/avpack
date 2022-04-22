#include <Arduino.h>

//#include "board-config.h"

#include <TinyGPS++.h>                       
#include <RadioLib.h>
#include <axp20x.h>

TinyGPSPlus gps;                            
AXP20X_Class axp;

#define LORA_SCK        5
#define LORA_MISO       19
#define LORA_MOSI       27
#define LORA_SS         18
#define LORA_DIO0       26
#define LORA_DIO1       33
#define LORA_DIO2       32
#define LORA_RST        23

SX1276 radio = new Module(LORA_SS, LORA_DIO0, LORA_RST, 0);

static void smartDelay(unsigned long ms)                
{
  unsigned long start = millis();
  do
  {
    while (Serial1.available())
      gps.encode(Serial1.read());
  } while (millis() - start < ms);
}

void setup() {
  Serial.begin(57600);
  delay(4000);
  Wire.begin(21, 22);
  if (!axp.begin(Wire, AXP192_SLAVE_ADDRESS)) {
    Serial.println("AXP192 Begin PASS");
  } else {
    Serial.println("AXP192 Begin FAIL");
  }
  axp.setLDO2Voltage(3300);
  axp.setPowerOutPut(AXP192_LDO2, AXP202_ON); //LoRa
  axp.setPowerOutPut(AXP192_LDO3, AXP202_ON); // GPS
  axp.setPowerOutPut(AXP192_DCDC2, AXP202_ON);  // do i need?
  axp.setPowerOutPut(AXP192_EXTEN, AXP202_ON);  // do I need?
  axp.setPowerOutPut(AXP192_DCDC1, AXP202_ON); // do I need?
  Serial1.begin(9600, SERIAL_8N1, 34, 12);   //17-TX 18-RX

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI);

  int state = radio.begin(915.0, 125.0, 9, 7, RADIOLIB_SX126X_SYNC_WORD_PRIVATE, 10, 8);
  radio.setCRC(true, false);

  Serial.print("State :");
  Serial.println(state, 10);
}

int ctr = 0;

void loop() {
  char buffer[40];

  ctr++;
  sprintf(buffer, "Hello %d World %08x\n", ctr, ctr);
  int state = radio.transmit(buffer);

  if (state == RADIOLIB_ERR_NONE) {
        // the packet was successfully transmitted
        Serial.println(F(" success!"));

        // print measured data rate
        Serial.print(F("[SX1278] Datarate:\t"));
        Serial.print(radio.getDataRate());
        Serial.println(F(" bps"));

} else if (state == RADIOLIB_ERR_PACKET_TOO_LONG) {
        // the supplied packet was longer than 256 bytes
        Serial.println(F("too long!"));

    } else if (state == RADIOLIB_ERR_TX_TIMEOUT) {
        // timeout occurred while transmitting packet
        Serial.println(F("timeout!"));

    } else {
        // some other error occurred
        Serial.print(F("failed, code "));
        Serial.println(state);

    }

    // wait for a second before transmitting again
    delay(1000);


  return;
  Serial.print("Latitude  : ");
  Serial.println(gps.location.lat(), 5);
  Serial.print("Longitude : ");
  Serial.println(gps.location.lng(), 4);
  Serial.print("Satellites: ");
  Serial.println(gps.satellites.value());
  Serial.print("Altitude  : ");
  Serial.print(gps.altitude.feet() / 3.2808);
  Serial.println("M");
  Serial.print("Time      : ");
  Serial.print(gps.time.hour());
  Serial.print(":");
  Serial.print(gps.time.minute());
  Serial.print(":");
  Serial.println(gps.time.second());
  Serial.println("**********************");

  smartDelay(1000);                                      

  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));
}

