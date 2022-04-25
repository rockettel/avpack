#include <Arduino.h>

//#include "board-config.h"

#include <TinyGPS++.h>                       
#include <RadioLib.h>
#include <axp20x.h>
#include <Wire.h>

#include <AdaFruit_BMP280.h>

#include "RocketTelCommon.h"

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
Adafruit_BMP280 bmp;

volatile bool axpIrq = false;
void setFlag(void) {
  Serial.println("AXPIRQ");
  axpIrq = true;
}

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
  Serial.begin(115200);
 

  Wire.begin(21, 22);
  if (!axp.begin(Wire, AXP192_SLAVE_ADDRESS)) {
    Serial.println("AXP192 Begin PASS");
  } else {
    Serial.println("AXP192 Begin FAIL");
  }

  
  if (!bmp.begin(0x76)) {
    Serial.println("NO BMP");
  }

  bmp.getTemperatureSensor()->printSensorDetails();
  bmp.getPressureSensor()->printSensorDetails();

  
  pinMode(35, INPUT_PULLUP);
  attachInterrupt(35, setFlag, FALLING);
  axp.clearIRQ();
  axp.enableIRQ(AXP202_ALL_IRQ, true);
  
  axp.setLDO2Voltage(3300);
  axp.setPowerOutPut(AXP192_LDO2, AXP202_ON); //LoRa
  axp.setPowerOutPut(AXP192_LDO3, AXP202_ON); // GPS
  axp.setPowerOutPut(AXP192_DCDC2, AXP202_ON);  // do i need?
  axp.setPowerOutPut(AXP192_EXTEN, AXP202_ON);  // do I need?
  axp.setPowerOutPut(AXP192_DCDC1, AXP202_ON); // do I need?

  Serial1.begin(9600, SERIAL_8N1, 34, 12);   //17-TX 18-RX
  axp.enableChargeing(true);
  axp.EnableCoulombcounter();
  axp.adc1Enable(0xFF, true);
  axp.setTimer(1);

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI);

  int state = radio.begin(915.0, 125.0, 9, 7, RADIOLIB_SX126X_SYNC_WORD_PRIVATE, 10, 8);
  radio.setCRC(true, false);

  Serial.print("State :");
  Serial.println(state, 10);
}

int ctr = 0;

//#define BME280_ADDRESS 0x77
void getTelemetry(DataPacket &packet) {
  float p = bmp.readPressure()/100.0;
  float t = bmp.readTemperature();  
  packet.packTPHData(t, p, 0.0f);
  
}

void loop() {
byte error, address;

  uint8_t buffer[64];
  uint32_t len;
  ctr++;

  
  /*
  if (axpIrq) {
    axpIrq = false;
    int irq = axp.readIRQ();
  }
  */
  
  /*
  Serial.print("Loop: ");
  Serial.println(ctr);
  */

  DataPacket packet;

  packet.initHeader(1, 1);

  if (gps.location.isUpdated()) {
    Serial.println("gps is updated");
    packet.packGPSData(gps);
  }

  getTelemetry(packet);


  packet.writeBitsInt(6, 0);
  packet.getBuffer(buffer, &len);
  int state = radio.transmit(buffer, len);

  if (state == RADIOLIB_ERR_NONE) {
        // the packet was successfully transmitted
        //Serial.println(F(" success!"));

        // print measured data rate
        /*
        Serial.print(F("[SX1278] Datarate:\t"));
        Serial.print(radio.getDataRate());
        Serial.println(F(" bps"));
        */
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
    smartDelay(1000);
}

