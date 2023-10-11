#include "crc32.hpp"
#include "data_link.hpp"
#include <SoftwareSerial.h>

const int softwareSerialRX = 10; // RX pin for SoftwareSerial
const int softwareSerialTX = 11; // TX pin for SoftwareSerial

SoftwareSerial Serial4(softwareSerialRX, softwareSerialTX, false); // RX, TX

DataLinkLayer<HardwareSerial, HardwareSerial>  *eth1;
DataLinkLayer<HardwareSerial, HardwareSerial>  *eth2;
DataLinkLayer<HardwareSerial, HardwareSerial>  *eth3;
DataLinkLayer<SoftwareSerial, HardwareSerial>  *eth4;

const int MAX_BUFFER_SIZE = 16; // Maximum size of the input buffer

uint8_t payload[MAX_BUFFER_SIZE + 1] = {0x00};

void setup()
{
  delay(2000);
  Serial.begin(9600);
  Serial.println("Initialize");

  eth1 = new DataLinkLayer<HardwareSerial, HardwareSerial>(&Serial1, &Serial, 0xF6, 9600);
  eth2 = new DataLinkLayer<HardwareSerial, HardwareSerial>(&Serial2, &Serial, 0xF7, 9600);
  eth3 = new DataLinkLayer<HardwareSerial, HardwareSerial>(&Serial3, &Serial, 0xF8, 9600);
  eth4 = new DataLinkLayer<SoftwareSerial, HardwareSerial>(&Serial4, &Serial, 0xF9, 9600);

  sei(); // Enable interrupts
}

void loop()
{

  // if (eth1.is_connected() && !eth1.isTxQueueFull()) {
  //   uint8_t size = Serial4.readBytesUntil('\n', payload, MAX_BUFFER_SIZE);  // Read a line from the serial monitor

  //   if (size > 0) {
  //     payload[size] = 0x00;
  //     Serial4.println(reinterpret_cast<char*>(payload));
  //     eth1.tx_push(payload);
  //   }

  // }

  // if (eth2.rx_pop(payload)) {
  //   Serial4.println(reinterpret_cast<char*>(payload));
  // }

  // if (eth3.rx_pop(payload)) {
  //   Serial4.println(reinterpret_cast<char*>(payload));
  // }

  // if (eth4.rx_pop(payload)) {
  //   Serial4.println(reinterpret_cast<char*>(payload));
  // }

  eth1->run();
  eth2->run();
  eth3->run();
  eth4->run();
}