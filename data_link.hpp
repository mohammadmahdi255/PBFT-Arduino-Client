#ifndef __DATALINK_HPP
#define __DATALINK_HPP

#include "Type.h"
#include "SoftwareSerial.h"
#include "HardwareSerial.h"
#include <stdint.h>
#include <Arduino.h>
#include "crc32.hpp"
#include <cppQueue.h>

const int MAX_QUEUE_SIZE = 10;

const uint8_t SFD = 0x7E;
const uint8_t BROAD_CAST = 0xFF;
const uint8_t ARP = 0x08;
const uint8_t ARP_REQUEST = 0x01;
const uint8_t ARP_REPLY = 0x02;

enum class DL_ST
{
  ARP_REQUEST,
  WAIT_ARP_REPLY,
  ESTABLISHED
};



template <typename T, typename K>
class DataLinkLayer
{
private:
  crc32_generator<IEEE8023_CRC32_POLYNOMIAL> crc_generator;

  cppQueue txQueue;
  cppQueue rxQueue;

  DL_ST state;
  uint8_t src_mac;
  uint8_t dest_mac;

  bool tx_transfer;
  uint8_t tx_buffer[24];
  uint8_t tx_size;

  bool rx_recving;
  uint8_t rx_buffer[24];
  uint8_t rx_size;

  T *serialPort;
  K *monitor;

  void make_arp(uint8_t payload[16], uint8_t operation)
  {
    payload[0] = ARP;
    payload[1] = operation;
    payload[2] = src_mac;
    payload[3] = dest_mac;
    payload[4] = 0x00;
  }

  bool handle_tx_frame()
  {
    if (txQueue.isEmpty())
    {
      return false;
    }

    tx_size = 0;
    tx_buffer[tx_size++] = src_mac;
    tx_buffer[tx_size++] = dest_mac;
    tx_buffer[tx_size++] = 0x00; // Set Later

    uint8_t size = 0;
    uint8_t payload[16];
    txQueue.pop(payload);

    while (size < 16 && payload[size] != 0x00)
    {
      tx_buffer[tx_size++] = payload[size++];
    }

    tx_buffer[2] = size;
    uint32_t crc = crc_generator.calculate_crc32(tx_buffer, tx_size, 0x00000000, 0x00000000);

    tx_buffer[tx_size++] = (crc >> 24) & 0xFF;
    tx_buffer[tx_size++] = (crc >> 16) & 0xFF;
    tx_buffer[tx_size++] = (crc >> 8) & 0xFF;
    tx_buffer[tx_size++] = crc & 0xFF;

    return true;
  }

  void handle_rx_frame(uint8_t* payload)
  {

    if (payload[0] == ARP)
    {

      switch (payload[1]) {
      case ARP_REQUEST:
        monitor->println("Send Reply");
        make_arp(payload, ARP_REPLY);
        txQueue.push(payload);
        break;
      case ARP_REPLY:
        monitor->print("ESTABLISHED 0x");
        monitor->print(this->src_mac, HEX);
        monitor->print(" with dest mac: 0x");
        monitor->println(payload[2], HEX);
        dest_mac = payload[2];
        state = DL_ST::ESTABLISHED;
        break;
      }

    }
    else if (state == DL_ST::ESTABLISHED && !rxQueue.isFull())
    {
      rxQueue.push(payload);
    }

  }

  uint8_t reverse_bits(uint8_t byte)
  {
    uint8_t reversedByte = 0;

    for (int8_t i = 7; i >= 0; i--)
    {
      reversedByte |= ((byte & 1) << i);
      byte >>= 1;
    }

    return reversedByte;
  }

  void print_buffer(uint8_t *buffer, uint8_t size)
  {
    String str = "==============================================================";
    for (int i = 0; i < size; ++i)
    {
      str += "0x";
      if (buffer[i] < 16)
        str += "0";
      str += String(buffer[i], HEX);
      str += " ";
    }
    str += "==============================================================";
    monitor->println(str);
  }

public:
  DataLinkLayer(T *port, K *monitor, uint8_t src_mac, uint32_t baud_rate) : 
  txQueue(sizeof(uint8_t) * 16, MAX_QUEUE_SIZE),
  rxQueue(sizeof(uint8_t) * 16, MAX_QUEUE_SIZE)
  {
    serialPort = port;
    serialPort->begin(baud_rate);
    this->monitor = monitor;
    // monitor->begin(9600);

    this->src_mac = src_mac;
    dest_mac = 0xFF;
    state = DL_ST::ARP_REQUEST;
    tx_transfer = false;
    rx_recving = false;
    tx_size = 0;
    rx_size = 0;

    txQueue.flush();
    rxQueue.flush();
  }

  bool isRxQueueEmpty()
  {
    return rxQueue.isEmpty();
  }

  bool isTxQueueFull()
  {
    return txQueue.isFull();
  }

  void tx_push(const uint8_t payload[16])
  {
    if (!txQueue.isFull())
    {
      txQueue.push(payload);
    }
  }

  bool rx_pop(uint8_t payload[16])
  {
    if (rxQueue.isEmpty())
    {
      memset(payload, 0, sizeof(payload));
      return false;
    }

    rxQueue.pop(payload);
    return true;
  }

  bool is_connected()
  {
    return state == DL_ST::ESTABLISHED;
  }

  void run()
  {

    uint8_t payload[16];

    if (state == DL_ST::ARP_REQUEST)
    {
      make_arp(payload, ARP_REQUEST);
      txQueue.push(payload);

      monitor->print("ARP REQUEST NIC 0x");
      monitor->println(this->src_mac, HEX);
      state = DL_ST::WAIT_ARP_REPLY;
    }

    if (!tx_transfer)
    {
      tx_transfer = handle_tx_frame();
    }
    else if (serialPort->availableForWrite() >= tx_size || is_same<T, SoftwareSerial>::value)
    {
      serialPort->write(reverse_bits(SFD));
      for (int i = 0; i < tx_size; i++)
        serialPort->write(reverse_bits(tx_buffer[i]));

      monitor->print("SEND NIC 0x");
      monitor->println(this->src_mac, HEX);
      print_buffer(tx_buffer, tx_size);
      tx_transfer = false;
    }

    while (serialPort->available() > 0)
    {
      uint8_t byte = reverse_bits(serialPort->read());

      if (rx_recving)
      {
        rx_buffer[rx_size++] = byte;
        if (rx_size < 7 + rx_buffer[2])
          continue;

        uint32_t crc = crc_generator.calculate_crc32(rx_buffer, rx_size, 0x00000000, 0x00000000);
        rx_recving = false;
        rx_size = 0;

        if (crc != 0)
          break;

        if (dest_mac != BROAD_CAST && rx_buffer[0] != dest_mac)
          break;

        if (rx_buffer[1] != BROAD_CAST && rx_buffer[1] != src_mac)
          break;

        monitor->print("RECIEVED NIC 0x");
        monitor->println(this->src_mac, HEX);
        print_buffer(rx_buffer, rx_buffer[2] + 7);

        memcpy(payload, rx_buffer + 3, rx_buffer[2]);
        payload[rx_buffer[2]] = 0x00;

        handle_rx_frame(payload);
      }
      else if (byte == SFD)
      {
        rx_buffer[2] = 0x00;
        rx_recving = true;
      }
    }

    
  }
};

#endif