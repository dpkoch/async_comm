/*
 * Software License Agreement (BSD-3 License)
 *
 * Copyright (c) 2018 Daniel Koch.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file serial_protocol.cpp
 * @author Daniel Koch <danielpkoch@gmail.com>
 */

#define NUM_MSGS 40000
#define BAUD_RATE 921600

#define ASYNC_COMM_READ_BUFFER_SIZE 512
#define ASYNC_COMM_WRITE_BUFFER_SIZE 256

#include <async_comm/serial.h>

#include <cstdint>
#include <cstdio>

#include <chrono>

#define START_BYTE 0xA5

#define START_BYTE_LEN 1
#define PAYLOAD_LEN 12
#define CRC_LEN 1
#define PACKET_LEN (START_BYTE_LEN + PAYLOAD_LEN + CRC_LEN)

#define NUM_START_BITS 1
#define NUM_STOP_BITS 1

uint8_t update_crc(uint8_t inCrc, uint8_t inData)
{
  uint8_t   i;
  uint8_t   data;

  data = inCrc ^ inData;

  for ( i = 0; i < 8; i++ )
  {
    if (( data & 0x80 ) != 0 )
    {
      data <<= 1;
      data ^= 0x07;
    }
    else
    {
      data <<= 1;
    }
  }
  return data;
}

void pack_message(uint8_t* dst, uint32_t id, uint32_t v1, uint32_t v2)
{
  dst[0] = START_BYTE;
  memcpy(dst+1, &id, 4);
  memcpy(dst+5, &v1, 4);
  memcpy(dst+9, &v2, 4);

  uint8_t crc = 0;
  for (size_t i = 0; i < PACKET_LEN-1; i++)
  {
    crc = update_crc(crc, dst[i]);
  }
  dst[PACKET_LEN-1] = crc;
}

void unpack_payload(uint8_t* src, uint32_t *id, uint32_t *v1, uint32_t *v2)
{
  memcpy(id, src, 4);
  memcpy(v1, src+4, 4);
  memcpy(v2, src+8, 4);
}

bool received[NUM_MSGS];

enum ParseState
{
  PARSE_STATE_IDLE,
  PARSE_STATE_GOT_START_BYTE,
  PARSE_STATE_GOT_PAYLOAD
};

ParseState parse_state = PARSE_STATE_IDLE;
uint8_t receive_buffer[PAYLOAD_LEN];

int receive_count = 0;

void parse_byte(uint8_t byte)
{
  static size_t payload_count;
  static uint8_t crc;

  switch (parse_state)
  {
  case PARSE_STATE_IDLE:
    if (byte == START_BYTE)
    {
      payload_count = 0;
      crc = 0;
      crc = update_crc(crc, byte);

      parse_state = PARSE_STATE_GOT_START_BYTE;
    }
    break;
  case PARSE_STATE_GOT_START_BYTE:
    receive_buffer[payload_count] = byte;
    crc = update_crc(crc, byte);
    if (++payload_count >= PAYLOAD_LEN)
    {
      parse_state = PARSE_STATE_GOT_PAYLOAD;
    }
    break;
  case PARSE_STATE_GOT_PAYLOAD:
    if (byte == crc)
    {
      uint32_t id, v1, v2;
      unpack_payload(receive_buffer, &id, &v1, &v2);
      received[id] = true;
      receive_count++;
    } // otherwise ignore it
    parse_state = PARSE_STATE_IDLE;
    break;
  }
}

void callback(const uint8_t* buf, size_t len)
{
  for (size_t i = 0; i < len; i++)
  {
    parse_byte(buf[i]);
  }
}

int main(int argc, char** argv)
{
  // initialize
  char* port;
  if (argc < 2)
  {
    std::printf("USAGE: %s PORT\n", argv[0]);
    return 1;
  }
  else
  {
    std::printf("Using port %s\n", argv[1]);
    port = argv[1];
  }

  // open serial port
  async_comm::Serial serial(port, BAUD_RATE);
  serial.register_receive_callback(&callback);

  if (!serial.init())
  {
    std::printf("Failed to initialize serial port\n");
    return 2;
  }

  memset(received, 0, sizeof(received));

  auto start = std::chrono::high_resolution_clock::now();

  uint8_t buffer[PACKET_LEN];
  for (uint32_t i = 0; i < NUM_MSGS; i++)
  {
    pack_message(buffer, i, i*2, i*4);
    serial.send_bytes(buffer, PACKET_LEN);
  }
  auto finish_write = std::chrono::high_resolution_clock::now();

  while (receive_count < NUM_MSGS);

  auto finish_read = std::chrono::high_resolution_clock::now();

  // close serial port
  serial.close();

  // did we get all the messages back?
  int num_received = 0;
  for (int i = 0; i < NUM_MSGS; i++)
  {
    if (received[i])
    {
      num_received++;
    }
    else
    {
      std::printf("Missing message %d\n", i);
    }
  }

  std::chrono::duration<double, std::milli> write_time = finish_write - start;
  std::chrono::duration<double, std::milli> read_time = finish_read - start;

  std::printf("Received %d of %d messages\n", num_received, NUM_MSGS);
  std::printf("Elapsed write time: %fms\n", write_time.count());
  std::printf("Elapsed read time: %fms\n", read_time.count());

  int num_bytes = NUM_MSGS * PACKET_LEN;
  double expected_time = num_bytes * (8 + NUM_START_BITS + NUM_STOP_BITS) / (double) BAUD_RATE;
  std::printf("Expected read time: %fms\n", expected_time*1e3);
  std::printf("Total: %d bytes\n", num_bytes);

  return 0;
}
