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
 * @file serial_loopback.cpp
 * @author Daniel Koch <danielpkoch@gmail.com>
 */

#include <async_comm/serial.h>

#include <cstdint>
#include <cstdio>

#include <chrono>
#include <thread>

#define NUM_BYTES 64

void echo(uint8_t byte)
{
  std::printf("Received byte: %d\n", byte);
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
  async_comm::Serial serial(port, 115200);
  serial.register_receive_callback(&echo);

  if (!serial.init())
  {
    std::printf("Failed to initialize serial port\n");
    return 2;
  }

  uint8_t buffer[NUM_BYTES];

  // test sending bytes one at a time
  std::printf("Transmit individual bytes:\n");
  for (uint8_t i = 0; i < NUM_BYTES; i++)
  {
    buffer[i] = i;
    serial.send_byte(i);
  }

  // wait for all bytes to be received
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // test sending all the bytes at once
  std::printf("Bulk transmit:\n");
  serial.send_bytes(buffer, NUM_BYTES);

  // wait for all bytes to be received
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // close serial port
  serial.close();

  return 0;
}
