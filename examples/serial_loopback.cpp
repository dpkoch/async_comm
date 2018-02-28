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
  for (int i = 0; i < NUM_BYTES; i++)
  {
    buffer[i] = i;
    serial.send_bytes((uint8_t*) &i, 1);
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
