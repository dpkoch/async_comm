#include <comm_library/udp.h>

#include <cstdint>
#include <cstring>
#include <iostream>

#include <chrono>
#include <thread>

void echo(uint8_t byte)
{
  std::cout << byte;
}

int main()
{
  // open UDP ports
  comm_library::UDP udp1("localhost", 14620, "localhost", 14625);
  udp1.register_receive_callback(&echo);

  comm_library::UDP udp2("localhost", 14625, "localhost", 14620);
  udp2.register_receive_callback(&echo);

  if (!udp1.init() || !udp2.init())
  {
    std::cout << "Failed to initialize UDP ports" << std::endl;
    return 1;
  }

  // send message one direction
  char message1[] = "hello world 1!";
  udp2.send_bytes((uint8_t*) message1, std::strlen(message1));

  // wait for all bytes to be received
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  std::cout << std::endl << std::flush;

  // send message the other direction
  char message2[] = "hello world 2!";
  udp1.send_bytes((uint8_t*) message2, std::strlen(message2));

  // wait for all bytes to be received
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  std::cout << std::endl << std::flush;

  std::cout.flush();

  // close UDP ports
  udp1.close();
  udp2.close();

  return 0;
}
