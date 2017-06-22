#include <comm_library/udp.h>

#include <cstdint>
#include <cstring>
#include <iostream>

void echo(uint8_t byte)
{
  std::cout << byte;
}

int main()
{
  comm_library::UDP udp1;
  udp1.init("localhost", 14620, "localhost", 14625);
  udp1.register_receive_callback(&echo);

  comm_library::UDP udp2;
  udp2.init("localhost", 14625, "localhost", 14620);

  char message[] = "hello world!";
  udp2.send_bytes((uint8_t*) message, std::strlen(message));

  return 0;
}
