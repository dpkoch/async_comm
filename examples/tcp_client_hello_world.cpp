/*
 * Software License Agreement (BSD-3 License)
 *
 * Copyright (c) 2019 Rein Appeldoorn.
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
 * @file tcp_client_hello_world.cpp
 * @author Rein Appeldoorn <reinzor@gmail.com>
 *
 * This example opens a TCP client that sends "hello world" messages.
 */

#include <async_comm/tcp_client.h>

#include <cstdint>
#include <cstring>
#include <iostream>

#include <chrono>
#include <thread>
#include <vector>


/**
 * @brief Callback function for the async_comm library
 *
 * Prints the received bytes to stdout.
 *
 * @param buf Received bytes buffer
 * @param len Number of bytes received
 */
void callback(const uint8_t* buf, size_t len)
{
  for (size_t i = 0; i < len; i++)
  {
    std::cout << buf[i];
  }
}


int main()
{
  // open TCP connection
  async_comm::TCPClient tcp_client("localhost", 16140);
  tcp_client.register_receive_callback(&callback);

  if (!tcp_client.init())
  {
    std::cout << "Failed to initialize TCP client" << std::endl;
    return 1;
  }

  // send message one direction
  for (size_t i = 0; i < 10; ++i)
  {
    std::string msg = "hello world " + std::to_string(i) + "!";
    tcp_client.send_bytes((uint8_t*) msg.data(), msg.size());
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  // close connection
  tcp_client.close();

  return 0;
}
