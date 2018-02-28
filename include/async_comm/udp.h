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
 * @file udp.h
 * @author Daniel Koch <danielpkoch@gmail.com>
 */

#ifndef ASYNC_COMM_UDP_H
#define ASYNC_COMM_UDP_H

#include <string>

#include <boost/asio.hpp>
#include <boost/function.hpp>

#include <async_comm/comm.h>

namespace async_comm
{

/**
 * @class UDP
 * @brief Asynchronous communication class for a UDP socket
 */
class UDP : public Comm
{
public:
  /**
   * @brief Bind a UPD socket
   * @param bind_host The bind host where this application is listening (usually "localhost")
   * @param bind_port The bind port where this application is listening
   * @param remote_host The remote host to communicate with
   * @param remote_port The port on the remote host
   */
  UDP(std::string bind_host = DEFAULT_BIND_HOST, uint16_t bind_port = DEFAULT_BIND_PORT,
      std::string remote_host = DEFAULT_REMOTE_HOST, uint16_t remote_port = DEFAULT_REMOTE_PORT);
  ~UDP();

private:
  static constexpr auto DEFAULT_BIND_HOST = "localhost";
  static constexpr uint16_t DEFAULT_BIND_PORT = 16140;
  static constexpr auto DEFAULT_REMOTE_HOST = "localhost";
  static constexpr uint16_t DEFAULT_REMOTE_PORT = 16145;

  bool is_open() override;
  bool do_init() override;
  void do_close() override;
  void do_async_read(const boost::asio::mutable_buffers_1 &buffer,
                     boost::function<void(const boost::system::error_code&, size_t)> handler) override;
  void do_async_write(const boost::asio::const_buffers_1 &buffer,
                      boost::function<void(const boost::system::error_code&, size_t)> handler) override;

  std::string bind_host_;
  uint16_t bind_port_;

  std::string remote_host_;
  uint16_t remote_port_;

  boost::asio::ip::udp::socket socket_;
  boost::asio::ip::udp::endpoint bind_endpoint_;
  boost::asio::ip::udp::endpoint remote_endpoint_;
};

} // namespace async_comm

#endif // ASYNC_COMM_UDP_H
