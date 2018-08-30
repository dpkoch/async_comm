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
 * @file serial.h
 * @author Daniel Koch <danielpkoch@gmail.com>
 */

#ifndef ASYNC_COMM_SERIAL_H
#define ASYNC_COMM_SERIAL_H

#include <string>

#include <boost/asio.hpp>
#include <boost/function.hpp>

#include <async_comm/comm.h>

namespace async_comm
{

/**
 * @class Serial
 * @brief Asynchronous communication class for a serial port
 */
class Serial : public Comm
{
public:
  /**
   * @brief Open a serial port
   * @param port The port to open (e.g. "/dev/ttyUSB0")
   * @param baud_rate The baud rate for the serial port (e.g. 115200)
   */
  Serial(std::string port, unsigned int baud_rate);
  ~Serial();


  /**
   * @brief Set serial port baud rate
   * @param baud_rate The baud rate for the serial port (e.g. 115200)
   * @return True if successful
   */
  bool set_baud_rate(unsigned int baud_rate);

private:
  bool is_open() override;
  bool do_init() override;
  void do_close() override;
  void do_async_read(const boost::asio::mutable_buffers_1 &buffer,
                     boost::function<void(const boost::system::error_code&, size_t)> handler) override;
  void do_async_write(const boost::asio::const_buffers_1 &buffer,
                      boost::function<void(const boost::system::error_code&, size_t)> handler) override;

  std::string port_;
  unsigned int baud_rate_;

  boost::asio::serial_port serial_port_;
};

} // namespace async_comm

#endif // ASYNC_COMM_SERIAL_H
