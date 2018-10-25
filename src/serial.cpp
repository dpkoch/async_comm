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
 * @file serial.cpp
 * @author Daniel Koch <danielpkoch@gmail.com>
 */

#include <async_comm/serial.h>

#include<iostream>

using boost::asio::serial_port_base;

namespace async_comm
{

Serial::Serial(std::string port, unsigned int baud_rate) :
  Comm(),
  port_(port),
  baud_rate_(baud_rate),
  serial_port_(io_service_)
{
}

Serial::~Serial()
{
  do_close();
}

bool Serial::set_baud_rate(unsigned int baud_rate)
{
  baud_rate_ = baud_rate;
  try
  {
    serial_port_.set_option(serial_port_base::baud_rate(baud_rate_));
    serial_port_.open(port_);
  }
  catch (boost::system::system_error e)
  {
    std::cerr << e.what() << std::endl;
    return false;
  }

  return true;
}

bool Serial::is_open()
{
  return serial_port_.is_open();
}

bool Serial::do_init()
{
  try
  {
    serial_port_.open(port_);
    serial_port_.set_option(serial_port_base::baud_rate(baud_rate_));
    serial_port_.set_option(serial_port_base::character_size(8));
    serial_port_.set_option(serial_port_base::parity(serial_port_base::parity::none));
    serial_port_.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
    serial_port_.set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));
  }
  catch (boost::system::system_error e)
  {
    std::cerr << e.what() << std::endl;
    return false;
  }

  return true;
}

void Serial::do_close()
{
  serial_port_.close();
}

void Serial::do_async_read(const boost::asio::mutable_buffers_1 &buffer,
                           boost::function<void (const boost::system::error_code&, size_t)> handler)
{
  serial_port_.async_read_some(buffer, handler);
}

void Serial::do_async_write(const boost::asio::const_buffers_1 &buffer,
                            boost::function<void (const boost::system::error_code&, size_t)> handler)
{
  serial_port_.async_write_some(buffer, handler);
}

} // namespace async_comm
