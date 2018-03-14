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
 * @file udp.cpp
 * @author Daniel Koch <danielpkoch@gmail.com>
 */

#include <async_comm/udp.h>

#include <iostream>

using boost::asio::ip::udp;

namespace async_comm
{


UDP::UDP(std::string bind_host, uint16_t bind_port, std::string remote_host, uint16_t remote_port) :
  Comm(),
  bind_host_(bind_host),
  bind_port_(bind_port),
  remote_host_(remote_host),
  remote_port_(remote_port),
  socket_(io_service_)
{
}

UDP::~UDP()
{
  do_close();
}

bool UDP::is_open()
{
  return socket_.is_open();
}

bool UDP::do_init()
{
  try
  {
    udp::resolver resolver(io_service_);

    bind_endpoint_ = *resolver.resolve({udp::v4(), bind_host_, ""});
    bind_endpoint_.port(bind_port_);

    remote_endpoint_ = *resolver.resolve({udp::v4(), remote_host_, ""});
    remote_endpoint_.port(remote_port_);

    socket_.open(udp::v4());
    socket_.bind(bind_endpoint_);

    socket_.set_option(udp::socket::reuse_address(true));
    socket_.set_option(udp::socket::send_buffer_size(ASYNC_COMM_WRITE_BUFFER_SIZE*1024));
    socket_.set_option(udp::socket::receive_buffer_size(ASYNC_COMM_READ_BUFFER_SIZE*1024));
  }
  catch (boost::system::system_error e)
  {
    std::cerr << e.what() << std::endl;
    return false;
  }

  return true;
}

void UDP::do_close()
{
  socket_.close();
}

void UDP::do_async_read(const boost::asio::mutable_buffers_1 &buffer,
                        boost::function<void(const boost::system::error_code&, size_t)> handler)
{
  socket_.async_receive_from(buffer, remote_endpoint_, handler);
}

void UDP::do_async_write(const boost::asio::const_buffers_1 &buffer,
                         boost::function<void(const boost::system::error_code&, size_t)> handler)
{
  socket_.async_send_to(buffer, remote_endpoint_, handler);
}

} // namespace async_comm
