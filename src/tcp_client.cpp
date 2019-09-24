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
 * @file tcp_client.cpp
 * @author Rein Appeldoorn <reinzor@gmail.com>
 */

#include <async_comm/tcp_client.h>

#include <iostream>

using boost::asio::ip::tcp;

namespace async_comm
{

TCPClient::TCPClient(std::string host, uint16_t port, MessageHandler& message_handler) :
  Comm(message_handler),
  host_(host),
  port_(port),
  socket_(io_service_)
{
}

TCPClient::~TCPClient()
{
  do_close();
}

bool TCPClient::is_open()
{
  return socket_.is_open();
}

bool TCPClient::do_init()
{
  try
  {
    tcp::resolver resolver(io_service_);

    endpoint_ = *resolver.resolve({tcp::v4(), host_, ""});
    endpoint_.port(port_);
    socket_.open(tcp::v4());

    socket_.connect(endpoint_);

    socket_.set_option(tcp::socket::reuse_address(true));
    socket_.set_option(tcp::socket::send_buffer_size(WRITE_BUFFER_SIZE*1024));
    socket_.set_option(tcp::socket::receive_buffer_size(READ_BUFFER_SIZE*1024));
  }
  catch (boost::system::system_error e)
  {
    message_handler_.error(e.what());
    return false;
  }

  return true;
}

void TCPClient::do_close()
{
  socket_.close();
}

void TCPClient::do_async_read(const boost::asio::mutable_buffers_1 &buffer,
                        boost::function<void(const boost::system::error_code&, size_t)> handler)
{
  socket_.async_receive(buffer, handler);
}

void TCPClient::do_async_write(const boost::asio::const_buffers_1 &buffer,
                         boost::function<void(const boost::system::error_code&, size_t)> handler)
{
  socket_.async_send(buffer, handler);
}

} // namespace async_comm
