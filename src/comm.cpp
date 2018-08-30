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
 * @file comm.cpp
 * @author Daniel Koch <danielpkoch@gmail.com>
 */

#include <async_comm/comm.h>

#include <iostream>
#include <boost/bind.hpp>

namespace async_comm
{

Comm::Comm() :
  io_service_(),
  write_in_progress_(false)
{
}

Comm::~Comm()
{
}

bool Comm::init()
{
  if (!do_init())
    return false;

  async_read();
  io_thread_ = std::thread(boost::bind(&boost::asio::io_service::run, &this->io_service_));

  return true;
}

void Comm::close()
{
  mutex_lock lock(mutex_);

  io_service_.stop();
  do_close();

  if (io_thread_.joinable())
  {
    io_thread_.join();
  }
}

void Comm::send_bytes(const uint8_t *src, size_t len)
{
  assert(len <= ASYNC_COMM_WRITE_BUFFER_SIZE);

  WriteBuffer *buffer = new WriteBuffer();
  memcpy(buffer->data, src, len);
  buffer->len = len;

  {
    mutex_lock lock(mutex_);
    write_queue_.push_back(buffer);
  }

  async_write(true);
}

void Comm::register_receive_callback(std::function<void(const uint8_t*, size_t)> fun)
{
  receive_callback_ = fun;
}

void Comm::async_read()
{
  if (!is_open()) return;

  do_async_read(boost::asio::buffer(read_buffer_, ASYNC_COMM_READ_BUFFER_SIZE),
                boost::bind(&Comm::async_read_end,
                            this,
                            boost::asio::placeholders::error,
                            boost::asio::placeholders::bytes_transferred));
}

void Comm::async_read_end(const boost::system::error_code &error, size_t bytes_transferred)
{
  if (error)
  {
    std::cerr << error.message() << std::endl;
    close();
    return;
  }

  receive_callback_(read_buffer_, bytes_transferred);

  async_read();
}

void Comm::async_write(bool check_write_state)
{
  if (check_write_state && write_in_progress_)
    return;

  mutex_lock lock(mutex_);
  if (write_queue_.empty())
    return;

  write_in_progress_ = true;
  WriteBuffer *buffer = write_queue_.front();
  do_async_write(boost::asio::buffer(buffer->dpos(), buffer->nbytes()),
                 boost::bind(&Comm::async_write_end,
                             this,
                             boost::asio::placeholders::error,
                             boost::asio::placeholders::bytes_transferred));
}

void Comm::async_write_end(const boost::system::error_code &error, size_t bytes_transferred)
{
  if (error)
  {
    std::cerr << error.message() << std::endl;
    close();
    return;
  }

  mutex_lock lock(mutex_);
  if (write_queue_.empty())
  {
    write_in_progress_ = false;
    return;
  }

  WriteBuffer *buffer = write_queue_.front();
  buffer->pos += bytes_transferred;
  if (buffer->nbytes() == 0)
  {
    write_queue_.pop_front();
    delete buffer;
  }

  if (write_queue_.empty())
    write_in_progress_ = false;
  else
    async_write(false);
}

} // namespace async_comm
