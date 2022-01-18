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
#include <boost/bind/bind.hpp>

namespace async_comm
{

DefaultMessageHandler Comm::default_message_handler_;

Comm::Comm(MessageHandler& message_handler) :
  message_handler_(message_handler),
  io_service_(),
  new_data_(false),
  shutdown_requested_(false),
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

  callback_thread_ = std::thread(std::bind(&Comm::process_callbacks, this));

  async_read();
  io_thread_ = std::thread(boost::bind(&boost::asio::io_service::run, &this->io_service_));

  return true;
}

void Comm::close()
{
  // send shutdown signal to callback thread
  {
    std::unique_lock<std::mutex> lock(callback_mutex_);
    shutdown_requested_ = true;
  }
  condition_variable_.notify_one();

  io_service_.stop();
  do_close();

  if (io_thread_.joinable())
  {
    io_thread_.join();
  }

  if (callback_thread_.joinable())
  {
    callback_thread_.join();
  }
}

void Comm::send_bytes(const uint8_t *src, size_t len)
{
  mutex_lock lock(write_mutex_);

  for (size_t pos = 0; pos < len; pos += WRITE_BUFFER_SIZE)
  {
    size_t num_bytes = (len - pos) > WRITE_BUFFER_SIZE ? WRITE_BUFFER_SIZE : (len - pos);
    write_queue_.emplace_back(src + pos, num_bytes);
  }

  async_write(true);
}

void Comm::register_receive_callback(std::function<void(const uint8_t*, size_t)> fun)
{
  receive_callback_ = fun;
}

void Comm::register_listener(CommListener &listener)
{
  listeners_.push_back(listener);
}

void Comm::async_read()
{
  if (!is_open()) return;

  do_async_read(boost::asio::buffer(read_buffer_, READ_BUFFER_SIZE),
                boost::bind(&Comm::async_read_end,
                            this,
                            boost::asio::placeholders::error,
                            boost::asio::placeholders::bytes_transferred));
}

void Comm::async_read_end(const boost::system::error_code &error, size_t bytes_transferred)
{
  if (error)
  {
    message_handler_.error(error.message());
    close();
    return;
  }

  {
    std::unique_lock<std::mutex> lock(callback_mutex_);
    read_queue_.emplace_back(read_buffer_, bytes_transferred);
    new_data_ = true;
  }
  condition_variable_.notify_one();

  async_read();
}

void Comm::async_write(bool check_write_state)
{
  if (check_write_state && write_in_progress_)
    return;

  mutex_lock lock(write_mutex_);
  if (write_queue_.empty())
    return;

  write_in_progress_ = true;
  WriteBuffer& buffer = write_queue_.front();
  do_async_write(boost::asio::buffer(buffer.dpos(), buffer.nbytes()),
                 boost::bind(&Comm::async_write_end,
                             this,
                             boost::asio::placeholders::error,
                             boost::asio::placeholders::bytes_transferred));
}

void Comm::async_write_end(const boost::system::error_code &error, size_t bytes_transferred)
{
  if (error)
  {
    message_handler_.error(error.message());
    close();
    return;
  }

  mutex_lock lock(write_mutex_);
  if (write_queue_.empty())
  {
    write_in_progress_ = false;
    return;
  }

  WriteBuffer& buffer = write_queue_.front();
  buffer.pos += bytes_transferred;
  if (buffer.nbytes() == 0)
  {
    write_queue_.pop_front();
  }

  if (write_queue_.empty())
    write_in_progress_ = false;
  else
    async_write(false);
}

void Comm::process_callbacks()
{
  std::list<ReadBuffer> local_queue;

  while (true)
  {
    // wait for either new data or a shutdown request
    std::unique_lock<std::mutex> lock(callback_mutex_);
    condition_variable_.wait(lock, [this]{ return new_data_ || shutdown_requested_; });

    // if shutdown requested, end thread execution
    if (shutdown_requested_)
    {
      break;
    }

    // move data to local buffer
    local_queue.splice(local_queue.end(), read_queue_);

    // release mutex to allow continued asynchronous read operations
    new_data_ = false;
    lock.unlock();

    // execute callbacks for all new data
    while (!local_queue.empty())
    {
      ReadBuffer buffer = local_queue.front();
      if (receive_callback_)
      {
        receive_callback_(buffer.data, buffer.len);
      }
      for (std::reference_wrapper<CommListener> listener_ref : listeners_)
      {
        listener_ref.get().receive_callback(buffer.data, buffer.len);
      }
      local_queue.pop_front();
    }
  }
}

} // namespace async_comm
