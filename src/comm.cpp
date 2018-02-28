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
  assert(len <= MAX_PACKET_LEN);

  WriteBuffer *buffer = new WriteBuffer();
  memcpy(buffer->data, src, len);
  buffer->len = len;

  {
    mutex_lock lock(mutex_);
    write_queue_.push_back(buffer);
  }

  async_write(true);
}

void Comm::register_receive_callback(std::function<void (uint8_t)> fun)
{
  receive_callback_ = fun;
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
    std::cerr << error.message() << std::endl;
    close();
    return;
  }

  for (int i = 0; i < bytes_transferred; i++)
  {
    receive_callback_(read_buffer_[i]);
  }

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
