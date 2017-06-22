#include <comm_library/udp.h>

#include <cstdio>
#include <iostream>

using boost::asio::ip::udp;

namespace comm_library
{


UDP::UDP() :
  io_service_(),
  socket_(io_service_),
  write_in_progress_(false)
{
}

UDP::~UDP()
{
  close();
}

bool UDP::init(std::string bind_host, uint16_t bind_port, std::string remote_host, uint16_t remote_port)
{
  try
  {
    // open socket
    udp::resolver resolver(io_service_);

    bind_endpoint_ = *resolver.resolve({udp::v4(), bind_host, ""});
    bind_endpoint_.port(bind_port);

    remote_endpoint_ = *resolver.resolve({udp::v4(), remote_host, ""});
    remote_endpoint_.port(remote_port);

    socket_.open(udp::v4());
    socket_.bind(bind_endpoint_);

    socket_.set_option(udp::socket::reuse_address(true));
    socket_.set_option(udp::socket::send_buffer_size(MAX_PACKET_LEN*1024));
    socket_.set_option(udp::socket::receive_buffer_size(READ_BUFFER_SIZE*1024));
  }
  catch (boost::system::system_error e)
  {
    std::cerr << e.what() << std::endl;
    return false;
  }

  do_async_read();
//  io_service_.post(std::bind(&UDP::do_async_read, this));
//  io_thread_ = std::thread(std::bind(&boost::asio::io_service::run, &this->io_service_));
  io_thread_ = std::thread([this] () { io_service_.run(); });

  return true;
}

void UDP::close()
{
  mutex_lock lock(mutex_);

  io_service_.stop();
  socket_.close();

  if (io_thread_.joinable())
  {
    io_thread_.join();
  }
}

void UDP::send_bytes(const uint8_t *src, size_t len)
{
  assert(len <= MAX_PACKET_LEN);

  WriteBuffer *buffer = new WriteBuffer();
  memcpy(buffer->data, src, len);
  buffer->len = len;

  {
    mutex_lock lock(mutex_);
    write_queue_.push_back(buffer);
  }

  do_async_write(true);
//  io_service_.post(std::bind(&UDP::do_async_write, this, true));
}

void UDP::register_receive_callback(std::function<void (uint8_t)> fun)
{
  receive_callback_ = fun;
}

void UDP::do_async_read()
{
  if (!socket_.is_open()) return;

  socket_.async_receive_from(
        boost::asio::buffer(read_buffer_, READ_BUFFER_SIZE),
        remote_endpoint_,
        [this] (const boost::system::error_code& error, size_t bytes_transferred) {
          async_read_end(error, bytes_transferred);
        });
//        std::bind(
//          &UDP::async_read_end,
//          this,
//          boost::asio::placeholders::error,
//          boost::asio::placeholders::bytes_transferred));
}

void UDP::async_read_end(const boost::system::error_code &error, size_t bytes_transferred)
{
//  if (!socket_.is_open()) return;

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

  do_async_read();
}

void UDP::do_async_write(bool check_write_state)
{
  if (check_write_state && write_in_progress_)
    return;

  mutex_lock lock(mutex_);
  if (write_queue_.empty())
    return;

  write_in_progress_ = true;
  WriteBuffer *buffer = write_queue_.front();
  socket_.async_send_to(
        boost::asio::buffer(buffer->dpos(), buffer->nbytes()),
        remote_endpoint_,
        [this] (const boost::system::error_code& error, size_t bytes_transferred) {
          async_write_end(error, bytes_transferred);
        });

//        std::bind(
//          &UDP::async_write_end,
//          this,
//          boost::asio::placeholders::error,
//          boost::asio::placeholders::bytes_transferred));
}

void UDP::async_write_end(const boost::system::error_code &error, size_t bytes_transferred)
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
    do_async_write(false);
}

} // namespace comm_library
