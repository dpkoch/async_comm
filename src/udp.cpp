#include <comm_library/udp.h>

#include <cstdio>
#include <iostream>

using boost::asio::ip::udp;

namespace comm_library
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
    socket_.set_option(udp::socket::send_buffer_size(MAX_PACKET_LEN*1024));
    socket_.set_option(udp::socket::receive_buffer_size(READ_BUFFER_SIZE*1024));
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

void UDP::do_async_read(const boost::asio::mutable_buffers_1 &buffer, boost::function<void(const boost::system::error_code&, size_t)> handler)
{
  socket_.async_receive_from(buffer, remote_endpoint_, handler);
}

void UDP::do_async_write(const boost::asio::const_buffers_1 &buffer, boost::function<void(const boost::system::error_code&, size_t)> handler)
{
  socket_.async_send_to(buffer, remote_endpoint_, handler);
}

} // namespace comm_library
