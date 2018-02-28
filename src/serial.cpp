#include <comm_library/serial.h>

#include<iostream>

using boost::asio::serial_port_base;

namespace comm_library
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

} // namespace comm_library
