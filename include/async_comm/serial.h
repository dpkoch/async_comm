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
