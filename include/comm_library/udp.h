#ifndef COMM_LIBRARY_UDP_H
#define COMM_LIBRARY_UDP_H

#include <functional>
#include <list>
#include <mutex>
#include <string>
#include <thread>

#include <boost/asio.hpp>

namespace comm_library
{

class UDP
{
public:
  UDP();
  ~UDP();
  bool init(std::string bind_host = DEFAULT_BIND_HOST, uint16_t bind_port = DEFAULT_BIND_PORT,
       std::string remote_host = DEFAULT_REMOTE_HOST, uint16_t remote_port = DEFAULT_REMOTE_PORT);
  void close();

  void send_bytes(const uint8_t * src, size_t len);
  void register_receive_callback(std::function<void(uint8_t)> fun);

private:
  static constexpr auto DEFAULT_BIND_HOST = "localhost";
  static constexpr uint16_t DEFAULT_BIND_PORT = 16140;
  static constexpr auto DEFAULT_REMOTE_HOST = "localhost";
  static constexpr uint16_t DEFAULT_REMOTE_PORT = 16145;

  static constexpr size_t READ_BUFFER_SIZE = 512;
  static constexpr size_t MAX_PACKET_LEN = 256;

  struct WriteBuffer
  {
    uint8_t data[MAX_PACKET_LEN];
    size_t len;
    size_t pos;

    WriteBuffer() : len(0), pos(0) {}

    WriteBuffer(const uint8_t * buf, uint16_t len) : len(len), pos(0)
    {
      assert(len <= MAX_PACKET_LEN); //! \todo Do something less catastrophic here
      memcpy(data, buf, len);
    }

    uint8_t * dpos() { return data + pos; }

    size_t nbytes() { return len - pos; }
  };

  typedef std::lock_guard<std::recursive_mutex> mutex_lock;

  void do_async_read();
  void async_read_end(const boost::system::error_code& error, size_t bytes_transferred);
  void do_async_write(bool check_write_state);
  void async_write_end(const boost::system::error_code& error, size_t bytes_transferred);

  boost::asio::io_service io_service_;
  std::thread io_thread_;
  std::recursive_mutex mutex_;

  boost::asio::ip::udp::socket socket_;
  boost::asio::ip::udp::endpoint bind_endpoint_;
  boost::asio::ip::udp::endpoint remote_endpoint_;

  uint8_t read_buffer_[READ_BUFFER_SIZE];
  std::list<WriteBuffer*> write_queue_;
  bool write_in_progress_;

  std::function<void(uint8_t)> receive_callback_;
};

} // namespace comm_library

#endif // COMM_LIBRARY_UDP_H
