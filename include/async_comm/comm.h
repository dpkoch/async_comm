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
 * @file comm.h
 * @author Daniel Koch <danielpkoch@gmail.com>
 */

#ifndef ASYNC_COMM_COMM_H
#define ASYNC_COMM_COMM_H

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <list>
#include <mutex>
#include <thread>

#include <boost/asio.hpp>
#include <boost/function.hpp>

#include <async_comm/message_handler.h>

namespace async_comm
{

// extern DefaultMessageHandler default_message_handler_;

/**
 * @class CommListener
 * @brief Abstract base class for getting comm events via a listener interface
 */
class CommListener
{
public:
  /**
   * @brief Callback for data received
   *
   * @warning The data buffer passed to the callback function will be invalid after the callback function exits. If you
   * want to store the data for later processing, you must copy the data to a new buffer rather than storing the
   * pointer to the buffer.
   *
   * @param buf Address of buffer containing received bytes
   * @param size Number of bytes in the receive buffer
   */
  virtual void receive_callback(const uint8_t * buf, size_t size) = 0;
};

/**
 * @class Comm
 * @brief Abstract base class for an asynchronous communication port
 */
class Comm
{
public:
  /**
   * @brief Set up asynchronous communication base class
   * @param message_handler Custom message handler, or omit for default handler
   */
  Comm(MessageHandler& message_handler = *(new DefaultMessageHandler));
  virtual ~Comm();

  /**
   * @brief Initializes and opens the port
   * @return True if the port was succesfully initialized
   */
  bool init();

  /**
   * @brief Closes the port
   */
  void close();

  /**
   * @brief Send bytes from a buffer over the port
   * @param src Address of the buffer
   * @param len Number of bytes to send
   */
  void send_bytes(const uint8_t * src, size_t len);

  /**
   * @brief Send a single byte over the port
   * @param data Byte to send
   */
  inline void send_byte(uint8_t data) { send_bytes(&data, 1); }

  /**
   * @brief Register a callback function for when bytes are received on the port
   *
   * The callback function needs to accept two parameters. The first is of type `const uint8_t*`, and is a constant
   * pointer to the data buffer. The second is of type `size_t`, and specifies the number of bytes available in the
   * buffer.
   *
   * @warning The data buffer passed to the callback function will be invalid after the callback function exits. If you
   * want to store the data for later processing, you must copy the data to a new buffer rather than storing the
   * pointer to the buffer.
   *
   * @param fun Function to call when bytes are received
   */
  void register_receive_callback(std::function<void(const uint8_t*, size_t)> fun);

  /**
   * @brief Register a listener for when bytes are received on the port
   *
   * The listener must inherit from CommListener and implement the `receive_callback` function.  This is another
   * mechanism for receiving data from the Comm interface without needing to create function pointers. Multiple
   * listeners can be added and all will get the callback
   *
   * @param listener Reference to listener
   */
  void register_listener(CommListener &listener);

protected:

  static constexpr size_t READ_BUFFER_SIZE = 1024;
  static constexpr size_t WRITE_BUFFER_SIZE = 1024;

  virtual bool is_open() = 0;
  virtual bool do_init() = 0;
  virtual void do_close() = 0;
  virtual void do_async_read(const boost::asio::mutable_buffers_1 &buffer,
                             boost::function<void(const boost::system::error_code&, size_t)> handler) = 0;
  virtual void do_async_write(const boost::asio::const_buffers_1 &buffer,
                              boost::function<void(const boost::system::error_code&, size_t)> handler) = 0;

  // DefaultMessageHandler default_message_handler_;
  MessageHandler& message_handler_;
  boost::asio::io_service io_service_;

private:

  struct ReadBuffer
  {
    uint8_t data[READ_BUFFER_SIZE];
    size_t len;

    ReadBuffer(const uint8_t * buf, size_t len) : len(len)
    {
      assert(len <= READ_BUFFER_SIZE); // only checks in debug mode
      memcpy(data, buf, len);
    }
  };

  struct WriteBuffer
  {
    uint8_t data[WRITE_BUFFER_SIZE];
    size_t len;
    size_t pos;

    WriteBuffer() : len(0), pos(0) {}

    WriteBuffer(const uint8_t * buf, size_t len) : len(len), pos(0)
    {
      assert(len <= WRITE_BUFFER_SIZE); // only checks in debug mode
      memcpy(data, buf, len);
    }

    const uint8_t * dpos() const { return data + pos; }

    size_t nbytes() const { return len - pos; }
  };

  typedef std::lock_guard<std::recursive_mutex> mutex_lock;

  void async_read();
  void async_read_end(const boost::system::error_code& error, size_t bytes_transferred);

  void async_write(bool check_write_state);
  void async_write_end(const boost::system::error_code& error, size_t bytes_transferred);

  void process_callbacks();

  std::thread io_thread_;
  std::thread callback_thread_;

  uint8_t read_buffer_[READ_BUFFER_SIZE];
  std::list<ReadBuffer> read_queue_;
  std::mutex callback_mutex_;
  std::condition_variable condition_variable_;
  bool new_data_;
  bool shutdown_requested_;

  std::list<WriteBuffer> write_queue_;
  std::recursive_mutex write_mutex_;
  bool write_in_progress_;

  std::function<void(const uint8_t *, size_t)> receive_callback_ = nullptr;
  std::vector<std::reference_wrapper<CommListener>> listeners_;
};

} // namespace async_comm

#endif // ASYNC_COMM_COMM_H
