# Async Comm Library

[![Documentation Status](https://codedocs.xyz/dpkoch/async_comm.svg)](https://codedocs.xyz/dpkoch/async_comm/)

This project provides a C++ library that gives a simple interface for asynchronous serial communications over a serial port or UDP.
It uses the [Boost.Asio](http://www.boost.org/doc/libs/master/doc/html/boost_asio.html) library under the hood, but hides from the user the details of interfacing with the ports or sockets and managing send/receive buffers.

## Usage

There are two classes that you'll use directly as a user:

  - `Serial`: for communication over a serial port
  - `UDP`: for communication over a UDP socket

Both of these classes have the same interface and inherit from the `Comm` base class.
The constructors for each class require the arguments to specify the details of the serial port or UDP socket.

The interface consists of the following functions:

  - `bool init()`: initializes and opens the port or socket
  - `void register_receive_callback(std::function<void(uint8_t)> fun)`: register a user-defined function to handle a received byte
  - `void send_bytes(const uint8_t * src, size_t len)`: send the specified number of bytes from the specified source buffer
  - `void close()`: close the port or socket

## Examples

There are two simple examples provided in the repository:

  - `examples/serial_loopback.cpp`: Designed for use with a USB-to-UART adapter with the RX and TX pins connected together (loopback). Sends a series of bytes out and prints them to the console as they are received back.
  - `examples/udp_hello_world.cpp`: Opens two UDP objects listening on different ports on the local host, and then uses each to send a simple "hello world" message to the other.
