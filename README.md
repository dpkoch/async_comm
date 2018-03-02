# Async Comm Library

[![Documentation Status](https://codedocs.xyz/dpkoch/async_comm.svg)](https://codedocs.xyz/dpkoch/async_comm/)

This project provides a C++ library that gives a simple interface for asynchronous serial communications over a serial port or UDP.
It uses the [Boost.Asio](http://www.boost.org/doc/libs/master/doc/html/boost_asio.html) library under the hood, but hides from the user the details of interfacing with the ports or sockets and managing send/receive buffers.

## Including in your project

The following instructions are for a project using Git for version control and CMake for a build system, but should serve as a starting point for other setups.

The easiest way to embed the `async_comm` library in your project is as a [Git submodule](https://git-scm.com/docs/gitsubmodules). For example, to put `async_comm` in the `lib/async_comm directory`, run the following from the root of your project:

```bash
git submodule add https://github.com/dpkoch/async_comm.git lib/async_comm
```

Then add the following to your `CMakeLists.txt` to build the `async_comm` library and make the headers accessible in your project:

```CMake
set(CMAKE_CXX_FLAGS "-std=c++11")
add_subdirectory(lib/async_comm)
include_directories(lib/async_comm/include)
```

Then link your executable to the `async_comm` library with something like

```CMake
add_executable(my_program src/my_program.cpp)
target_link_libraries(my_program async_comm)
```

## Usage

There are two classes that you'll use directly as a user:

  - `async_comm::Serial`: for communication over a serial port
  - `async_comm::UDP`: for communication over a UDP socket

Both of these classes have the same interface and inherit from the `async_comm::Comm` base class.
The constructors for each class require the arguments to specify the details of the serial port or UDP socket.

The interface consists of the following functions:

  - `bool init()`: initializes and opens the port or socket
  - `void register_receive_callback(std::function<void(uint8_t)> fun)`: register a user-defined function to handle a received byte; this function will be called by the `Serial` or `UDP` object every time a new byte is received
  - `void send_bytes(const uint8_t * src, size_t len)`: send the specified number of bytes from the specified source buffer
  - `void close()`: close the port or socket

More details can be found in the [code API documentation](https://codedocs.xyz/dpkoch/async_comm/).
Very simple example programs are provided to illustrate the usage as described below.

One tricky part is registering the member function of a class as the receive callback. This is accomplished using `std::bind`. For example, if I want to register the `receive` function of `MyClass` from within the class, I would use

```C++
serial_.register_receive_callback(std::bind(&MyClass::receive, this, std::placeholders::_1));
```

where `serial_` is an instance of `async_comm::Serial`.

## Examples

There are two simple examples provided in the repository. To build the examples, run CMake with the `-DBUILD_EXAMPLES=ON` flag.

  - `examples/serial_loopback.cpp`: Designed for use with a USB-to-UART adapter with the RX and TX pins connected together (loopback). Sends a series of bytes out and prints them to the console as they are received back.
  - `examples/udp_hello_world.cpp`: Opens two UDP objects listening on different ports on the local host, and then uses each to send a simple "hello world" message to the other.
