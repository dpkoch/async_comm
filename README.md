# Async Comm Library

[![Build Status](http://build.ros.org/buildStatus/icon?job=Mdev__async_comm__ubuntu_bionic_amd64)](http://build.ros.org/job/Mdev__async_comm__ubuntu_bionic_amd64)
[![Documentation Status](https://codedocs.xyz/dpkoch/async_comm.svg)](https://codedocs.xyz/dpkoch/async_comm/)

This project provides a C++ library that gives a simple interface for asynchronous serial communications over a serial port or UDP.
It uses the [Boost.Asio](http://www.boost.org/doc/libs/master/doc/html/boost_asio.html) library under the hood, but hides from the user the details of interfacing with the ports or sockets and managing send/receive buffers.

## Including in your project

There are three ways to use the `async_comm` library in your project:

  1. If you'll be using the library in a ROS package, install from the ROS repositories
  2. Build and install the library on your system, then use CMake's `find_package()` functionality
  3. Include the async_comm as a submodule in your project

With the second and third options, you will need to ensure that the Boost library is installed before proceeding:

```bash
sudo apt -y install libboost-dev
```

### ROS install

The `async_comm` library is released as a third-party, non-catkin package for ROS following the guidelines in [REP 136](http://www.ros.org/reps/rep-0136.html). To use the library in your ROS package, first install the library from the ROS repositories:

```bash
sudo apt install ros-<DISTRO>-async-comm
```

Replace `<DISTRO>` with your ROS distribution. The library is currently released for kinetic, lunar, and melodic.

Then, add something like the following lines to your package's CMakeLists.txt:

```CMake
set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(async_comm REQUIRED)

catkin_package(
  ...
  DEPENDS async_comm
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ...
  ${async_comm_INCLUDE_DIRS}
)

add_executable(my_node src/my_node.cpp)
target_link_libraries(my_node ${async_comm_LIBRARIES})
```

Also be sure to list `async_comm` as a dependency in your package.xml:

```XML
<?xml version="1.0"?>
<package format="2">
  ...
  <depend>async_comm</depend>
  ...
</package>
```

### System install

First, download and install the library:

```bash
git clone https://github.com/dpkoch/async_comm.git
cd async_comm
mkdir build && cd build/
cmake .. && make
sudo make install
```

Then, do something like this in your project's CMakeLists.txt:

```CMake
cmake_minimum_required(VERSION 2.8.11)
project(my_project)

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(async_comm REQUIRED)
include_directories(${async_comm_INCLUDE_DIRS})

add_executable(my_project src/my_project.cpp)
target_link_libraries(my_project ${async_comm_LIBRARIES})
```

### Including as a submodule

If you don't want to go with the ROS or system install options, the next easiest way to embed the `async_comm` library in your project is as a [Git submodule](https://git-scm.com/docs/gitsubmodules). The following instructions are for a project using Git for version control and CMake for a build system, but should serve as a starting point for other setups.

For example, to put `async_comm` in the `lib/async_comm directory`, run the following from the root of your project:

```bash
git submodule add https://github.com/dpkoch/async_comm.git lib/async_comm
```

Your CMakeLists.txt file would then look something like this:

```CMake
cmake_minimum_required(VERSION 2.8.11)
project(my_project)

set(CMAKE_CXX_FLAGS "-std=c++11")

add_subdirectory(lib/async_comm)
include_directories(lib/async_comm/include)

add_executable(my_project src/my_project.cpp)
target_link_libraries(my_project async_comm)
```

## Usage

There are two classes that you'll use directly as a user:

  - `async_comm::Serial`: for communication over a serial port
  - `async_comm::UDP`: for communication over a UDP socket

Both of these classes have the same interface and inherit from the `async_comm::Comm` base class.
The constructors for each class require the arguments to specify the details of the serial port or UDP socket.

The interface consists of the following functions:

  - `bool init()`: initializes and opens the port or socket
  - `void register_receive_callback(std::function<void(const uint8_t*, size_t)> fun)`: register a user-defined function to handle received bytes; this function will be called by the `Serial` or `UDP` object every time a new data is received
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

There are three examples provided in the repository. The first two are very simple, while the third is more complete. To build the examples, run CMake with the `-DASYNC_COMM_BUILD_EXAMPLES=ON` flag.

  - `examples/serial_loopback.cpp`: Designed for use with a USB-to-UART adapter with the RX and TX pins connected together (loopback). Sends a series of bytes out and prints them to the console as they are received back.
  - `examples/udp_hello_world.cpp`: Opens two UDP objects listening on different ports on the local host, and then uses each to send a simple "hello world" message to the other.
  - `examples/serial_protocol.cpp`: Implements a simple serial protocol and parser for a message that includes two integer values, including a cyclic reduncancy check. Tests the protocol and `async_comm` library over a serial loopback.
