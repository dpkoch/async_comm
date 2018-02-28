cmake_minimum_required(VERSION 2.8.3)
project(async_comm)

set(CMAKE_CXX_FLAGS "-std=c++11")

if (NOT DEFINED CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif (NOT DEFINED CMAKE_BUILD_TYPE)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Threads)

include_directories(include)
include_directories(${Boost_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/comm.cpp
  src/udp.cpp
  src/serial.cpp
)
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

add_executable(udp_hello_world examples/udp_hello_world.cpp)
target_link_libraries(udp_hello_world ${PROJECT_NAME})

add_executable(serial_loopback examples/serial_loopback.cpp)
target_link_libraries(serial_loopback ${PROJECT_NAME})
