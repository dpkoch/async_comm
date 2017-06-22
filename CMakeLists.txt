cmake_minimum_required(VERSION 2.8.3)
project(comm_library)

set(CMAKE_CXX_FLAGS "-std=c++11 -O3")

find_package(Boost REQUIRED COMPONENTS system)
find_package(Threads)

include_directories(include)
include_directories(${Boost_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/comm.cpp
  src/udp.cpp
)
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

add_executable(test_udp
  src/test_udp.cpp
)
target_link_libraries(test_udp ${PROJECT_NAME})
