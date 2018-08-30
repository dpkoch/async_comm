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

add_library(${PROJECT_NAME} SHARED
  src/comm.cpp
  src/udp.cpp
  src/serial.cpp
)
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

# examples
option(ASYNC_COMM_BUILD_EXAMPLES "Build examples" OFF)

if(ASYNC_COMM_BUILD_EXAMPLES)
  add_executable(udp_hello_world examples/udp_hello_world.cpp)
  target_link_libraries(udp_hello_world ${PROJECT_NAME})

  add_executable(serial_loopback examples/serial_loopback.cpp)
  target_link_libraries(serial_loopback ${PROJECT_NAME})

  add_executable(serial_protocol examples/serial_protocol.cpp)
  target_link_libraries(serial_protocol ${PROJECT_NAME})
endif(ASYNC_COMM_BUILD_EXAMPLES)

# install
set(LIB_DEST lib/${PROJECT_NAME})
set(INCLUDE_DEST include)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}-targets
  ARCHIVE DESTINATION ${LIB_DEST}
  LIBRARY DESTINATION ${LIB_DEST}
)
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${INCLUDE_DEST}
  FILES_MATCHING PATTERN "*.h"
)

# install CMake package configuration
install(EXPORT ${PROJECT_NAME}-targets DESTINATION ${LIB_DEST})
install(FILES ${PROJECT_NAME}-config.cmake DESTINATION ${LIB_DEST})

# install package.xml for ROS release
install(FILES package.xml DESTINATION share/${PROJECT_NAME})

