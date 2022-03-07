cmake_minimum_required(VERSION 3.5)

project(serial_test)

set(CMAKE_CXX_STANDARD 17)

find_package(Boost REQUIRED COMPONENTS system filesystem thread)

include_directories(${Boost_INCLUDE_DIRS})

add_executable(serial_test src/serial_test.cpp src/serial_linux.cpp)

target_link_libraries(serial_test ${Boost_LIBRARIES})