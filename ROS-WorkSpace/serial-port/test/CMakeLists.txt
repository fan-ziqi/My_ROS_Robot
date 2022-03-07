cmake_minimum_required(VERSION 3.5)

project(test)

set(CMAKE_CXX_STANDARD 17)

find_package(Boost REQUIRED COMPONENTS system filesystem thread)
include_directories(${Boost_INCLUDE_DIRS})

find_package(glog)
include_directories(${GLOG_INCLUDE_DIRS})


add_executable(test src/serial_test.cpp src/serial_linux.cpp)

target_link_libraries(test ${Boost_LIBRARIES} glog::glog)