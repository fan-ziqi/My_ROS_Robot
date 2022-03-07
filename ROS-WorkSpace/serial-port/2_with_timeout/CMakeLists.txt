cmake_minimum_required(VERSION 3.5)
#项目名
project(timeout)

set(CMAKE_CXX_STANDARD 17)

#set(Boost_DEBUG ON)

find_package(Boost REQUIRED COMPONENTS system filesystem thread)

include_directories(${Boost_INCLUDE_DIRS})

add_executable(timeout main.cpp TimeoutSerial.cpp)

target_link_libraries(timeout ${Boost_LIBRARIES})

