cmake_minimum_required(VERSION 3.5)
#项目名
project(simple)

set(CMAKE_CXX_STANDARD 17)

set(Boost_DEBUG ON)

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_STATIC_RUNTIME OFF)

find_package(Boost REQUIRED COMPONENTS system filesystem thread)

include_directories(${Boost_INCLUDE_DIRS})

add_executable(simple main.cpp)

target_link_libraries(simple ${Boost_LIBRARIES})
