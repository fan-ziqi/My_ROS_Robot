cmake_minimum_required(VERSION 3.5)
#项目名
set(project_name async)

project(${project_name})

set(CMAKE_CXX_STANDARD 17)

#set(Boost_DEBUG ON)

find_package(Boost REQUIRED COMPONENTS system filesystem thread)

add_executable(${project_name} main.cpp AsyncSerial.cpp BufferedAsyncSerial.cpp)

target_link_libraries(${project_name} ${Boost_LIBRARIES})