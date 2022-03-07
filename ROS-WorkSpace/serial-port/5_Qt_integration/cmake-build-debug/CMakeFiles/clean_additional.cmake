# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "Release")
  file(REMOVE_RECURSE
  "CMakeFiles/SerialGUI_autogen.dir/AutogenUsed.txt"
  "CMakeFiles/SerialGUI_autogen.dir/ParseCache.txt"
  "SerialGUI_autogen"
  )
endif()
