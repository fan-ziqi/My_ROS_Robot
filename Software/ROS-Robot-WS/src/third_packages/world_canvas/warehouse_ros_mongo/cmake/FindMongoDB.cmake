# - Find MongoDB; NOTE: this is specific to warehouse_ros!
#
# Find the MongoDB includes and client library
# This module defines
#  MongoDB_INCLUDE_DIR, where to find mongo/client/dbclient.h
#  MongoDB_LIBRARIES, the libraries needed to use MongoDB.
#  MongoDB_FOUND, If false, do not try to use MongoDB.
#  MongoDB_EXPOSE_MACROS, If true, warehouse_ros_mongo should use '#define MONGO_EXPOSE_MACROS'

set(MongoDB_EXPOSE_MACROS "NO")

set(MongoDB_PossibleIncludePaths
  /usr/include/
  /usr/local/include/
  /usr/include/mongo/
  /usr/local/include/mongo/
  /opt/mongo/include/
  $ENV{ProgramFiles}/Mongo/*/include
  $ENV{SystemDrive}/Mongo/*/include
  )
find_path(MongoDB_INCLUDE_DIR mongo/client/dbclient.h
  ${MongoDB_PossibleIncludePaths})

if(MongoDB_INCLUDE_DIR)
  find_path(MongoDB_dbclientinterface_Path mongo/client/dbclientinterface.h
    ${MongoDB_PossibleIncludePaths})
  if (MongoDB_dbclientinterface_Path)
    set(MongoDB_EXPOSE_MACROS "YES")
  endif()
endif()

if(WIN32)
  find_library(MongoDB_LIBRARIES NAMES mongoclient
    PATHS
    $ENV{ProgramFiles}/Mongo/*/lib
    $ENV{SystemDrive}/Mongo/*/lib
    )
else(WIN32)
  find_library(MongoDB_LIBRARIES NAMES mongoclient
    PATHS
    /usr/lib
    /usr/lib64
    /usr/lib/mongo
    /usr/lib64/mongo
    /usr/local/lib
    /usr/local/lib64
    /usr/local/lib/mongo
    /usr/local/lib64/mongo
    /opt/mongo/lib
    /opt/mongo/lib64
    )
endif(WIN32)

if(MongoDB_INCLUDE_DIR AND MongoDB_LIBRARIES)
  set(MongoDB_FOUND TRUE)
  message(STATUS "Found MongoDB: ${MongoDB_INCLUDE_DIR}, ${MongoDB_LIBRARIES}")
  message(STATUS "MongoDB using new interface: ${MongoDB_EXPOSE_MACROS}")
else(MongoDB_INCLUDE_DIR AND MongoDB_LIBRARIES)
  set(MongoDB_FOUND FALSE)
  if (MongoDB_FIND_REQUIRED)
    message(FATAL_ERROR "MongoDB not found.")
  else (MongoDB_FIND_REQUIRED)
    message(STATUS "MongoDB not found.")
  endif (MongoDB_FIND_REQUIRED)
endif(MongoDB_INCLUDE_DIR AND MongoDB_LIBRARIES)

mark_as_advanced(MongoDB_INCLUDE_DIR MongoDB_LIBRARIES MongoDB_EXPOSE_MACROS)
