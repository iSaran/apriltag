find_path( Apriltag_INCLUDE_DIRS
  NAMES
    apriltag.h
  PATHS
    ${CMAKE_CURRENT_SOURCE_DIR}/../apriltag/
  # NO_CMAKE_PATH
  # NO_CMAKE_ENVIRONMENT_PATH
  # NO_SYSTEM_ENVIRONMENT_PATH
  # NO_CMAKE_SYSTEM_PATH
  NO_DEFAULT_PATH
)

message("!!!!!!!!!!!!!!! Config: CMAKE_CURRENT_SOURCE_DIR: " ${CMAKE_CURRENT_SOURCE_DIR})
message("!!!!!!!!!!!!!!! Config: Apriltag_INCLUDE_DIRS: " ${Apriltag_INCLUDE_DIRS})

find_library( Apriltag_LIBRARY
  LIBRARY_NAMES
    apriltag
  NO_DEFAULT_PATH
  PATHS
    ${CMAKE_CURRENT_SOURCE_DIR}/../apriltag
  # NO_CMAKE_PATH
  # NO_CMAKE_ENVIRONMENT_PATH
  # NO_SYSTEM_ENVIRONMENT_PATH
  # NO_CMAKE_SYSTEM_PATH
)
message("!!!!!!!!!!!!!!! Config: Apriltag_LIBRARY: " ${Apriltag_LIBRARY})

if(Apriltag_INCLUDE_DIRS AND
   Apriltag_LIBRARY)

  set(Apriltag_FOUND true )
  set(Apriltag_LIBRARIES ${Apriltag_LIBRARY})
  set(Apriltag_LIBRARIES ${Apriltag_LIBRARY})
 

endif(Apriltag_INCLUDE_DIRS AND
      Apriltag_LIBRARY)

IF(Apriltag_FOUND)
  MESSAGE(STATUS "Found Apriltag Library: ${Apriltag_LIBRARIES} ===========================================")
ELSE(Apriltag_FOUND)
  IF(Apriltag_FIND_REQUIRED)
    MESSAGE(FATAL_ERROR "Could not find Apriltag library ==============================================")
  ENDIF(Apriltag_FIND_REQUIRED)
ENDIF(Apriltag_FOUND)