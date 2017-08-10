# - Try to find ORB_SLAM2
# Set alternative paths to search for using ORB_SLAM2_DIR
# Once done this will define
#  ORB_SLAM2_FOUND - System has ORB_SLAM2
#  ORB_SLAM2_INCLUDE_DIRS - The ORB_SLAM2 include directories
#  ORB_SLAM2_LIBRARIES - The libraries needed to use ORB_SLAM2
#  ORB_SLAM2_DEFINITIONS - Compiler switches required for using ORB_SLAM2

# TODO: This need to find dependencies properly, I can't find an example of how to do that
#find_package(OpenCV REQUIRED)
#find_package(Eigen3 REQUIRED)
#find_package(Pangolin REQUIRED)

set(_ORB_SLAM2_SEARCHES /usr/local)
if (ORB_SLAM2_DIR)
    set(_ORB_SLAM2_SEARCHES ${ORB_SLAM2_DIR} ${_ORB_SLAM2_SEARCHES})
endif()
find_path(ORB_SLAM2_INCLUDE_DIR ORB_SLAM2/System.h
          PATHS ${_ORB_SLAM2_SEARCHES} PATH_SUFFIXES include)

find_library(ORB_SLAM2_LIBRARY NAMES ORB_SLAM2 libORB_SLAM2
             PATHS ${_ORB_SLAM2_SEARCHES} PATH_SUFFIXES lib)

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set ORB_SLAM2_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(ORB_SLAM2  DEFAULT_MSG
                                  ORB_SLAM2_LIBRARY ORB_SLAM2_INCLUDE_DIR)

mark_as_advanced(ORB_SLAM2_INCLUDE_DIR ORB_SLAM2_LIBRARY )

set(ORB_SLAM2_LIBRARIES ${ORB_SLAM2_LIBRARY})
set(ORB_SLAM2_INCLUDE_DIRS ${ORB_SLAM2_INCLUDE_DIR})

