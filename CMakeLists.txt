cmake_minimum_required(VERSION 2.8)
project(ORB_SLAM2_PYTHON_BINDING)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(PythonLibs 3.5 REQUIRED)
find_package(NumPy)
find_package(Boost 1.45.0 REQUIRED COMPONENTS python-py35)
find_package(OpenCV REQUIRED)
find_package(ORB_SLAM2 REQUIRED)
find_package(Pangolin)

# RPATH fiddling for the library, so that the installed version can find the linked libraries
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

include_directories(
${PROJECT_SOURCE_DIR}/src
${ORB_SLAM2_INCLUDE_DIR}
${Boost_INCLUDE_DIRS}
${PYTHON_INCLUDE_DIRS}
${PYTHON_NUMPY_INCLUDE_DIR}
)
if (Pangolin_FOUND)
  include_directories(${Pangolin_INCLUDE_DIRS})
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

set(TARGET_MODULE_NAME orbslam2)
add_library(${TARGET_MODULE_NAME} SHARED
src/ORBSlamPython.cpp
src/pyboost_cv2_converter.cpp
src/pyboost_cv3_converter.cpp)
set_target_properties(${TARGET_MODULE_NAME} PROPERTIES PREFIX "")

target_link_libraries(${TARGET_MODULE_NAME} 
${ORB_SLAM2_LIBRARIES}
${OpenCV_LIBS}
${Boost_LIBRARIES}
${PYTHON_LIBRARIES}
)
if (Pangolin_FOUND)
  target_link_libraries(${TARGET_MODULE_NAME} ${Pangolin_LIBRARIES})
endif()

install(TARGETS ${TARGET_MODULE_NAME} DESTINATION lib/python3.5/dist-packages)

