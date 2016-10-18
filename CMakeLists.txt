cmake_minimum_required(VERSION 2.8)
project(ORB_SLAM2_PYTHON_BINDING)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})


#LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

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

#Set the ORBSLAM directory path
set(ORBSLAM2_BASE_DIR "/home/john/Software/ORB_SLAM2")
set(ORBSLAM2_INCLUDE_DIR "${ORBSLAM2_BASE_DIR}/include")
set(ORBSLAM2_LIB "${ORBSLAM2_BASE_DIR}/lib/libORB_SLAM2.so")

# I'm using a custom boost library, because the default one uses python 2 -_-
set(BOOST_ROOT "/home/john/Software/Boost/boost/")
set(Python_ADDITIONAL_VERSIONS 3.4)

find_package(PythonLibs 3.4 REQUIRED)
#find_package(Boost 1.45.0 REQUIRED)
find_package(Boost 1.45.0 REQUIRED COMPONENTS python)
find_package(OpenCV 2.4.3 REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)

include_directories(
${PROJECT_SOURCE_DIR}/src
${ORBSLAM2_INCLUDE_DIR}
${EIGEN3_INCLUDE_DIR}
${Boost_INCLUDE_DIRS}
${PYTHON_INCLUDE_DIRS}
#${Pangolin_INCLUDE_DIRS}
)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

set(TARGET_MODULE_NAME orbslam2)
add_library(${TARGET_MODULE_NAME} SHARED src/ORBSlamPython.cpp)
set_target_properties(${TARGET_MODULE_NAME} PROPERTIES PREFIX "")

MESSAGE(${Boost_INCLUDE_DIRS})

target_link_libraries(${TARGET_MODULE_NAME} 
${ORBSLAM2_LIB}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${Boost_LIBRARIES}
${PYTHON_LIBRARIES}
)

set(${INSTALL_DIR} /home/john/python)
install(TARGETS ${TARGET_MODULE_NAME} DESTINATION /home/john/python)

