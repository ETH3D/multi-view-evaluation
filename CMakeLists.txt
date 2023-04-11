project(ETH3DMultiViewEvaluation)
cmake_minimum_required(VERSION 3.14)

include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_set_find_paths()

set(BOOST_ROOT ${CONAN_BOOST_ROOT})
set(Boost_NO_SYSTEM_PATHS        ON)
set(Boost_USE_STATIC_LIBS        OFF)
set(Boost_USE_MULTITHREADED      ON)
set(Boost_USE_STATIC_RUNTIME     OFF)
find_package(Boost COMPONENTS filesystem system program_options chrono thread date_time iostreams REQUIRED QUIET)
if (WIN32)
    target_link_libraries(Boost::system INTERFACE bcrypt)
endif()

# External dependency: Eigen3
find_package(Eigen3 REQUIRED)
find_package(Nanoflann REQUIRED QUIET)
find_package(TinyPly REQUIRED QUIET)

# OpenMP
find_package(OpenMP)
if (OPENMP_FOUND)
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# Settings.
if(NOT CMAKE_BUILD_TYPE)
  message(STATUS "Build type not specified, using RelWithDebInfo")
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

if(NOT "${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
    add_definitions("-O2 -msse2 -msse3 -std=c++17")
endif()

include_directories(
    ./src
)

# Evaluation executable.
add_executable(ETH3DMultiViewEvaluation
    src/accuracy.cc
    src/accuracy.h
    src/completeness.cc
    src/completeness.h
    src/main.cc
    src/meshlab_project.cc
    src/meshlab_project.h
    src/tinyxml2.cpp
    src/tinyxml2.h
    src/util.h
)

target_link_libraries(ETH3DMultiViewEvaluation
    PRIVATE
        Eigen3::Eigen
        Boost::program_options
        Boost::filesystem
        TinyPly::TinyPly
        Nanoflann::Nanoflann
)
