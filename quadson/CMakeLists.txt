cmake_minimum_required(VERSION 3.0.0)
project(Quadson VERSION 0.1.0)

include(CTest)
enable_testing()

find_path(EIGEN3_INCLUDE_DIRS "Eigen" paths "/usr/include" "/usr/local/include" path_suffixes "eigen3")
message(${EIGEN3_INCLUDE_DIRS})
if ( NOT EIGEN3_INCLUDE_DIRS )
    message(FATAL_ERROR "CMake variable EIGEN3_INCLUDE_DIRS not found.")
endif()
include_directories ( "${EIGEN3_INCLUDE_DIRS}" )

add_subdirectory(source)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_C_FLAGS_DEBUG "-g -DDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "-g -DDEBUG")
include(CPack)
