cmake_minimum_required(VERSION 3.10.0)

project(GOMA CXX)

if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

################################################################################
# Options
################################################################################
list(APPEND CMAKE_CXX_FLAGS "-fmax-errors=10 -pthread -std=c++11 ${CMAKE_CXX_FLAGS} -g -ggdb -Wall")
add_definitions(-g -ggdb -Wall -O3 -pthread -std=c++11 )

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

################################################################################
# Find packages
################################################################################

find_package(PkgConfig)

find_package(Ceres REQUIRED)

find_package(CGAL QUIET COMPONENTS)

find_package(COLMAP REQUIRED)

find_path(EIGEN_INCLUDE_DIR Eigen/Core
	/usr/local/include/eigen3
	/usr/local/include/eigen2
	/usr/local/include/eigen
	/usr/include/eigen3
	/usr/include/eigen2
	/usr/include/eigen
	/opt/local/include/eigen3
)

################################################################################
# Add sources
################################################################################
include_directories(
  ${EIGEN_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS}
  ${COLMAP_INCLUDE_DIRS}
)

link_directories(
  ${COLMAP_LINK_DIRS}
)

add_subdirectory(goma)
add_subdirectory(examples)
