

cmake_minimum_required(VERSION 2.6)

set(DICEPHYS_VERSION_MAJOR 0)
set(DICEPHYS_VERSION_MINOR 3)
set(DICEPHYS_VERSION_PATCH 0)
set(DICEPHYS_VERSION "${DICEPHYS_VERSION_MAJOR}.${DICEPHYS_VERSION_MINOR}.${DICEPHYS_VERSION_PATCH}")

message(STATUS "Configuring DicePhys ${DICEPHYS_VERSION}")

project(DicePhys)


## FIND PACKAGES ########################################
# First add any user specified paths to the package search array
set(CMAKE_PREFIX_PATH "${BOOST_ROOT}" "${ARMADILLO_ROOT}")


# Boost #
# Tell Windows builds to look for static boost libraries
if(WIN32)
  set(Boost_USE_STATIC_LIBS ON)
endif(WIN32)
find_package(Boost 1.36.0 COMPONENTS program_options REQUIRED)

add_subdirectory(lib/bullet) # Bullet physics engine

set(DICEPHYS_INCLUDE_DIRS
  ${PROJECT_SOURCE_DIR}/src
)

include_directories(
  ${DICEPHYS_INCLUDE_DIRS}
)

add_subdirectory(src ${CMAKE_BINARY_DIR}/bin)

set_property(TARGET PROPERTY PROJECT_LABEL "DicePhys")