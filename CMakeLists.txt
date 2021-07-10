cmake_minimum_required(VERSION 2.8.12)
project(libhuman)
set(library_VERSION 0.1.0)

option(LIBHUMAN_TREAT_WARNINGS_AS_ERRORS "Treat warnings as errors" OFF)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

#================================================================================
# Dependencies
#

include(ExternalProject)

find_package(DART 6.9.0 REQUIRED
  COMPONENTS utils utils-urdf optimizer-nlopt
)

find_package(aikido 0.4.0 REQUIRED
  COMPONENTS
    common
    constraint
    control
    control_ros
    distance
    planner
    planner_ompl
    planner_parabolic
    planner_vectorfield
    planner_kunzretimer
    robot
    rviz
    statespace
    io
)

find_package(Boost REQUIRED)

find_package(roslib REQUIRED)

find_package(srdfdom REQUIRED)

find_package(urdf REQUIRED)

#================================================================================
# Compiler settings
#
add_compile_options(-Wall -Wextra -Wpedantic)
if(LIBHUMAN_TREAT_WARNINGS_AS_ERRORS)
  add_compile_options(-Werror)
endif()

#================================================================================
# Library
#

set(sources
  src/Human.cpp
        include/HumanHand.h include/HumanHandKinematicsSimulationPositionCommandExecutor.h src/HumanHandKinematicsSimulationPositionCommandExecutor.cpp src/HumanHand.cpp)

add_library(libhuman SHARED ${sources})

target_include_directories(libhuman PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(libhuman SYSTEM
  PUBLIC
    ${DART_INCLUDE_DIRS}
    ${aikido_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
  PRIVATE
    ${roslib_INCLUDE_DIRS}
    ${srdfdom_INCLUDE_DIRS}
    ${urdf_INCLUDE_DIRS}
)

set_property(TARGET libhuman PROPERTY VERSION ${library_VERSION})

# Don't name shared library "liblibhuman"
set_property(TARGET libhuman PROPERTY PREFIX "")

target_link_libraries(libhuman
  PUBLIC
    ${DART_LIBRARIES}
    ${aikido_LIBRARIES}
  PRIVATE
    ${roslib_LIBRARIES}
    ${srdfdom_LIBRARIES}
    ${urdf_LIBRARIES}
)

#================================================================================
# Testing
#

# TODO.

#=======================================================================
# Installation
#

install(TARGETS libhuman EXPORT libhumanConfig
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

# This makes the project importable from the install directory
install(EXPORT libhumanConfig DESTINATION share/libhuman/cmake)

# This makes the project importable from the build directory
export(TARGETS libhuman FILE libhumanConfig.cmake)

# Install the package.xml file (to satisfy REP-136).
install(FILES "package.xml"
  DESTINATION "share/libhuman")

install(DIRECTORY resources DESTINATION share)

#===============================================================================
# Basic scripts to play around with.

add_executable(simple_load
  script/simple_load.cpp)

target_link_libraries(simple_load
  ${DART_LIBRARIES}
  ${aikido_LIBRARIES}
  ${Boost_LIBRARIES}
  libhuman)

install(TARGETS simple_load RUNTIME DESTINATION bin)

#===============================================================================
# Basic scripts to play around with.

add_executable(plan_to_tsr
        script/plan_to_tsr.cpp)

target_link_libraries(plan_to_tsr
        ${DART_LIBRARIES}
        ${aikido_LIBRARIES}
        ${Boost_LIBRARIES}
        libhuman)

install(TARGETS plan_to_tsr RUNTIME DESTINATION bin)

#================================================================================
# Formatting
#

find_program(CLANG_FORMAT_EXECUTABLE NAMES clang-format-6.0)

if(CLANG_FORMAT_EXECUTABLE)
  message(STATUS "Found clang-format.")

  file(GLOB_RECURSE ALL_SOURCE_FILES
    LIST_DIRECTORIES false
    include/*.h include/*.hpp src/*.c src/*.cpp)
  list(LENGTH ALL_SOURCE_FILES NUM_SOURCE_FILES)

  add_custom_target(format
    COMMAND ${CMAKE_COMMAND} -E echo "Formatting ${NUM_SOURCE_FILES} files..."
    COMMAND ${CLANG_FORMAT_EXECUTABLE} -style=file -i ${ALL_SOURCE_FILES}
    COMMAND ${CMAKE_COMMAND} -E echo "Done."
    DEPENDS ${CLANG_FORMAT_EXECUTABLE}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

  add_custom_target(check-format
    COMMAND ${CMAKE_COMMAND} -E echo "Checking code style of"
            "${NUM_SOURCE_FILES} files... "
    COMMAND ${CMAKE_SOURCE_DIR}/tools/check_format.sh
            ${CLANG_FORMAT_EXECUTABLE} ${ALL_SOURCE_FILES}
    COMMAND ${CMAKE_COMMAND} -E echo "Done."
    DEPENDS ${CLANG_FORMAT_EXECUTABLE}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
else()
  message(STATUS "Looking for clang-format - NOT found, please install "
      "clang-format to enable automatic code formatting.")
endif()

# This custom target doesn't affect building libhuman but is only for
# displaying the header files in IDEs.
FILE(GLOB_RECURSE libhuman_headers "include/*.hpp")
add_custom_target(headers SOURCES ${libhuman_headers})
