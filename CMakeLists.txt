#
# Copyright (C) 2020-2023 LAAS-CNRS, INRIA
#
cmake_minimum_required(VERSION 3.14)

# Project properties
set(PROJECT_NAMESPACE gepetto)
set(PROJECT_NAME quadruped_reactive_walking)
set(PROJECT_DESCRIPTION "Reactive walking control architecture for quadrupeds")
set(PROJECT_URL https://gitlab.laas.fr/${PROJECT_NAMESPACE}/${PROJECT_NAME})
set(CUSTOM_HEADER_DIR "qrw")
set(PROJECT_CUSTOM_HEADER_EXTENSION "hpp")

# Disable -Werror on Unix.
set(CXX_DISABLE_WERROR True)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# JRL-cmakemodule setup
include(cmake/base.cmake)
include(cmake/python.cmake)
include(cmake/boost.cmake)
include(cmake/apple.cmake)
include(CMakeDependentOption)

# Project definition
compute_project_args(PROJECT_ARGS LANGUAGES CXX)
project(${PROJECT_NAME} ${PROJECT_ARGS})
set(CMAKE_VERBOSE_MAKEFILE OFF)
apply_default_apple_configuration()

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'Release' as none was specified.")
  set(CMAKE_BUILD_TYPE
      Release
      CACHE STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel"
                                               "RelWithDebInfo")
endif()

# Project options
option(BUILD_PYTHON_INTERFACE "Build the python binding" ON)
option(BUILD_JOYSTICK "Build with Gamepad support." ON)
cmake_dependent_option(INSTALL_PYTHON_INTERFACE_ONLY "Install *ONLY* the python bindings"
                       OFF BUILD_PYTHON_INTERFACE OFF)
cmake_dependent_option(
  GENERATE_PYTHON_STUBS "Generate the Python stubs associated to the Python library" ON
  BUILD_PYTHON_INTERFACE ON)
option(SUFFIX_SO_VERSION "Suffix library name with its version" ON)
set(INSTALL_DOCUMENTATION OFF)
option(BUILD_WITH_ROS_SUPPORT "Add support for running MPC on a separate rosnode" OFF)

# Project configuration
if(NOT INSTALL_PYTHON_INTERFACE_ONLY)
  set(PROJECT_USE_CMAKE_EXPORT TRUE)
endif()

if(BUILD_WITH_ROS_SUPPORT)
  # ROS macros temper with some variables, save one of them (that affect add_executable for tests)
  if(DEFINED CMAKE_RUNTIME_OUTPUT_DIRECTORY)
    set(REMEMBER_CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
  endif(DEFINED CMAKE_RUNTIME_OUTPUT_DIRECTORY)

  find_package(catkin REQUIRED message_generation std_msgs)
  add_service_files(DIRECTORY ros_qrw_msgs FILES MPCInit.srv MPCSolve.srv MPCStop.srv)
  generate_messages(DEPENDENCIES std_msgs)

  # Restore saved variable to previous state
  if(DEFINED REMEMBER_CMAKE_RUNTIME_OUTPUT_DIRECTORY)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${REMEMBER_CMAKE_RUNTIME_OUTPUT_DIRECTORY})
  else(DEFINED REMEMBER_CMAKE_RUNTIME_OUTPUT_DIRECTORY)
    unset(CMAKE_RUNTIME_OUTPUT_DIRECTORY)
  endif(DEFINED REMEMBER_CMAKE_RUNTIME_OUTPUT_DIRECTORY)
endif()

# Project dependencies
set(BOOST_COMPONENTS unit_test_framework)
add_project_dependency(Eigen3 REQUIRED)
add_project_dependency(example-robot-data REQUIRED)
add_project_dependency(crocoddyl REQUIRED)

if(APPLE OR NOT UNIX)
  set(BUILD_JOYSTICK OFF)
endif()

if(BUILD_JOYSTICK)
  add_compile_definitions(QRW_JOYSTICK_SUPPORT)
endif(BUILD_JOYSTICK)

if(BUILD_PYTHON_INTERFACE)
  add_project_dependency(eigenpy REQUIRED)
  string(REGEX REPLACE "-" "_" PY_NAME ${PROJECT_NAME})
  set(${PY_NAME}_LIB "${PY_NAME}_pywrap")
  set(${PY_NAME}_INSTALL_DIR ${PYTHON_SITELIB}/${PY_NAME})
endif()

# Main Library
set(${PROJECT_NAME}_HEADERS
    include/qrw/Types.hpp
    include/qrw/Params.hpp
    include/qrw/MPCResult.hpp
    include/qrw/yaml-eigen.hpp
    include/qrw/Animator.hpp
    include/qrw/Estimator.hpp
    include/qrw/LowPassFilter.hpp
    include/qrw/ComplementaryFilter.hpp
    include/qrw/IOCPAbstract.hpp
    include/qrw/IMPCWrapper.hpp
    include/qrw/ResidualFlyHigh.hpp
    include/qrw/utils.hpp)

set(${PROJECT_NAME}_SOURCES
    src/Params.cpp src/Animator.cpp src/Estimator.cpp src/LowPassFilter.cpp
    src/ComplementaryFilter.cpp src/IOCPAbstract.cpp)

if(BUILD_JOYSTICK)
  list(APPEND ${PROJECT_NAME}_HEADERS include/qrw/Joystick.hpp)
  list(APPEND ${PROJECT_NAME}_SOURCES src/Joystick.cpp)
endif(BUILD_JOYSTICK)

add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_HEADERS})
target_include_directories(${PROJECT_NAME} PUBLIC $<INSTALL_INTERFACE:include>)
target_include_directories(${PROJECT_NAME} PUBLIC $<INSTALL_INTERFACE:include/qrw>)
target_link_libraries(${PROJECT_NAME} PUBLIC example-robot-data::example-robot-data)

target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)

add_project_dependency(pinocchio REQUIRED)
target_link_libraries(${PROJECT_NAME} PUBLIC pinocchio::pinocchio)
target_link_libraries(${PROJECT_NAME} PUBLIC crocoddyl::crocoddyl)

# Link odri_control_interface library
# find_package(odri_control_interface REQUIRED)
# target_link_libraries(${PROJECT_NAME}
#                       PUBLIC odri_control_interface::odri_control_interface)

# Link master board library
# target_link_libraries(${PROJECT_NAME} PUBLIC master_board_sdk::master_board_sdk)

if(BUILD_TESTING)
  add_subdirectory(tests)
endif()

add_project_dependency(yaml-cpp CONFIG REQUIRED)
if(TARGET yaml-cpp)
  target_link_libraries(${PROJECT_NAME} PUBLIC yaml-cpp)
elseif(TARGET yaml-cpp::yaml-cpp)
  target_link_libraries(${PROJECT_NAME} PUBLIC yaml-cpp::yaml-cpp)
else()
  message(SEND_ERROR "Couldn't link to yaml-cpp")
endif()

if(SUFFIX_SO_VERSION)
  set_target_properties(${PROJECT_NAME} PROPERTIES SOVERSION ${PROJECT_VERSION})
endif()

target_compile_definitions(
  ${PROJECT_NAME}
  PUBLIC CONFIG_SOLO12_YAML="${PROJECT_SOURCE_DIR}/config/config_solo12.yaml")
target_compile_definitions(
  ${PROJECT_NAME}
  PUBLIC WALK_PARAMETERS_YAML="${PROJECT_SOURCE_DIR}/config/walk_parameters.yaml")

# Python Bindings
if(BUILD_PYTHON_INTERFACE)
  add_subdirectory(python)
endif()

# Installation
if(NOT INSTALL_PYTHON_INTERFACE_ONLY)
  install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${TARGETS_EXPORT_NAME}
    DESTINATION lib)
endif()
