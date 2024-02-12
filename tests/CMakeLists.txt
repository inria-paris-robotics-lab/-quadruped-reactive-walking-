#
# Copyright (C) 2022-2023 LAAS-CNRS, INRIA
#

find_package(Boost REQUIRED COMPONENTS unit_test_framework)

macro(add_test_cflags test_name flag)
  set_property(
    TARGET ${test_name}
    APPEND_STRING
    PROPERTY COMPILE_FLAGS " ${flag}")
endmacro()

function(add_qrw_test name dependencies)
  set(test_name "test-cpp-${name}")
  set(test_file ${name}.cpp)
  message(STATUS "Adding C++ test: ${test_file} (${test_name})")

  add_unit_test(${test_name} ${test_file})
  set_target_properties(${test_name} PROPERTIES LINKER_LANGUAGE CXX)
  target_include_directories(${test_name} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

  # define macros required by boost_test
  # define module name, replace '-' by '_'
  set(MODULE_NAME "${name}Test")
  string(REGEX REPLACE "-" "_" MODULE_NAME ${MODULE_NAME})

  add_test_cflags(${test_name} "-DBOOST_TEST_DYN_LINK")
  add_test_cflags(${test_name} "-DBOOST_TEST_MODULE=${MODULE_NAME}")

  target_link_libraries(${test_name} PUBLIC ${dependencies})
  target_link_libraries(${test_name} PRIVATE Boost::unit_test_framework)
endfunction()

set(TEST_NAMES residual-models)

foreach(test_name ${TEST_NAMES})
  add_qrw_test(${test_name} ${PROJECT_NAME})
endforeach(test_name)
