cmake_minimum_required(VERSION 3.16)
project(dm_openarm VERSION 0.1.8 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED IMPORTED_TARGET libusb-1.0)
find_package(yaml-cpp REQUIRED)

include(CTest)

option(DM_OPENARM_BUILD_PYTHON "Build Python bindings" OFF)
option(DM_OPENARM_BUILD_EXAMPLES "Build dm_openarm examples" ON)

set(DMCAN_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/damiao_sdk/lib)
set(DMCAN_SO ${DMCAN_LIB_DIR}/libdm_device.so)
if(NOT EXISTS ${DMCAN_SO})
  message(FATAL_ERROR "Missing ${DMCAN_SO}; copy from resources/u2canfd/dlls/libdm_device.so")
endif()

add_library(dmcan_device SHARED IMPORTED GLOBAL)
set_target_properties(
  dmcan_device
  PROPERTIES
    IMPORTED_LOCATION ${DMCAN_SO}
    INTERFACE_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR}/third_party/damiao_sdk/include
)

add_library(
  damiao_sdk STATIC
  third_party/damiao_sdk/src/protocol/damiao.cpp
)
target_include_directories(
  damiao_sdk
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/damiao_sdk/include
)
target_link_libraries(
  damiao_sdk
  PUBLIC
    Threads::Threads
    dmcan_device
    PkgConfig::LIBUSB
)
set_target_properties(damiao_sdk PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(
  dm_openarm
  src/backend/dm_serial_backend.cpp
  src/dm_arm.cpp
  src/mit_control_command.cpp
  src/mit_loop_controller.cpp
  src/yaml_loader.cpp
)
target_include_directories(
  dm_openarm
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(
  dm_openarm
  PUBLIC
    damiao_sdk
    yaml-cpp
)
set_target_properties(
  dm_openarm
  PROPERTIES
    POSITION_INDEPENDENT_CODE ON
    BUILD_RPATH ${DMCAN_LIB_DIR}
    INSTALL_RPATH "\$ORIGIN;\$ORIGIN/../lib"
)

function(dm_openarm_set_rpath target_name)
  set_target_properties(
    ${target_name}
    PROPERTIES
      BUILD_RPATH ${DMCAN_LIB_DIR}
      INSTALL_RPATH "\$ORIGIN;\$ORIGIN/../lib"
  )
endfunction()

if(DM_OPENARM_BUILD_EXAMPLES)
  add_executable(check_comm examples/check_comm.cpp)
  target_link_libraries(check_comm PRIVATE dm_openarm)
  dm_openarm_set_rpath(check_comm)

  add_executable(enable_disable examples/enable_disable.cpp)
  target_link_libraries(enable_disable PRIVATE dm_openarm)
  dm_openarm_set_rpath(enable_disable)

  add_executable(hold_position examples/hold_position.cpp)
  target_link_libraries(hold_position PRIVATE dm_openarm)
  dm_openarm_set_rpath(hold_position)

  add_executable(set_zero_position examples/set_zero_position.cpp)
  target_link_libraries(set_zero_position PRIVATE dm_openarm)
  dm_openarm_set_rpath(set_zero_position)

  add_executable(arm_mit_control examples/arm_mit_control.cpp)
  target_link_libraries(arm_mit_control PRIVATE dm_openarm)
  dm_openarm_set_rpath(arm_mit_control)
endif()

if(DM_OPENARM_BUILD_PYTHON)
  find_package(
    Python
    COMPONENTS Interpreter Development.Module
    REQUIRED)
  execute_process(
    COMMAND ${Python_EXECUTABLE} -m nanobind --cmake_dir
    OUTPUT_STRIP_TRAILING_WHITESPACE
    OUTPUT_VARIABLE nanobind_ROOT)
  find_package(nanobind CONFIG REQUIRED)

  nanobind_add_module(dm_openarm_python src/python_bindings.cpp)
  set_target_properties(dm_openarm_python PROPERTIES OUTPUT_NAME "_core")
  target_link_libraries(dm_openarm_python PRIVATE dm_openarm)
  dm_openarm_set_rpath(dm_openarm_python)
  # Also install the shared device library next to the extension when packaging.
  install(TARGETS dm_openarm_python LIBRARY DESTINATION dm_openarm)
  install(FILES ${DMCAN_SO} DESTINATION dm_openarm)
endif()

if(BUILD_TESTING)
  add_executable(test_yaml_loader tests/test_yaml_loader.cpp)
  target_link_libraries(test_yaml_loader PRIVATE dm_openarm)
  target_compile_definitions(
    test_yaml_loader PRIVATE DM_OPENARM_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  dm_openarm_set_rpath(test_yaml_loader)
  add_test(NAME test_yaml_loader COMMAND test_yaml_loader)

  add_executable(test_mit_control_command tests/test_mit_control_command.cpp)
  target_link_libraries(test_mit_control_command PRIVATE dm_openarm)
  dm_openarm_set_rpath(test_mit_control_command)
  add_test(NAME test_mit_control_command COMMAND test_mit_control_command)

  add_executable(test_mit_loop_controller tests/test_mit_loop_controller.cpp)
  target_link_libraries(test_mit_loop_controller PRIVATE dm_openarm)
  dm_openarm_set_rpath(test_mit_loop_controller)
  add_test(NAME test_mit_loop_controller COMMAND test_mit_loop_controller)

endif()

if(BUILD_TESTING)
  add_executable(test_command_guard tests/test_command_guard.cpp)
  target_include_directories(test_command_guard PRIVATE include third_party/damiao_sdk/include)
  add_test(NAME test_command_guard COMMAND test_command_guard)
endif()

if(BUILD_TESTING)
  add_executable(test_shared_bus tests/test_shared_bus.cpp)
  target_link_libraries(test_shared_bus PRIVATE dm_openarm)
  add_test(NAME test_shared_bus COMMAND test_shared_bus)
endif()
