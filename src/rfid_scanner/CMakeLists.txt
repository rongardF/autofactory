cmake_minimum_required(VERSION 3.10)
project(rfid_scanner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --------------------------------------------------------------------------
# Dependencies
# Gazebo Harmonic is provided via the ROS 2 Jazzy *_vendor packages. Each
# vendor package exposes a version-less redirect config (e.g. `gz-sim`) that
# resolves to the concrete versioned target (`gz-sim8::gz-sim8`).
# --------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Python version — needed to compute the correct site-packages install path
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED COMPONENTS register)
find_package(gz_transport_vendor REQUIRED)
find_package(gz-transport REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz-msgs REQUIRED)
find_package(gz_math_vendor REQUIRED)
find_package(gz-math REQUIRED)
find_package(gz_common_vendor REQUIRED)
find_package(gz-common REQUIRED)

# --------------------------------------------------------------------------
# Gazebo system plugin shared library
# Both plugins and the custom component live in one library so the
# RfidTagData component type identity is shared between them. The library
# name is kept as `rfid_sim_systems` so existing SDF/xacro usages stay valid.
# --------------------------------------------------------------------------
add_library(rfid_sim_systems SHARED
  src/RfidTag.cc
  src/RfidReader.cc
  src/encoding.cc
  src/link_resolver.cc
  src/register_plugins.cc
)

target_include_directories(rfid_sim_systems PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

target_link_libraries(rfid_sim_systems PUBLIC
  gz-sim::gz-sim
  gz-plugin::register
  gz-transport::gz-transport
  gz-msgs::gz-msgs
  gz-math::gz-math
  gz-common::gz-common
)

install(
  TARGETS rfid_sim_systems
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

# Add the library directory to GZ_SIM_SYSTEM_PLUGIN_PATH so `gz sim` can load
# the plugins by their `filename="rfid_sim_systems"`.
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

# --------------------------------------------------------------------------
# ROS2 interface generation
# SKIP_INSTALL: prevents rosidl from calling ament_python_install_package
# internally, which would conflict with our own call below.
# See: https://github.com/ros2/rosidl_python/issues/141
# --------------------------------------------------------------------------
set(interface_names ReadData WriteData)
set(interface_files "")
foreach(name ${interface_names})
  list(APPEND interface_files "srv/${name}.srv")
endforeach()

rosidl_generate_interfaces(${PROJECT_NAME}
  ${interface_files}
  SKIP_INSTALL
)

# SKIP_INSTALL also skips installing the interface definitions and registering
# them in the ament index, which `ros2 interface show/list` rely on.
set(interface_index_lines "")
foreach(name ${interface_names})
  install(
    FILES
      "srv/${name}.srv"
      "${CMAKE_CURRENT_BINARY_DIR}/rosidl_adapter/${PROJECT_NAME}/srv/${name}.idl"
    DESTINATION share/${PROJECT_NAME}/srv
  )
  list(APPEND interface_index_lines "srv/${name}.idl" "srv/${name}.srv")
endforeach()
list(SORT interface_index_lines)
string(REPLACE ";" "\n" interface_index_lines "${interface_index_lines}")
ament_index_register_resource("rosidl_interfaces" CONTENT "${interface_index_lines}")

# Install the Python module so it is importable (import rfid_scanner...).
ament_python_install_package(${PROJECT_NAME})

# Install the generated Python interface code that SKIP_INSTALL skipped.
install(
  DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py/${PROJECT_NAME}/"
  DESTINATION
    "lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages/${PROJECT_NAME}"
)

# Install and export the generated C/C++ type support libraries that
# SKIP_INSTALL skipped; they are required at runtime by any node using these
# interfaces.
install(
  TARGETS
    ${PROJECT_NAME}__rosidl_generator_c
    ${PROJECT_NAME}__rosidl_generator_cpp
    ${PROJECT_NAME}__rosidl_typesupport_c
    ${PROJECT_NAME}__rosidl_typesupport_cpp
    ${PROJECT_NAME}__rosidl_typesupport_fastrtps_c
    ${PROJECT_NAME}__rosidl_typesupport_fastrtps_cpp
    ${PROJECT_NAME}__rosidl_typesupport_introspection_c
    ${PROJECT_NAME}__rosidl_typesupport_introspection_cpp
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME})
rosidl_export_typesupport_targets(__rosidl_generator_c
  ${PROJECT_NAME}__rosidl_generator_c)
rosidl_export_typesupport_targets(__rosidl_generator_cpp
  ${PROJECT_NAME}__rosidl_generator_cpp)
rosidl_export_typesupport_targets(__rosidl_typesupport_c
  ${PROJECT_NAME}__rosidl_typesupport_c)
rosidl_export_typesupport_targets(__rosidl_typesupport_cpp
  ${PROJECT_NAME}__rosidl_typesupport_cpp)
rosidl_export_typesupport_targets(__rosidl_typesupport_fastrtps_c
  ${PROJECT_NAME}__rosidl_typesupport_fastrtps_c)
rosidl_export_typesupport_targets(__rosidl_typesupport_fastrtps_cpp
  ${PROJECT_NAME}__rosidl_typesupport_fastrtps_cpp)
rosidl_export_typesupport_targets(__rosidl_typesupport_introspection_c
  ${PROJECT_NAME}__rosidl_typesupport_introspection_c)
rosidl_export_typesupport_targets(__rosidl_typesupport_introspection_cpp
  ${PROJECT_NAME}__rosidl_typesupport_introspection_cpp)

# The Python conversion library is a separate chain from the C typesupport
# libraries above, and SKIP_INSTALL drops its install + export too.
install(
  TARGETS ${PROJECT_NAME}__rosidl_generator_py
  EXPORT export_${PROJECT_NAME}__rosidl_generator_py
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
rosidl_export_typesupport_targets("" ${PROJECT_NAME}__rosidl_generator_py)
ament_export_targets(export_${PROJECT_NAME}__rosidl_generator_py)

# Install the SIMATIC RF240R node so it is discoverable as
# `ros2 run rfid_scanner simatic_rf240r_ascii` and via the launch file.
install(PROGRAMS
  ${PROJECT_NAME}/simatic_rf240r_ascii.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME simatic_rf240r_ascii
)

# --------------------------------------------------------------------------
# Resources
# --------------------------------------------------------------------------
install(
  DIRECTORY launch urdf srdf model config worlds
  DESTINATION share/${PROJECT_NAME}
)

# Register this package's Gazebo models on GZ_SIM_RESOURCE_PATH.
# gz_register_models is baked into the container image (see docker/cmake).
# model contains the simatic/ model:// folder.
list(APPEND CMAKE_MODULE_PATH "$ENV{GZ_CMAKE_MODULE_DIR}")
include(gz_register_models)
gz_register_models(model)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
