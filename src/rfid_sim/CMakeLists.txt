cmake_minimum_required(VERSION 3.10)
project(rfid_sim)

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
# System plugin shared library
# Both plugins and the custom component live in one library so the
# RfidTagData component type identity is shared between them.
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

# --------------------------------------------------------------------------
# Install
# --------------------------------------------------------------------------
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

install(
  DIRECTORY config launch worlds
  DESTINATION share/${PROJECT_NAME}
)

# Add the library directory to GZ_SIM_SYSTEM_PLUGIN_PATH so `gz sim` can load
# the plugins by their `filename="rfid_sim_systems"`.
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Copyright headers are present; keep the copyright linter disabled to avoid
  # false positives on the SPDX-style header used across this repo.
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
