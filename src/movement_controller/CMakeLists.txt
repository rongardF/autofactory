# Copyright (c) 2026, Movement Controller Contributors
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

cmake_minimum_required(VERSION 3.8)
project(movement_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --------------------------------------------------------------------------
# Required packages
# --------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Python version — needed to compute the correct site-packages install path
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

# Message dependencies used in interface files
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)

# --------------------------------------------------------------------------
# ROS2 interface generation
# SKIP_INSTALL: prevents rosidl from calling ament_python_install_package
# internally, which would conflict with our own call below.
# See: https://github.com/ros2/rosidl_python/issues/141
# --------------------------------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ExecuteTrajectory.action"
  "msg/TrajectoryPath.msg"
  DEPENDENCIES geometry_msgs shape_msgs
  SKIP_INSTALL
)

# --------------------------------------------------------------------------
# Python package installation
# --------------------------------------------------------------------------
# Install our Python source (symlinked with --symlink-install)
ament_python_install_package(${PROJECT_NAME})

# Install the generated Python interface code that SKIP_INSTALL skipped.
# rosidl_generator_py writes these files to ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py/.
# Evaluated at install time (after build), so the directory will exist.
install(
  DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py/${PROJECT_NAME}/"
  DESTINATION
    "lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages/${PROJECT_NAME}"
)

# Install the generated C/C++ type support shared libraries that SKIP_INSTALL skipped.
# These are required at runtime by any node (Python or C++) that uses these interfaces.
install(
  TARGETS
    ${PROJECT_NAME}__rosidl_generator_c
    ${PROJECT_NAME}__rosidl_typesupport_c
    ${PROJECT_NAME}__rosidl_typesupport_cpp
    ${PROJECT_NAME}__rosidl_typesupport_fastrtps_c
    ${PROJECT_NAME}__rosidl_typesupport_fastrtps_cpp
    ${PROJECT_NAME}__rosidl_typesupport_introspection_c
    ${PROJECT_NAME}__rosidl_typesupport_introspection_cpp
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# --------------------------------------------------------------------------
# Testing
# --------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  # NOTE: ament_cmake_pytest does NOT auto-discover tests — list each file explicitly.
  # PYTEST_ADDOPTS passes --import-mode=importlib to prevent pytest from inserting
  # the rootdir into sys.path[0], which would shadow the installed package.
  ament_add_pytest_test(test_imports
    "tests/unit/test_imports.py"
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py
    ENV PYTEST_ADDOPTS=--import-mode=importlib
    TIMEOUT 60
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  )

  ament_add_pytest_test(test_enums_and_dtos
    "tests/unit/test_enums_and_dtos.py"
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py
    ENV PYTEST_ADDOPTS=--import-mode=importlib
    TIMEOUT 60
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  )

  ament_add_pytest_test(test_trajectory_grouper
    "tests/unit/test_trajectory_grouper.py"
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py
    ENV PYTEST_ADDOPTS=--import-mode=importlib
    TIMEOUT 60
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  )

  ament_add_pytest_test(test_movement_controller
    "tests/unit/test_movement_controller.py"
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py
    ENV PYTEST_ADDOPTS=--import-mode=importlib
    TIMEOUT 60
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  )
endif()

# --------------------------------------------------------------------------
# Install executable scripts
# --------------------------------------------------------------------------
install(
  PROGRAMS scripts/movement_controller
  DESTINATION lib/${PROJECT_NAME}
)

# --------------------------------------------------------------------------
# Install launch, config, urdf, world and srdf files
# --------------------------------------------------------------------------
install(DIRECTORY config launch srdf urdf world
  DESTINATION share/${PROJECT_NAME}
)

# --------------------------------------------------------------------------
# Finalize
# --------------------------------------------------------------------------
ament_export_dependencies(rosidl_default_runtime)
ament_package()
