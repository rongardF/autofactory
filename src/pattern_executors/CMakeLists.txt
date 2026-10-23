# Copyright (c) 2026, Autofactory
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
project(pattern_executors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

find_package(geometry_msgs REQUIRED)
find_package(movement_controller REQUIRED)

get_filename_component(
  MOVEMENT_CONTROLLER_PREFIX
  "${movement_controller_DIR}/../../.."
  ABSOLUTE
)
include_directories("${MOVEMENT_CONTROLLER_PREFIX}/include")

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ExecuteDispensingPattern.action"
  "msg/PatternAction.msg"
  "msg/DispensingMovement.msg"
  "msg/DispensingPattern.msg"
  "msg/DispensedElementMetric.msg"
  "msg/DispensingPatternMetric.msg"
  DEPENDENCIES geometry_msgs movement_controller
  SKIP_INSTALL
)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py/${PROJECT_NAME}/"
  DESTINATION
    "lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages/${PROJECT_NAME}"
)

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

install(DIRECTORY config launch action msg
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/dispensing_pattern_executor
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
