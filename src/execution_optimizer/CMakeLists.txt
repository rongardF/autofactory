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
project(execution_optimizer)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

find_package(geometry_msgs REQUIRED)
find_package(repositories REQUIRED)

# SKIP_INSTALL: rosidl must not call ament_python_install_package itself, it
# would conflict with the call below (https://github.com/ros2/rosidl_python/issues/141).
set(interface_files
  msg/PatternGroup.msg
  srv/DispensingOptimizePatterns.srv
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${interface_files}
  DEPENDENCIES geometry_msgs repositories
  SKIP_INSTALL
)

# SKIP_INSTALL also skips installing the interface definitions and registering
# them in the ament index, which `ros2 interface show/list` rely on.
set(interface_index_lines "")
foreach(file ${interface_files})
  get_filename_component(dir "${file}" DIRECTORY)
  get_filename_component(name "${file}" NAME_WE)
  install(
    FILES
      "${file}"
      "${CMAKE_CURRENT_BINARY_DIR}/rosidl_adapter/${PROJECT_NAME}/${dir}/${name}.idl"
    DESTINATION share/${PROJECT_NAME}/${dir}
  )
  list(APPEND interface_index_lines "${dir}/${name}.idl" "${file}")
endforeach()
list(SORT interface_index_lines)
string(REPLACE ";" "\n" interface_index_lines "${interface_index_lines}")
ament_index_register_resource("rosidl_interfaces" CONTENT "${interface_index_lines}")

# Install the Python module so it is importable (import execution_optimizer...).
ament_python_install_package(${PROJECT_NAME})

# Generated Python interface code that SKIP_INSTALL skipped.
install(
  DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py/${PROJECT_NAME}/"
  DESTINATION
    "lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages/${PROJECT_NAME}"
)

# Type support libraries that SKIP_INSTALL skipped; needed at runtime.
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

# Export the installed targets and register them as typesupport targets so
# downstream `rosidl_generate_interfaces` calls can link against them.
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

# The Python conversion library is a separate chain that SKIP_INSTALL also
# drops; downstream packages link it through the unsuffixed target variable.
install(
  TARGETS ${PROJECT_NAME}__rosidl_generator_py
  EXPORT export_${PROJECT_NAME}__rosidl_generator_py
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
rosidl_export_typesupport_targets("" ${PROJECT_NAME}__rosidl_generator_py)
ament_export_targets(export_${PROJECT_NAME}__rosidl_generator_py)

install(DIRECTORY config launch msg srv
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/dispensing_optimizer
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(
  rosidl_default_runtime
  geometry_msgs
  repositories
  action_msgs
)
ament_package()
