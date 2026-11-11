cmake_minimum_required(VERSION 3.8)
project(cameras)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(action_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

# SKIP_INSTALL: rosidl must not call ament_python_install_package itself, it
# would conflict with the call below (https://github.com/ros2/rosidl_python/issues/141).
set(interface_files
  action/GetImage.action
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${interface_files}
  DEPENDENCIES builtin_interfaces action_msgs sensor_msgs
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

# Install the Python module so it is importable (import cameras...).
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

# Install camera model files (meshes) referenced via
# $(find cameras)/config/model/... and launch files.
install(
  DIRECTORY config launch urdf srdf
  DESTINATION share/${PROJECT_NAME}
)

# Register this package's Gazebo models on GZ_SIM_RESOURCE_PATH.
# gz_register_models is baked into the container image (see docker/cmake).
# config/model contains the basler/ model:// folder.
list(APPEND CMAKE_MODULE_PATH "$ENV{GZ_CMAKE_MODULE_DIR}")
include(gz_register_models)
gz_register_models(config/model)

# Install the simulated camera node so it is discoverable as
# `ros2 run cameras basler_camera` and via the launch file.
install(PROGRAMS
  ${PROJECT_NAME}/basler_camera.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME basler_camera
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(
  rosidl_default_runtime
  builtin_interfaces
  action_msgs
  sensor_msgs
)
ament_package()
