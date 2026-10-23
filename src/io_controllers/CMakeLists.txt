cmake_minimum_required(VERSION 3.8)
project(io_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Python version — needed to compute the correct site-packages install path
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

# --------------------------------------------------------------------------
# ROS 2 interface generation.
# SKIP_INSTALL: prevents rosidl from calling ament_python_install_package
# internally, which would conflict with our own call below.
# See: https://github.com/ros2/rosidl_python/issues/141
# --------------------------------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DigitalIO.msg"
  "msg/AnalogIO.msg"
  "msg/IOStates.msg"
  "srv/SetIO.srv"
  "srv/SetMockBehavior.srv"
  "action/MonitorIO.action"
  DEPENDENCIES std_msgs builtin_interfaces
  SKIP_INSTALL
)

# Install the Python module so it is importable (import io_controllers...).
ament_python_install_package(${PROJECT_NAME})

# Install the generated Python interface code that SKIP_INSTALL skipped.
# rosidl_generator_py writes these files to
# ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py/. Evaluated at install time
# (after build), so the directory will exist.
install(
  DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py/${PROJECT_NAME}/"
  DESTINATION
    "lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages/${PROJECT_NAME}"
)

# Install the generated C/C++ type support shared libraries that SKIP_INSTALL
# skipped. These are required at runtime by any node (Python or C++) that uses
# these interfaces.
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

# Export the installed targets (namespace defaults to ${PROJECT_NAME}::) and
# register them as typesupport targets so downstream `rosidl_generate_interfaces`
# populates ${PROJECT_NAME}_TARGETS and links against them. SKIP_INSTALL omits
# this, which breaks any package that references these interfaces in its own
# interface files (runtime "undefined symbol" / "Could not import
# 'rosidl_typesupport_c'" failure).
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

# The Python conversion library (lib${PROJECT_NAME}__rosidl_generator_py.so) is a
# SEPARATE chain from the C typesupport libraries above, and SKIP_INSTALL drops its
# install + export too. Downstream interface packages link it via the *unsuffixed*
# ${PROJECT_NAME}_TARGETS variable; without it they fail at runtime with
# "undefined symbol: <pkg>__msg__<type>__convert_from_py".
# Installing it also keeps the Python extension modules from depending on build/.
install(
  TARGETS ${PROJECT_NAME}__rosidl_generator_py
  EXPORT export_${PROJECT_NAME}__rosidl_generator_py
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
rosidl_export_typesupport_targets("" ${PROJECT_NAME}__rosidl_generator_py)
ament_export_targets(export_${PROJECT_NAME}__rosidl_generator_py)

# Install the lifecycle node program so it is discoverable as
# `ros2 run io_controllers gpio_controller` and via the launch file.
install(PROGRAMS
  ${PROJECT_NAME}/gpio_controller.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME gpio_controller
)

# Install the launch files and sample config to the package share directory so
# they are discoverable via `ros2 launch io_controllers ...` and resolvable by
# the config loader (share/io_controllers/config).
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
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
  std_msgs
  action_msgs
)
ament_package()
