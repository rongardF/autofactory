cmake_minimum_required(VERSION 3.8)
project(repositories)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(action_msgs REQUIRED)

# Python version — needed to compute the correct site-packages install path
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

# --------------------------------------------------------------------------
# ROS 2 interface generation.
# SKIP_INSTALL: prevents rosidl from calling ament_python_install_package
# internally, which would conflict with our own call below.
# See: https://github.com/ros2/rosidl_python/issues/141
# --------------------------------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/StationCacheStringCache.msg"
  "msg/StationCacheBooleanCache.msg"
  "msg/StationCacheNumberCache.msg"
  "msg/StationCacheStringArrayCache.msg"
  "msg/StationCacheBytesCache.msg"
  "msg/StationCacheNumberArrayCache.msg"
  "srv/StationCacheStoreString.srv"      "srv/StationCacheFetchString.srv"
  "srv/StationCacheStoreBoolean.srv"     "srv/StationCacheFetchBoolean.srv"
  "srv/StationCacheStoreNumber.srv"      "srv/StationCacheFetchNumber.srv"
  "srv/StationCacheStoreStringArray.srv" "srv/StationCacheFetchStringArray.srv"
  "srv/StationCacheStoreBytes.srv"       "srv/StationCacheFetchBytes.srv"
  "srv/StationCacheStoreNumberArray.srv" "srv/StationCacheFetchNumberArray.srv"
  "srv/StationCacheDelete.srv"
  # object_model_storage interfaces
  "msg/ObjectModelStorageMappingValue.msg"
  "msg/ObjectModelStorageModelInfo.msg"
  "msg/ObjectModelStorageCachedModels.msg"
  "srv/ObjectModelStorageGetModelsInfo.srv"
  "srv/ObjectModelStorageGetModel.srv"
  "action/ObjectModelStorageGetModels.action"
  DEPENDENCIES builtin_interfaces action_msgs
  SKIP_INSTALL
)

# Install the Python module so it is importable (import repositories...).
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

# Install the lifecycle node programs so they are discoverable as
# `ros2 run repositories <node>`.
install(PROGRAMS
  ${PROJECT_NAME}/station_cache.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME station_cache
)
install(PROGRAMS
  ${PROJECT_NAME}/object_model_storage.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME object_model_storage
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

ament_package()
