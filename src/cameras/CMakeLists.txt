cmake_minimum_required(VERSION 3.8)
project(cameras)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Install the Python module so it is importable (import cameras...).
ament_python_install_package(${PROJECT_NAME})

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

ament_package()
