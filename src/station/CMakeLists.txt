cmake_minimum_required(VERSION 3.8)
project(station)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Install launch files and the station-specific description/scene assets
# (URDF, SRDF, world) so they are available in the package share directory
# and can be forwarded to the movement_controller launch file.
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Register this package's Gazebo models on GZ_SIM_RESOURCE_PATH.
# gz_register_models is baked into the container image (see docker/cmake).
# config/model contains the station/, textured_ground/, tool_mount/, conveyor/
# and calibration_board/ model:// folders.
list(APPEND CMAKE_MODULE_PATH "$ENV{GZ_CMAKE_MODULE_DIR}")
include(gz_register_models)
gz_register_models(config/model)

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
