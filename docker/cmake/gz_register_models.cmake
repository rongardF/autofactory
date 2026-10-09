# gz_register_models.cmake
#
# Shared Gazebo helper baked into the container image (see docker/Dockerfile).
# Included by ROS 2 packages via:
#
#   list(APPEND CMAKE_MODULE_PATH "$ENV{GZ_CMAKE_MODULE_DIR}")
#   include(gz_register_models)
#   gz_register_models(config/model)
#
# ---------------------------------------------------------------------------
# gz_register_models(<relative-share-path>)
#
# Registers an already-installed directory (the one that directly contains the
# model:// folders) onto GZ_SIM_RESOURCE_PATH via an ament environment hook.
#
# The directory must already be installed under share/<PROJECT_NAME>/ by a
# regular install(DIRECTORY ...) rule; this macro only wires up the runtime
# environment, it does not install anything itself (keeps model assets from
# being installed twice).
#
# At runtime, sourcing the workspace overlay prepends
#   <install-prefix>/share/<PROJECT_NAME>/<relative-share-path>
# to GZ_SIM_RESOURCE_PATH. Duplicate entries are harmless to Gazebo.
#
# Example (station):
#   install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
#   gz_register_models(config/model)   # contains station/, textured_ground/, ...
# ---------------------------------------------------------------------------

macro(gz_register_models _gz_rel_path)
  if(NOT COMMAND ament_environment_hooks)
    message(FATAL_ERROR
      "gz_register_models(): ament_environment_hooks is not available. "
      "Call find_package(ament_cmake REQUIRED) before gz_register_models().")
  endif()

  # Generate a colcon DSV environment hook. The 'prepend-non-duplicate'
  # descriptor records a path RELATIVE to the install prefix; colcon resolves it
  # against the package's actual prefix when sourcing local_setup.dsv. This is
  # leak-proof and relocatable.
  #
  # A .sh hook using $AMENT_CURRENT_PREFIX must NOT be used here: for an overlay
  # package, AMENT_CURRENT_PREFIX can retain the base ROS prefix (/opt/ros/jazzy)
  # because the package local_setup.sh only sets it when unset and does not
  # override an already-existing directory. That makes GZ_SIM_RESOURCE_PATH
  # resolve to a non-existent /opt/ros/jazzy/share/<pkg>/... path. The DSV
  # descriptor avoids the shell variable entirely, mirroring how the stock
  # path/pythonpath hooks resolve correctly.
  # ${PROJECT_NAME} and ${_gz_rel_path} are expanded by CMake at configure time.
  set(_gz_hook_file "${CMAKE_CURRENT_BINARY_DIR}/gz_resource_path.dsv")
  file(WRITE "${_gz_hook_file}"
    "prepend-non-duplicate;GZ_SIM_RESOURCE_PATH;share/${PROJECT_NAME}/${_gz_rel_path}\n")

  ament_environment_hooks("${_gz_hook_file}")
endmacro()
