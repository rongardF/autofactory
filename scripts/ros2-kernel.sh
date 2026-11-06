#!/usr/bin/env bash
# Jupyter kernel launcher that sources the ROS 2 overlay BEFORE starting the
# kernel, so LD_LIBRARY_PATH (and the rest of the overlay env) is set in the
# actual kernel process. VS Code's python.envFile does not reliably propagate
# LD_LIBRARY_PATH to notebook kernels, which breaks rosidl typesupport dlopen.
# Note: no `set -u` — ROS setup.bash references optional unset variables.
set -e

source /opt/ros/jazzy/setup.bash
source /workspaces/autofactory/install/setup.bash

exec /opt/venv/bin/python "$@"
