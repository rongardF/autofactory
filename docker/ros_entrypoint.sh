#!/bin/bash
set -e

# Enforce required variables for the prod profile.
# Validation is here (not in docker-compose :? interpolation) so that dev-profile
# commands (docker compose up mongodb ...) do not fail due to MONGO_URI being unset.
if [ "${APP_PROFILE:-}" = "prod" ]; then
    if [ -z "${WORKSTATION_ID:-}" ]; then
        echo "ERROR: WORKSTATION_ID must be set for prod profile" >&2
        exit 1
    fi
fi

# Activate the project venv (must come before ROS 2 sourcing so venv python3 is
# used by colcon and all downstream commands; --system-site-packages ensures
# rclpy and other ROS 2 apt packages remain visible inside the venv)
. /opt/venv/bin/activate

# Source ROS 2 base setup
. /opt/ros/jazzy/setup.bash

# Source the colcon workspace overlay if it exists (safe no-op if not yet built)
if [ -f "/workspaces/autofactory/install/setup.bash" ]; then
    . /workspaces/autofactory/install/setup.bash
fi

exec "$@"
