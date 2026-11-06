#!/usr/bin/env bash
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
#
# Prepare the local-mode files and launch the station_workplace testbench.
#
# Copies the assets into DOWNLOAD_PATH (object_model_storage download_path and
# production_manager assembly file location), puts DOWNLOAD_PATH on
# GZ_SIM_RESOURCE_PATH and runs testbench.launch.py. Extra arguments are
# forwarded to the launch, e.g.:
#
#   ./setup.sh                 # Gazebo GUI + RViz
#   ./setup.sh rviz:=false
#   DOWNLOAD_PATH=/tmp/other ./setup.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"
ASSETS_DIR="${SCRIPT_DIR}/assets"
DOWNLOAD_PATH="${DOWNLOAD_PATH:-/tmp/station}"

# Model IDs, see assets/local_storage.json
CARRIER_SDF_ID=c4e38990-3383-4375-b2ac-0d1468d6604f
PRODUCT_SDF_ID=fd71afb0-d3e9-428c-a7e6-b2613b877088
CARRIER_STL_ID=382eb88f-e429-46bd-a4e0-3a6dbd8564a0
PRODUCT_STL_ID=edf9129e-6656-4d99-895a-85d6b7de4ac7

# Environment (setup scripts are not 'set -u' safe)
set +u
if [[ -z "${VIRTUAL_ENV:-}" && -f /opt/venv/bin/activate ]]; then
    source /opt/venv/bin/activate
fi
source /opt/ros/jazzy/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"
set -u

# Assets; existing entries are removed first so 'cp -r' does not nest folders
mkdir -p "${DOWNLOAD_PATH}"
rm -rf "${DOWNLOAD_PATH:?}/${CARRIER_SDF_ID}" "${DOWNLOAD_PATH:?}/${PRODUCT_SDF_ID}" \
       "${DOWNLOAD_PATH:?}/${CARRIER_STL_ID}" "${DOWNLOAD_PATH:?}/${PRODUCT_STL_ID}"

cp "${ASSETS_DIR}/local_storage.json" "${DOWNLOAD_PATH}/local_storage.json"
cp "${ASSETS_DIR}/mock_assembly_file.json" "${DOWNLOAD_PATH}/mock_assembly_file.json"

cp -r "${ASSETS_DIR}/carrier_model" "${DOWNLOAD_PATH}/${CARRIER_SDF_ID}"
cp -r "${ASSETS_DIR}/product_model" "${DOWNLOAD_PATH}/${PRODUCT_SDF_ID}"
cp "${ASSETS_DIR}/carrier_model/meshes/collision.stl" "${DOWNLOAD_PATH}/${CARRIER_STL_ID}"
cp "${ASSETS_DIR}/product_model/meshes/collision.stl" "${DOWNLOAD_PATH}/${PRODUCT_STL_ID}"

# Gazebo resolves model://<model_id>/... from the download path
export GZ_SIM_RESOURCE_PATH="${DOWNLOAD_PATH}${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"

echo "Assets copied to ${DOWNLOAD_PATH} (ROS_DOMAIN_ID=${ROS_DOMAIN_ID})"
exec ros2 launch "${SCRIPT_DIR}/testbench.launch.py" download_path:="${DOWNLOAD_PATH}" "$@"
