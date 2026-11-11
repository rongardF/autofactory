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
"""Frozen configuration of the ``mock_localizer`` node."""

from pathlib import Path

from pydantic import BaseModel, ConfigDict, Field, field_validator

from localization.enums.constants import ROS_NAME_PATTERN, ROS_NODE_PATH_PATTERN
from localization.models.pose6 import Pose6


class LocalizerConfigDTO(BaseModel):
    """Validated node parameters."""

    model_config = ConfigDict(frozen=True)

    station_workplace_node_name: str = Field(
        default='station_workplace', pattern=ROS_NAME_PATTERN,
        description='Node publishing the workplace state.')
    movement_controller_node_name: str = Field(
        default='movement_controller', pattern=ROS_NAME_PATTERN,
        description='Node serving the execute_trajectory action.')
    camera_node_name: str = Field(
        default='cameras/basler_camera', pattern=ROS_NODE_PATH_PATTERN,
        description='Camera node (with its namespace) serving the get_image action.')
    camera_frame_id: str = Field(
        default='camera_tcp_calibrated', pattern=ROS_NAME_PATTERN,
        description='Camera TCP frame, used as tool frame for the camera movements.')
    base_frame_id: str = Field(
        default='station', pattern=ROS_NAME_PATTERN,
        description='Frame of the initial localization pose; parent of localization_origin.')
    carrier_frame_id: str = Field(
        default='carrier', pattern=ROS_NAME_PATTERN,
        description='Parent frame of product_origin.')
    initial_localization_pose: Pose6 = Field(
        description='Initial image capture pose [x, y, z, roll, pitch, yaw] in base_frame_id.')
    calibration_poses_file: str = Field(
        description='Absolute path of the YAML file with the localization image capture poses.')
    image_capture_settling_s: float = Field(
        default=0.5, ge=0.0, allow_inf_nan=False,
        description='Time to wait after reaching a pose before capturing the image [s].')
    image_capture_timeout_s: float = Field(
        default=5.0, gt=0.0, allow_inf_nan=False,
        description='Time allowed for the camera to return an image [s].')

    @field_validator('calibration_poses_file')
    @classmethod
    def _absolute_path(cls, value: str) -> str:
        if not Path(value).is_absolute():
            raise ValueError('must be an absolute file path')
        return value

    @property
    def workplace_topic(self) -> str:
        """Topic of the workplace state."""
        return f'/{self.station_workplace_node_name}/workplace'

    @property
    def execute_trajectory_action(self) -> str:
        """Name of the movement action."""
        return f'/{self.movement_controller_node_name}/execute_trajectory'

    @property
    def get_image_action(self) -> str:
        """Name of the camera image action."""
        return f'/{self.camera_node_name}/get_image'
