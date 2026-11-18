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
"""Step 13.1 / 13.3: move the camera to a pose, capture an image and store it."""

import copy
import uuid

from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import Image

from cameras.action import GetImage
from manufacturing_executor.exceptions import SystemFailureError
from manufacturing_executor.models.manufacturing_executor_config_dto import (
    ManufacturingExecutorConfigDTO,
)
from manufacturing_executor.services.abort_context import AbortContext
from manufacturing_executor.services.async_calls import call_service, run_action, sleep_abortable
from manufacturing_executor.services.loop_clients import LoopClients
from manufacturing_executor.services.movement_session_client import MovementSessionClient
from manufacturing_executor.utils.capture_metadata import build_capture_metadata
from movement_controller.action import ExecuteTrajectory
from movement_controller.msg import TrajectoryPath
from repositories.srv import ImageRepositoryUploadImage
from station_workplace.models.product_data_dto import ProductDataDTO


class ImageCaptureService:
    """Takes an image of the product from a given camera pose and uploads it."""

    def __init__(self, node: Node, config: ManufacturingExecutorConfigDTO, clients: LoopClients) -> None:
        self._logger = node.get_logger()
        self._config = config
        self._clients = clients

    def capture(
        self,
        pose: PoseStamped,
        *,
        operation_id: str,
        product: ProductDataDTO,
        session: MovementSessionClient,
        ctx: AbortContext,
    ) -> None:
        """PTP to ``pose`` with the camera TCP, wait for the camera to settle, capture and upload.

        :raises SystemFailureError: If any of the steps fails.
        """
        self._move(pose, session, ctx)
        sleep_abortable(self._config.image_capture_settling_s, ctx)
        image = self._get_image(ctx)
        self._upload(image, build_capture_metadata(operation_id, product.serial_number, pose), ctx)

    def _move(self, pose: PoseStamped, session: MovementSessionClient, ctx: AbortContext) -> None:
        session.ensure_valid()
        path = TrajectoryPath()
        path.path_id = str(uuid.uuid4())
        path.motion_type = TrajectoryPath.MOTION_TYPE_PTP
        path.target_pose = copy.deepcopy(pose)
        path.tool_frame = self._config.camera_tcp
        goal = ExecuteTrajectory.Goal()
        goal.paths = [path]
        goal.session_id = session.session_id
        outcome = run_action(
            self._clients.execute_trajectory,
            goal,
            ctx=ctx,
            description='Move to the image capture pose',
            accept_timeout_s=self._config.service_call_timeout_s,
            result_timeout_s=None,
        )
        outcome.ensure_success('Move to the image capture pose')

    def _get_image(self, ctx: AbortContext) -> Image:
        outcome = run_action(
            self._clients.get_image,
            GetImage.Goal(),
            ctx=ctx,
            description='Image capture',
            accept_timeout_s=self._config.service_call_timeout_s,
            result_timeout_s=self._config.image_capture_timeout_s,
        )
        return outcome.ensure_success('Image capture').image

    def _upload(self, image: Image, metadata: str, ctx: AbortContext) -> None:
        request = ImageRepositoryUploadImage.Request()
        request.image = image
        request.image_name = str(uuid.uuid4())
        request.metadata = metadata
        response = call_service(
            self._clients.upload_image,
            request,
            timeout_s=self._config.image_capture_timeout_s,
            ctx=ctx,
            description='Image upload',
        )
        if not response.success:
            raise SystemFailureError(f'Image upload failed: {response.error_message}')
