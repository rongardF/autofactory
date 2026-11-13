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

"""Lifecycle node that groups dispensing patterns for shared image capture poses."""

from __future__ import annotations

from collections.abc import Sequence

from geometry_msgs.msg import Point
from rcl_interfaces.msg import ParameterDescriptor
from rclpy import init, ok, shutdown
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.service import Service

from execution_optimizer.interface.optimizer_base import OptimizerBase
from execution_optimizer.model.camera_intrinsics_dto import CameraIntrinsicsDTO
from execution_optimizer.model.dispensing_optimizer_config_dto import (
    DispensingOptimizerConfigDTO,
)
from execution_optimizer.model.pattern_points_dto import PatternPointsDTO
from execution_optimizer.model.vec3 import Vec3
from execution_optimizer.service.dispensing_grouping_service import DispensingGroupingService
from execution_optimizer.srv import DispensingOptimizePatterns


def _point(point: Point) -> Vec3:
    return (float(point.x), float(point.y), float(point.z))


class DispensingOptimizer(OptimizerBase):
    """Groups dispensing patterns so each group is imaged from a single camera pose.

    Interface (node private, exists only while the node is active):

    * ``~/optimize_patterns`` (``execution_optimizer/srv/DispensingOptimizePatterns``)
    """

    def __init__(self, node_name: str = 'dispensing_optimizer') -> None:
        """Initialise the node and declare the dispensing specific parameters.

        :param node_name: ROS 2 node name passed to :class:`LifecycleNode`.
        :type node_name: str
        """
        super().__init__(node_name)

        self.declare_parameter(
            'feature_resolution_range',
            [3.33, 4.16],
            ParameterDescriptor(
                description='Inclusive [low, high] feature resolution range in px/mm.'
            ),
        )
        self.declare_parameter(
            'max_angle_deviation',
            20.0,
            ParameterDescriptor(
                description='Maximum angle in degrees between any ROI normal of a group and '
                'the mean normal of that group.'
            ),
        )
        # An empty list is inferred as a byte array, so the type must stay dynamic to
        # accept both [] and an integer array.
        self.declare_parameter(
            'operational_area',
            [],
            ParameterDescriptor(
                description='Empty (whole image) or inclusive pixel rectangle '
                '[x_min, y_min, x_max, y_max] with a top-left origin.',
                dynamic_typing=True,
            ),
        )
        self.declare_parameter(
            'planarity_tolerance_mm',
            0.5,
            ParameterDescriptor(
                description='Maximum distance in millimetres of pattern points from the '
                'fitted plane or line.'
            ),
        )

    # region: subclass hooks
    def _build_config(self) -> DispensingOptimizerConfigDTO:
        area = list(self.get_parameter('operational_area').value or [])
        return DispensingOptimizerConfigDTO(
            **self._read_common_kwargs(),
            feature_resolution_range=tuple(self.get_parameter('feature_resolution_range').value),
            max_angle_deviation=self.get_parameter('max_angle_deviation').value,
            operational_area=tuple(area) if area else None,
            planarity_tolerance_mm=self.get_parameter('planarity_tolerance_mm').value,
        )

    def _create_optimizer(self, config: DispensingOptimizerConfigDTO) -> DispensingGroupingService:
        return DispensingGroupingService(config)

    def _create_interface(self) -> Service:
        return self.create_service(
            DispensingOptimizePatterns,
            '~/optimize_patterns',
            self._handle_optimize,
            callback_group=self._service_callback_group,
        )

    def _validate_intrinsics(self, intrinsics: CameraIntrinsicsDTO) -> None:
        config = self._config
        if not isinstance(config, DispensingOptimizerConfigDTO) or config.operational_area is None:
            return
        _, _, x_max, y_max = config.operational_area
        if x_max > intrinsics.width or y_max > intrinsics.height:
            raise ValueError(
                f'operational_area {list(config.operational_area)} exceeds the image size '
                f'{intrinsics.width}x{intrinsics.height}'
            )

    def _to_pattern_points(self, patterns: Sequence) -> list[PatternPointsDTO]:
        """Collect start, target and circular points of every pattern.

        Arcs are not sampled; the interim or centre point of a circular movement is
        expressed in the frame of that movement's target pose.
        """
        result: list[PatternPointsDTO] = []
        for pattern in patterns:
            frame_ids = [pattern.start_pose.header.frame_id]
            points = [_point(pattern.start_pose.pose.position)]
            for movement in pattern.movements:
                frame_ids.append(movement.target_pose.header.frame_id)
                points.append(_point(movement.target_pose.pose.position))
                points.extend(_point(point) for point in movement.circ_point)
            result.append(PatternPointsDTO(
                id=pattern.id, frame_ids=tuple(frame_ids), points=tuple(points)
            ))
        return result
    # endregion: subclass hooks


def main(args: list[str] | None = None) -> None:
    """Entry point for the ``dispensing_optimizer`` executable.

    The node is spun by a :class:`MultiThreadedExecutor` so the camera action callbacks
    run while the activation transition waits for the camera.

    :param args: Optional command-line arguments forwarded to :func:`rclpy.init`.
    :type args: list[str] | None
    """
    init(args=args)
    node = DispensingOptimizer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if ok():
            shutdown()


if __name__ == '__main__':
    main()
