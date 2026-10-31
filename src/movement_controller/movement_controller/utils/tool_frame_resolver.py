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
"""ToolFrameResolver — resolve a TF tool frame to the robot-model link it hangs off.

MoveIt goal/path constraints must reference a link in the robot's kinematic
model (the URDF/SRDF compiled into the ``RobotModel``). A tool frame that only
exists on the TF tree (e.g. a calibrated TCP broadcast on ``/tf_static`` by an
end-tool node) is invisible to the planner and is rejected with
``Link '<frame>' not found in model``.

This resolver mirrors what MoveIt's ``ResolveConstraintFrames`` adapter does for
collision-object subframes, but sources the rigid offset from TF so it works for
any static frame: given a target pose expressed for a TF frame ``F``, it finds
the nearest URDF link ``L`` that ``F`` is rigidly attached to and rewrites the
target so the same physical goal is expressed for ``L`` instead.
"""

from __future__ import annotations

import xml.etree.ElementTree as ElementTree

import yaml

from rclpy.duration import Duration
from rclpy.lifecycle.node import LifecycleNode
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from rclpy.time import Time

from geometry_msgs.msg import Point, Pose, PoseStamped, TransformStamped
from std_msgs.msg import String
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_geometry_msgs import do_transform_pose

from movement_controller.enums import MotionTypeEnum
from movement_controller.exceptions import FrameResolutionError
from movement_controller.models import TrajectoryPathDTO


class ToolFrameResolver:
    """Resolve TF tool frames to robot-model links via rigid TF offsets.

    The resolver listens to ``/robot_description`` (latched) to learn the set of
    URDF link names, and to ``/tf`` + ``/tf_static`` through a
    :class:`tf2_ros.buffer.Buffer` to look up rigid offsets. It is created with
    the parent node and activated/deactivated alongside the planner so its
    subscriptions follow the node lifecycle.
    """

    # Maximum time to wait for a rigid tool-frame -> link transform to appear in
    # the TF buffer. Static transforms are latched, so this only covers startup
    # races between the end-tool broadcaster and the first motion request.
    _TF_LOOKUP_TIMEOUT_S: float = 2.0

    def __init__(self, node: LifecycleNode) -> None:
        """Store the node and prepare empty subscription/buffer references.

        :param node: Parent lifecycle node used to create subscriptions and the
            TF listener, and to access the logger and clock.
        :type node: LifecycleNode
        """
        self._node = node
        self._logger = node.get_logger()
        self._buffer: Buffer | None = None
        self._listener: TransformListener | None = None
        self._robot_description_sub = None
        self._link_names: set[str] = set()

    # region: lifecycle
    def on_activate(self) -> None:
        """Create the TF listener and subscribe to the latched robot description."""
        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self._node)
        self._link_names = set()
        self._robot_description_sub = self._node.create_subscription(
            String,
            '/robot_description',
            self._on_robot_description,
            QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
            ),
        )
        self._logger.debug('ToolFrameResolver activated')

    def on_deactivate(self) -> None:
        """Tear down the TF listener and robot-description subscription."""
        if self._robot_description_sub is not None:
            self._node.destroy_subscription(self._robot_description_sub)
            self._robot_description_sub = None
        if self._listener is not None:
            # TransformListener holds /tf and /tf_static subscriptions on the
            # node; destroy them explicitly so a later re-activation does not
            # leak duplicate subscriptions.
            for attr in ('tf_sub', 'tf_static_sub'):
                sub = getattr(self._listener, attr, None)
                if sub is not None:
                    self._node.destroy_subscription(sub)
            self._listener = None
        self._buffer = None
        self._link_names = set()
        self._logger.debug('ToolFrameResolver deactivated')

    # endregion: lifecycle

    # region: callbacks
    def _on_robot_description(self, msg: String) -> None:
        """Parse the URDF and cache the set of link names for frame classification."""
        try:
            root = ElementTree.fromstring(msg.data)
            self._link_names = {
                link.get('name')
                for link in root.iter('link')
                if link.get('name')
            }
            self._logger.debug(
                f'ToolFrameResolver cached {len(self._link_names)} robot-model link names'
            )
        except ElementTree.ParseError as exc:
            self._logger.error(f'Failed to parse /robot_description URDF: {exc}')

    # endregion: callbacks

    # region: public methods
    def resolve(self, path: TrajectoryPathDTO) -> TrajectoryPathDTO:
        """Return a path whose ``tool_frame`` is a robot-model link.

        If ``tool_frame`` is empty or already a robot-model link the path is
        returned unchanged. Otherwise the frame is treated as a TF frame: the
        nearest URDF link it is rigidly attached to is found, and the target
        pose is recomposed with the rigid offset so the same physical goal is
        expressed for that link.

        :param path: The path to resolve.
        :type path: TrajectoryPathDTO
        :returns: The original path, or a copy with ``tool_frame`` and
            ``target_pose`` rewritten to reference a robot-model link.
        :rtype: TrajectoryPathDTO
        :raises FrameResolutionError: If the frame is unknown to both the robot
            model and TF, cannot be traced to a URDF link, or its rigid offset
            cannot be looked up.
        """
        frame = path.tool_frame
        if not frame or frame in self._link_names:
            return path

        if not self._link_names:
            raise FrameResolutionError(
                f"cannot resolve tool_frame '{frame}': robot model link names are not "
                "available yet (no /robot_description received)"
            )

        link = self._find_parent_link(frame)
        if link is None:
            raise FrameResolutionError(
                f"tool_frame '{frame}' is not a robot-model link and could not be "
                "traced through TF to one"
            )

        if self._buffer is None:
            raise FrameResolutionError(
                f"cannot resolve tool_frame '{frame}': TF buffer is not active"
            )

        try:
            # pose of the link expressed in the tool frame (T_F_L)
            offset = self._buffer.lookup_transform(
                frame,
                link,
                Time(),
                timeout=Duration(seconds=self._TF_LOOKUP_TIMEOUT_S),
            )
        except TransformException as exc:
            raise FrameResolutionError(
                f"failed to look up transform from '{frame}' to link '{link}': {exc}"
            ) from exc

        resolved_pose = self._compose(path.target_pose, offset)
        update: dict[str, object] = {'tool_frame': link, 'target_pose': resolved_pose}

        if path.motion_type == MotionTypeEnum.CIRC:
            update['circ_point'] = self._compose_point(
                path.circ_point, path.target_pose, offset
            )

        self._logger.info(
            f"Resolved tool_frame '{frame}' to robot-model link '{link}' for path {path.path_id}"
        )
        return path.model_copy(update=update)

    # endregion: public methods

    # region: private helpers
    def _find_parent_link(self, frame: str) -> str | None:
        """Walk the TF parent chain from ``frame`` to the nearest URDF link.

        :param frame: The TF frame to start from.
        :type frame: str
        :returns: The nearest ancestor frame that is a robot-model link, or
            ``None`` if the chain reaches a root without hitting one.
        :rtype: str | None
        """
        if self._buffer is None:
            return None
        try:
            frames = yaml.safe_load(self._buffer.all_frames_as_yaml()) or {}
        except yaml.YAMLError as exc:
            self._logger.error(f'Failed to parse TF frame tree: {exc}')
            return None

        current = frame
        seen: set[str] = set()
        while current not in self._link_names:
            seen.add(current)
            info = frames.get(current)
            if not isinstance(info, dict) or 'parent' not in info:
                return None
            parent = info['parent']
            if parent in seen:
                return None
            current = parent
        return current

    @staticmethod
    def _compose(target: PoseStamped, offset: TransformStamped) -> PoseStamped:
        """Recompose ``target`` (a pose for the tool frame) as a pose for the link.

        With ``T_H_F`` the target pose (tool frame ``F`` expressed in header
        frame ``H``) and ``T_F_L`` the rigid offset (link ``L`` expressed in
        ``F``, i.e. ``offset``), the link pose in ``H`` is
        ``T_H_L = T_H_F . T_F_L``. ``do_transform_pose(pose, transform)``
        computes ``transform . pose``, so the target becomes the transform and
        the offset becomes the pose.

        :param target: Target pose expressed for the tool frame.
        :type target: PoseStamped
        :param offset: Rigid transform giving the link's pose in the tool frame.
        :type offset: TransformStamped
        :returns: The equivalent target pose expressed for the link, in the same
            header frame as ``target``.
        :rtype: PoseStamped
        """
        offset_pose = Pose()
        offset_pose.position.x = offset.transform.translation.x
        offset_pose.position.y = offset.transform.translation.y
        offset_pose.position.z = offset.transform.translation.z
        offset_pose.orientation = offset.transform.rotation

        target_as_transform = TransformStamped()
        target_as_transform.transform.translation.x = target.pose.position.x
        target_as_transform.transform.translation.y = target.pose.position.y
        target_as_transform.transform.translation.z = target.pose.position.z
        target_as_transform.transform.rotation = target.pose.orientation

        resolved = PoseStamped()
        resolved.header.frame_id = target.header.frame_id
        resolved.header.stamp = target.header.stamp
        resolved.pose = do_transform_pose(offset_pose, target_as_transform)
        return resolved

    @classmethod
    def _compose_point(
        cls, point: Point, target: PoseStamped, offset: TransformStamped
    ) -> Point:
        """Re-express a CIRC arc reference point from the tool frame to the link.

        ``circ_point`` is a bare position describing where the *tool frame*
        passes (``interim``) or the centre the tool frame orbits (``center``).
        Once ``target_pose`` is recomposed for the link, the arc point must be
        moved by the same rigid offset, otherwise PILZ builds its circle from
        two link poses and one tool-frame point and plans a wildly wrong arc.

        PILZ holds orientation fixed (or slerps between identical start/goal
        orientations) along a CIRC, so the tool-to-link offset is the same
        constant translation at the arc point as it is at the goal; applying
        the goal orientation to the arc point reproduces it exactly.

        :param point: Arc reference point expressed for the tool frame.
        :type point: Point
        :param target: Original (unresolved) target pose, supplying the
            orientation under which the offset is applied.
        :type target: PoseStamped
        :param offset: Rigid transform giving the link's pose in the tool frame.
        :type offset: TransformStamped
        :returns: The arc reference point expressed for the link.
        :rtype: Point
        """
        point_as_pose = PoseStamped()
        point_as_pose.header = target.header
        point_as_pose.pose.position = point
        point_as_pose.pose.orientation = target.pose.orientation
        return cls._compose(point_as_pose, offset).pose.position

    # endregion: private helpers
