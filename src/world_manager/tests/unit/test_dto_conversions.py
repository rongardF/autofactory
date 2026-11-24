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
"""Round-trip the DTO conversions used at the ROS boundaries."""

from shape_msgs.msg import SolidPrimitive

from world_manager.enums import JointStateEnum
from world_manager.models import (
    AllowedCollisionPairDTO,
    JointInfoDTO,
    ModelInfoDTO,
    ModelSpecDTO,
    ObjectInfoDTO,
    PoseDTO,
    SceneObjectDTO,
    SolidPrimitiveDTO,
    WorldChangesDTO,
)


def test_gazebo_dto_round_trips():
    """Model specs and nested model states retain every field."""
    pose = PoseDTO(
        frame_id='mount', x=1.2, y=2.3, z=3.4,
        qx=0.1, qy=0.2, qz=0.3, qw=0.9,
    )
    spec = ModelSpecDTO(
        model_id='tool', sdf='<sdf/>', pose=pose, attach_link='mount', delete_with_child=True
    )
    model = ModelInfoDTO(
        model_id='tool', owner='test',
        joints=(JointInfoDTO(
            link_name='mount', child_model='robot',
            state=JointStateEnum.ATTACHED,
            state_confirmed=True, controllable=False,
        ),),
    )
    assert ModelSpecDTO.from_msg(ModelSpecDTO.to_msg(spec)) == spec
    assert ModelInfoDTO.from_msg(ModelInfoDTO.to_msg(model)) == model
    assert PoseDTO.from_stamped_msg(PoseDTO.to_stamped_msg(pose)) == pose
    gazebo_pose = PoseDTO.to_gazebo_msg(pose)
    assert (
        gazebo_pose.position.x, gazebo_pose.position.y, gazebo_pose.position.z
    ) == (1.2, 2.3, 3.4)
    assert (
        gazebo_pose.orientation.x, gazebo_pose.orientation.y,
        gazebo_pose.orientation.z, gazebo_pose.orientation.w,
    ) == (0.1, 0.2, 0.3, 0.9)


def test_moveit_dto_round_trips():
    """Geometry, frames, touch links and ACM changes survive conversion."""
    obj = SceneObjectDTO(
        object_id='box',
        primitives=(SolidPrimitiveDTO(
            type=SolidPrimitive.BOX, dimensions=(1.0, 2.0, 3.0),
        ),),
        primitive_poses=(PoseDTO(x=0.25, qz=0.5, qw=0.5),),
        pose=PoseDTO(frame_id='tool0', z=0.5),
        attach_link='tool0', touch_links=('finger',),
    )
    changes = WorldChangesDTO(
        add_or_replace=(obj,), remove_ids=('old',),
        acm_changes=(AllowedCollisionPairDTO(
            name_a='box', name_b='finger', allowed=True,
        ),),
    )
    info = ObjectInfoDTO(object_id='box', attach_link='tool0')
    assert WorldChangesDTO.from_msg(WorldChangesDTO.to_msg(changes)) == changes
    assert ObjectInfoDTO.from_msg(ObjectInfoDTO.to_msg(info)) == info
