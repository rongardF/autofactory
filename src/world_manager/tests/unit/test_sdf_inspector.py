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
"""Unit tests for SdfInspector."""

from pytest import raises

from world_manager.exceptions import ModelValidationError
from world_manager.services.sdf_inspector import SdfInspector
from conftest import make_sdf


def test_model_without_joints_has_none():
    """A plain model yields its name and no joints."""
    result = SdfInspector().inspect(make_sdf('box'))
    assert result.model_name == 'box'
    assert result.joints == ()


def test_single_joint_with_explicit_topics():
    """Explicit attach/detach/output topics are taken as authored."""
    result = SdfInspector().inspect(make_sdf('tool', [('station', 'mount', '/a', '/d', '/s')]))
    joint = result.joints[0]
    assert (joint.link_name, joint.child_model) == ('mount', 'station')
    assert (joint.attach_topic, joint.detach_topic, joint.state_topic) == ('/a', '/d', '/s')


def test_missing_topics_resolve_to_gz_sim_defaults():
    """Defaults: attach/detach keyed on this model, state keyed on the child model."""
    joint = SdfInspector().inspect(make_sdf('pallet', [('conveyor', 'slot')])).joints[0]
    assert joint.attach_topic == '/model/pallet/detachable_joint/attach'
    assert joint.detach_topic == '/model/pallet/detachable_joint/detach'
    assert joint.state_topic == '/model/conveyor/detachable_joint/state'


def test_two_joints_are_both_returned():
    """A model with two DetachableJoint plugins reports both."""
    result = SdfInspector().inspect(make_sdf('tool', [('station', 'a'), ('station', 'b')]))
    assert [j.link_name for j in result.joints] == ['a', 'b']


def test_duplicate_child_link_is_rejected():
    """Two joints on the same child link are ambiguous and rejected."""
    with raises(ModelValidationError, match='child_link'):
        SdfInspector().inspect(make_sdf('tool', [('station', 'a'), ('station', 'a')]))


def test_invalid_xml_is_rejected():
    """Unparseable SDF raises a validation error rather than an XML error."""
    with raises(ModelValidationError, match='does not parse'):
        SdfInspector().inspect('<sdf><model')


def test_sdf_with_two_models_is_rejected():
    """Exactly one <model> is allowed."""
    with raises(ModelValidationError, match='exactly one'):
        SdfInspector().inspect('<sdf><model name="a"/><model name="b"/></sdf>')


def test_joint_without_child_link_is_rejected():
    """A plugin lacking child_link cannot be addressed."""
    sdf = ('<sdf><model name="t"><plugin name="gz::sim::systems::DetachableJoint">'
           '<child_model>m</child_model></plugin></model></sdf>')
    with raises(ModelValidationError, match='child_link'):
        SdfInspector().inspect(sdf)
