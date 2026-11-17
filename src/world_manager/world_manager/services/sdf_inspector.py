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
"""Finds DetachableJoint plugins in an SDF document and resolves their topics."""

from xml.etree.ElementTree import Element, ParseError, fromstring

from world_manager.exceptions import ModelValidationError
from world_manager.models import DetachableJointDTO, SdfModelDTO

_PLUGIN_NAME = 'gz::sim::systems::DetachableJoint'


def _topic(value: str) -> str:
    """Normalize a gz topic to its absolute form."""
    value = value.strip()
    return value if value.startswith('/') else f'/{value}'


def _text(plugin: Element, tag: str) -> str:
    """Stripped text of a direct child tag, empty if missing."""
    child = plugin.find(tag)
    return (child.text or '').strip() if child is not None else ''


class SdfInspector:
    """Parses an SDF string and reproduces gz-sim 8's DetachableJoint topic defaults."""

    def inspect(self, sdf: str) -> SdfModelDTO:
        """Return the model name and its DetachableJoint plugins.

        Raises ModelValidationError if the SDF does not parse, does not contain exactly one
        model, or declares two joints with the same child link.
        """
        try:
            root = fromstring(sdf)
        except ParseError as e:
            raise ModelValidationError(f'SDF does not parse: {e}') from e

        models = root.findall('model') if root.tag == 'sdf' else []
        if len(models) != 1:
            raise ModelValidationError('SDF must contain exactly one <model> under <sdf>')
        model = models[0]
        model_name = model.get('name', '').strip()
        if not model_name:
            raise ModelValidationError('SDF <model> has no name')

        joints: list[DetachableJointDTO] = []
        seen_links: set[str] = set()
        for plugin in model.findall('plugin'):
            if plugin.get('name') != _PLUGIN_NAME:
                continue
            child_model = _text(plugin, 'child_model')
            child_link = _text(plugin, 'child_link')
            if not child_model or not child_link:
                raise ModelValidationError(
                    f"DetachableJoint in model '{model_name}' needs <child_model> and <child_link>")
            if child_link in seen_links:
                raise ModelValidationError(
                    f"model '{model_name}' has two DetachableJoints with child_link '{child_link}'")
            seen_links.add(child_link)
            joints.append(DetachableJointDTO(
                link_name=child_link,
                child_model=child_model,
                attach_topic=_topic(_text(plugin, 'attach_topic') or f'/model/{model_name}/detachable_joint/attach'),
                detach_topic=_topic(_text(plugin, 'detach_topic') or f'/model/{model_name}/detachable_joint/detach'),
                state_topic=_topic(_text(plugin, 'output_topic') or f'/model/{child_model}/detachable_joint/state'),
            ))
        return SdfModelDTO(model_name=model_name, joints=tuple(joints))
