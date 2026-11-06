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
"""Extraction of link poses and collision geometry from an expanded SDF document."""

import os
import xml.etree.ElementTree as ET

from station_workplace.models.sdf_model_dto import (
    BoxGeometryDTO,
    CylinderGeometryDTO,
    MeshGeometryDTO,
    Pose6,
    SdfCollisionDTO,
    SdfModelDTO,
    SphereGeometryDTO,
)

_MODEL_URI_SCHEME = 'model://'
_ZERO_POSE: Pose6 = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
_UNSUPPORTED_MODEL_CHILDREN = ('model', 'include')


def parse_sdf_model(sdf: str, model_dir: str) -> SdfModelDTO:
    """Parse the single top-level ``<model>`` of an SDF document.

    Only links that are direct children of the model are supported; nested
    ``<model>``/``<include>`` elements are rejected. Link poses must be relative
    to the model frame and collision poses relative to their link. Relative
    ``<uri>`` values are rejected anywhere in the model, because Gazebo cannot
    resolve them for an SDF spawned from a string. Collision mesh URIs must be
    ``model://<name>/<path>`` and are resolved against the parent of ``model_dir``.

    :param sdf: The expanded SDF document.
    :param model_dir: Folder of the model, used to resolve mesh URIs.
    :returns: The model's link poses and collision geometry.
    :raises ValueError: If the document is malformed or uses unsupported features.
    """
    try:
        root = ET.fromstring(sdf)
    except ET.ParseError as exc:
        raise ValueError(f'invalid SDF XML: {exc}') from exc
    if root.tag != 'sdf':
        raise ValueError(f'root element must be <sdf>, got <{root.tag}>')
    models = root.findall('model')
    if len(models) != 1:
        raise ValueError(f'SDF must contain exactly one top-level <model>, found {len(models)}')
    model = models[0]
    model_name = model.get('name', '')
    for tag in _UNSUPPORTED_MODEL_CHILDREN:
        if model.find(tag) is not None:
            raise ValueError(f"model '{model_name}': nested <{tag}> is not supported")
    for uri_element in model.iter('uri'):
        uri = (uri_element.text or '').strip()
        if '://' not in uri:
            raise ValueError(
                f"model '{model_name}': relative uri '{uri}' is not supported "
                '(use model://<model_id>/<path>)')

    link_poses: dict[str, Pose6] = {}
    collisions: list[SdfCollisionDTO] = []
    for link in model.findall('link'):
        link_name = link.get('name', '')
        if not link_name:
            raise ValueError(f"model '{model_name}': <link> without a name")
        if link_name in link_poses:
            raise ValueError(f"model '{model_name}': duplicate link '{link_name}'")
        link_poses[link_name] = _parse_pose(
            link.find('pose'), {'', '__model__', model_name}, f"link '{link_name}'")
        for collision in link.findall('collision'):
            collision_name = collision.get('name', '')
            description = f"collision '{collision_name}' of link '{link_name}'"
            collisions.append(SdfCollisionDTO(
                link_name=link_name,
                name=collision_name,
                pose=_parse_pose(collision.find('pose'), {'', link_name}, description),
                geometry=_parse_geometry(collision.find('geometry'), model_dir, description),
            ))
    return SdfModelDTO(name=model_name, link_poses=link_poses, collisions=tuple(collisions))


def _parse_pose(element: ET.Element | None, allowed_frames: set[str], description: str) -> Pose6:
    """Parse an SDF ``<pose>`` (Euler RPY, radians) relative to one of ``allowed_frames``."""
    if element is None:
        return _ZERO_POSE
    relative_to = element.get('relative_to', '')
    if relative_to not in allowed_frames:
        raise ValueError(f"{description}: pose relative_to='{relative_to}' is not supported")
    if element.get('degrees', 'false').strip().lower() in ('true', '1'):
        raise ValueError(f'{description}: poses in degrees are not supported')
    rotation_format = element.get('rotation_format', 'euler_rpy')
    if rotation_format != 'euler_rpy':
        raise ValueError(f"{description}: rotation_format='{rotation_format}' is not supported")
    values = _parse_floats(element.text, description)
    if not values:
        return _ZERO_POSE
    if len(values) != 6:
        raise ValueError(f'{description}: pose must have 6 values, got {len(values)}')
    return (values[0], values[1], values[2], values[3], values[4], values[5])


def _parse_geometry(element: ET.Element | None, model_dir: str, description: str):
    """Parse the single shape of a ``<geometry>`` element."""
    if element is None:
        raise ValueError(f'{description}: missing <geometry>')
    shapes = list(element)
    if len(shapes) != 1:
        raise ValueError(f'{description}: <geometry> must contain exactly one shape')
    shape = shapes[0]
    try:
        if shape.tag == 'box':
            return BoxGeometryDTO(size=_child_floats(shape, 'size', 3, description))  # type: ignore
        if shape.tag == 'cylinder':
            return CylinderGeometryDTO(
                radius=_child_floats(shape, 'radius', 1, description)[0],
                length=_child_floats(shape, 'length', 1, description)[0])
        if shape.tag == 'sphere':
            return SphereGeometryDTO(radius=_child_floats(shape, 'radius', 1, description)[0])
        if shape.tag == 'mesh':
            return _parse_mesh(shape, model_dir, description)
    except ValueError as exc:
        raise ValueError(f'{description}: {exc}') from exc
    raise ValueError(f'{description}: geometry <{shape.tag}> is not supported')


def _parse_mesh(shape: ET.Element, model_dir: str, description: str) -> MeshGeometryDTO:
    if shape.find('submesh') is not None:
        raise ValueError('<submesh> is not supported')
    uri = (shape.findtext('uri') or '').strip()
    if not uri:
        raise ValueError('mesh without <uri>')
    scale_element = shape.find('scale')
    scale = (1.0, 1.0, 1.0)
    if scale_element is not None:
        values = _parse_floats(scale_element.text, description)
        if len(values) != 3:
            raise ValueError(f'mesh <scale> must have 3 values, got {len(values)}')
        scale = (values[0], values[1], values[2])
    return MeshGeometryDTO(path=resolve_mesh_uri(uri, model_dir), scale=scale)


def resolve_mesh_uri(uri: str, model_dir: str) -> str:
    """Resolve a ``model://<name>/<path>`` mesh URI to an existing local file.

    :param uri: Mesh URI; ``<name>`` is looked up next to ``model_dir``.
    :param model_dir: Folder of the model.
    :returns: The absolute path of the mesh file.
    :raises ValueError: If the URI is not a ``model://`` URI or the file does not exist.
    """
    if not uri.startswith(_MODEL_URI_SCHEME):
        raise ValueError(f"mesh uri '{uri}' is not supported (use model://<model_id>/<path>)")
    relative = uri[len(_MODEL_URI_SCHEME):]
    base = os.path.dirname(os.path.normpath(model_dir))
    path = os.path.normpath(os.path.join(base, relative))
    if not os.path.isfile(path):
        raise ValueError(f"mesh '{uri}' not found at '{path}'")
    return path


def _child_floats(element: ET.Element, tag: str, count: int, description: str) -> list[float]:
    child = element.find(tag)
    if child is None:
        raise ValueError(f'<{element.tag}> without <{tag}>')
    values = _parse_floats(child.text, description)
    if len(values) != count:
        raise ValueError(f'<{tag}> must have {count} value(s), got {len(values)}')
    return values


def _parse_floats(text: str | None, description: str) -> list[float]:
    try:
        return [float(value) for value in (text or '').split()]
    except ValueError as exc:
        raise ValueError(f"{description}: invalid number in '{text}'") from exc


def find_link_pose(model: SdfModelDTO, link_name: str) -> Pose6:
    """Return the pose of ``link_name`` in the model frame.

    :raises ValueError: If the model has no such link.
    """
    pose = model.link_poses.get(link_name)
    if pose is None:
        raise ValueError(f"model '{model.name}' has no link '{link_name}'")
    return pose
