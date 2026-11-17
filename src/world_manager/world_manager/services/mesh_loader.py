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
# INTERRUPTION) HOWEVER CAUSED BY ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
"""Loads a triangle mesh and converts it to a shape_msgs/Mesh."""

from pathlib import Path

import trimesh
from geometry_msgs.msg import Point
from shape_msgs.msg import Mesh, MeshTriangle

from world_manager.exceptions import MeshLoadError


def load_mesh(mesh_path: str, file_type: str = 'stl') -> Mesh:
    """Load a mesh file (units are meters) or raise MeshLoadError.

    The explicit file type supports cached mesh files with no extension.
    """
    path = Path(mesh_path)
    if not path.is_absolute():
        raise MeshLoadError(f"mesh path '{mesh_path}' must be absolute")
    if not file_type:
        raise MeshLoadError(f"mesh file type is empty for '{mesh_path}'")
    try:
        loaded = trimesh.load(str(path), file_type=file_type, force='mesh')
    except Exception as e:
        raise MeshLoadError(f"failed to load mesh '{mesh_path}': {e}") from e
    if not isinstance(loaded, trimesh.Trimesh) or len(loaded.faces) == 0:
        raise MeshLoadError(f"'{mesh_path}' does not contain a triangle mesh")
    mesh = Mesh()
    mesh.vertices = [
        Point(x=float(vertex[0]), y=float(vertex[1]), z=float(vertex[2]))
        for vertex in loaded.vertices
    ]
    mesh.triangles = [
        MeshTriangle(vertex_indices=[int(face[0]), int(face[1]), int(face[2])])
        for face in loaded.faces
    ]
    return mesh
