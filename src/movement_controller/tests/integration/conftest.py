# Copyright (c) 2026, Movement Controller Contributors
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
"""Integration test conftest — restores real moveit_msgs.msg.Constraints.

The root conftest.py replaces ``moveit_msgs.msg.Constraints`` with a lightweight
stub (``_ConstraintsStub``) to keep unit tests self-contained.  Integration
tests create real ROS 2 services that require the genuine type-support metadata
on ``Constraints``; without it ROS 2 raises ``AttributeError: type object 'type'
has no attribute '_TYPE_SUPPORT'``.

This conftest runs ``pytest_configure`` AFTER the root conftest so it can
unconditionally restore the real class and fix library paths.

Fix 2 — LD_LIBRARY_PATH for generated C type-support libraries
The CMakeLists uses ``SKIP_INSTALL`` for ``rosidl_generate_interfaces`` which
leaves the generated C shared libraries only in the colcon ``build/`` directory.
The integration tests create real ``ActionServer`` / ``ActionClient`` instances
whose ``dlopen`` calls need these libraries.  Prepending the build directory to
``LD_LIBRARY_PATH`` before the first node creation makes them findable.
"""

import os
import sys
from unittest.mock import MagicMock


def pytest_configure(config):  # noqa: ARG001
    """Fix LD_LIBRARY_PATH and restore the real Constraints class."""
    # ------------------------------------------------------------------
    # 1. Ensure generated C type-support libraries can be loaded
    #
    # The CMakeLists uses SKIP_INSTALL so the generated shared libraries
    # live only in the colcon build/ directory.  Setting LD_LIBRARY_PATH
    # at this stage does NOT help (the dynamic linker already caches the
    # path list at process start).  Instead we pre-load every
    # libmovement_controller__*.so by absolute path with RTLD_GLOBAL so
    # their symbols are available globally when the ROS2 type-support
    # dispatcher later resolves the same library by short name.
    # ------------------------------------------------------------------
    import ctypes

    build_lib = os.path.normpath(
        os.path.join(
            os.path.dirname(__file__),  # tests/integration/
            '..', '..', '..', '..', 'build', 'movement_controller',
        )
    )
    if os.path.isdir(build_lib):
        for fname in os.listdir(build_lib):
            if fname.startswith('libmovement_controller') and fname.endswith('.so'):
                try:
                    ctypes.CDLL(os.path.join(build_lib, fname), ctypes.RTLD_GLOBAL)
                except OSError:
                    pass  # best-effort; missing library will surface as a clear error later

    # ------------------------------------------------------------------
    # 2. Rebuild Pydantic models that use TYPE_CHECKING-only forward refs
    #
    # PlanningSessionDTO and PlanResultDTO reference moveit_msgs types
    # only inside TYPE_CHECKING blocks.  Pydantic v2 cannot resolve those
    # references at runtime without an explicit model_rebuild() call.
    # Using ``object`` as the namespace value is valid: any real
    # MotionSequenceResponse / RobotState instance is a subclass of object,
    # so validators accept them without needing the precise type.
    # (Unit-test files perform the same rebuild at module level; we do it
    # here in pytest_configure so integration tests are covered even when
    # the unit-test modules have not yet been imported.)
    # ------------------------------------------------------------------
    try:
        from movement_controller.models.plan_result_dto import PlanResultDTO
        from movement_controller.models.planning_session_dto import PlanningSessionDTO
        PlanResultDTO.model_rebuild(_types_namespace={'MotionSequenceResponse': MagicMock})
        PlanningSessionDTO.model_rebuild(_types_namespace={'RobotState': MagicMock})
    except Exception:
        pass  # best-effort; a clear error will surface during test execution if needed

    # ------------------------------------------------------------------
    # 3. Restore the real Constraints class for ROS 2 service type support
    # ------------------------------------------------------------------
    if 'moveit_msgs.msg' not in sys.modules:
        return
    try:
        from moveit_msgs.msg._constraints import Constraints as _RealConstraints
        sys.modules['moveit_msgs.msg'].Constraints = _RealConstraints  # type: ignore
    except ImportError:
        pass  # real moveit_msgs not installed — integration tests will skip naturally
