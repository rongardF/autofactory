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
"""Constants of the localization package."""

LOCALIZATION_ORIGIN_FRAME = 'localization_origin'
PRODUCT_ORIGIN_FRAME = 'product_origin'
ESTIMATED_PRODUCT_ORIGIN_FRAME = 'estimated_product_origin'

# Frame the movement controller plans in; target poses are transformed to it.
PLANNING_FRAME = 'base_link'

# NOTE: the real localizer derives this pose from the initial image; the mock uses a
# fixed estimate of the product centre [x, y, z, roll, pitch, yaw] in the base frame.
LOCALIZATION_ORIGIN_PLACEHOLDER_POSE = (1.0, 0.0, 0.96, 0.0, 0.0, 0.0)

# Duration of the mocked localization calculation.
MOCK_CALCULATION_DURATION_S = 20.0

TF_PUBLISH_PERIOD_S = 0.1
SERVER_WAIT_TIMEOUT_S = 5.0
MOVE_TIMEOUT_S = 120.0
TF_LOOKUP_TIMEOUT_S = 2.0

ROS_NAME_PATTERN = r'^[A-Za-z_][A-Za-z0-9_]*$'
# A node name optionally prefixed by its namespace, e.g. 'cameras/basler_camera'.
ROS_NODE_PATH_PATTERN = r'^[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*$'
