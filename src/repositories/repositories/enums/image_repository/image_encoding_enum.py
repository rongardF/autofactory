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
"""Image encodings supported by the ``image_repository`` node."""

from enum import Enum


class ImageEncodingEnum(str, Enum):
    """``sensor_msgs/Image`` encodings that map losslessly to PNG."""

    MONO8 = 'mono8'
    MONO16 = 'mono16'
    RGB8 = 'rgb8'
    BGR8 = 'bgr8'
    RGBA8 = 'rgba8'
    BGRA8 = 'bgra8'
    RGB16 = 'rgb16'
    BGR16 = 'bgr16'
    RGBA16 = 'rgba16'
    BGRA16 = 'bgra16'

    @property
    def channels(self) -> int:
        """Number of channels per pixel (1, 3 or 4)."""
        prefix = self.value.rstrip('0123456789')
        if prefix == 'mono':
            return 1
        return 4 if prefix.endswith('a') else 3

    @property
    def bytes_per_channel(self) -> int:
        """Number of bytes per channel sample (1 or 2)."""
        return 2 if self.value.endswith('16') else 1

    @property
    def is_rgb_order(self) -> bool:
        """True for RGB(A) channel order, which needs a swap to OpenCV's BGR(A)."""
        return self.value.startswith('rgb')
