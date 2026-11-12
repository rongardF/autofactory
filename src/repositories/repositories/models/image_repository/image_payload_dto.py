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
"""Image payload models exchanged between the node, manager, store and backend."""

import cv2
import numpy as np
from pydantic import BaseModel, Field, JsonValue
from sensor_msgs.msg import Image

from repositories.enums.image_repository.image_encoding_enum import ImageEncodingEnum
from repositories.utils.json_metadata import parse_metadata

_RGB_TO_BGR = {3: cv2.COLOR_RGB2BGR, 4: cv2.COLOR_RGBA2BGRA}
_BGR_TO_RGB = {3: cv2.COLOR_BGR2RGB, 4: cv2.COLOR_BGRA2RGBA}


def _sample_dtype(encoding: ImageEncodingEnum, big_endian: bool) -> np.dtype:
    """Return the numpy dtype of one channel sample as laid out in the message."""
    if encoding.bytes_per_channel == 1:
        return np.dtype(np.uint8)
    return np.dtype('>u2' if big_endian else '<u2')


class ImageInfoDTO(BaseModel, frozen=True):
    """``sensor_msgs/Image`` fields that a PNG file cannot hold."""

    encoding: ImageEncodingEnum = Field(description='Original image encoding.')
    width: int = Field(gt=0, description='Image width in pixels.')
    height: int = Field(gt=0, description='Image height in pixels.')
    is_bigendian: int = Field(
        default=0, ge=0, le=1, description='1 if 16-bit samples were big-endian.'
    )
    frame_id: str = Field(default='', description='Header frame id.')
    stamp_sec: int = Field(default=0, description='Header stamp seconds.')
    stamp_nanosec: int = Field(default=0, description='Header stamp nanoseconds.')


class _MetadataFileDTO(BaseModel, frozen=True):
    """Layout of the ``<image_id>_metadata.json`` file."""

    image_name: str = Field(default='', description='User supplied image name.')
    metadata: JsonValue = Field(default=None, description='User supplied JSON value.')
    image: ImageInfoDTO = Field(description='Image fields needed for reconstruction.')


class ImagePayloadDTO(BaseModel, frozen=True):
    """An image as PNG bytes plus everything needed to rebuild the message."""

    png_bytes: bytes = Field(description='Image encoded as PNG.')
    image_name: str = Field(default='', description='User supplied image name.')
    metadata: JsonValue = Field(
        default=None, description='Parsed user metadata; None means no metadata.'
    )
    info: ImageInfoDTO = Field(description='Image fields not stored in the PNG.')

    def metadata_file_bytes(self) -> bytes:
        """Serialize the metadata file content.

        Returns:
            bytes: UTF-8 JSON with ``image_name``, ``metadata`` and ``image``.
        """
        return _MetadataFileDTO(
            image_name=self.image_name, metadata=self.metadata, image=self.info
        ).model_dump_json().encode('utf-8')

    @property
    def size_bytes(self) -> int:
        """Combined size in bytes of the image file and the metadata file."""
        return len(self.png_bytes) + len(self.metadata_file_bytes())

    @classmethod
    def from_files(cls, png_bytes: bytes, metadata_file_bytes: bytes) -> 'ImagePayloadDTO':
        """Rebuild a payload from the stored file contents.

        Args:
            png_bytes: Content of the image file.
            metadata_file_bytes: Content of the metadata file.

        Returns:
            ImagePayloadDTO: The reconstructed payload.

        Raises:
            ValueError: If the metadata file is not valid.
        """
        stored = _MetadataFileDTO.model_validate_json(metadata_file_bytes)
        return cls(
            png_bytes=png_bytes,
            image_name=stored.image_name,
            metadata=stored.metadata,
            info=stored.image,
        )

    @classmethod
    def from_image_msg(
        cls, msg: Image, image_name: str, metadata: str
    ) -> 'ImagePayloadDTO':
        """Validate an image message and encode it as a PNG payload.

        Args:
            msg: The image to encode.
            image_name: User supplied image name.
            metadata: Raw JSON metadata string; empty means no metadata.

        Returns:
            ImagePayloadDTO: The PNG bytes together with the fields PNG cannot hold.

        Raises:
            ValueError: If the encoding is unsupported, the geometry is
                inconsistent, the metadata is not valid JSON, or PNG encoding fails.
        """
        try:
            encoding = ImageEncodingEnum(msg.encoding)
        except ValueError as exc:
            supported = ', '.join(member.value for member in ImageEncodingEnum)
            raise ValueError(
                f"unsupported image encoding '{msg.encoding}' (supported: {supported})"
            ) from exc

        width, height, step = int(msg.width), int(msg.height), int(msg.step)
        channels = encoding.channels
        row_bytes = width * channels * encoding.bytes_per_channel
        if width <= 0 or height <= 0:
            raise ValueError(f'image size must be positive (got {width}x{height})')
        if step < row_bytes:
            raise ValueError(f'image step {step} is smaller than the row size {row_bytes}')
        raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        if raw.size < step * height:
            raise ValueError(
                f'image data has {raw.size} bytes, expected at least {step * height}'
            )

        big_endian = bool(msg.is_bigendian)
        rows = raw[: step * height].reshape(height, step)[:, :row_bytes]
        samples = np.ascontiguousarray(rows).view(_sample_dtype(encoding, big_endian))
        if encoding.bytes_per_channel == 2:
            samples = samples.astype(np.uint16)
        pixels = samples.reshape(height, width, channels) if channels > 1 else samples
        if encoding.is_rgb_order:
            pixels = cv2.cvtColor(pixels, _RGB_TO_BGR[channels])

        ok, encoded = cv2.imencode('.png', pixels)
        if not ok:
            raise ValueError('PNG encoding failed')

        return cls(
            png_bytes=encoded.tobytes(),
            image_name=image_name,
            metadata=parse_metadata(metadata),
            info=ImageInfoDTO(
                encoding=encoding,
                width=width,
                height=height,
                is_bigendian=int(big_endian),
                frame_id=msg.header.frame_id,
                stamp_sec=msg.header.stamp.sec,
                stamp_nanosec=msg.header.stamp.nanosec,
            ),
        )

    def to_image_msg(self) -> Image:
        """Rebuild a ``sensor_msgs/Image`` from this payload.

        Returns:
            Image: The reconstructed message.

        Raises:
            ValueError: If the PNG is corrupt or does not match the stored fields.
        """
        info = self.info
        encoding = info.encoding
        channels = encoding.channels

        pixels = cv2.imdecode(
            np.frombuffer(self.png_bytes, dtype=np.uint8), cv2.IMREAD_UNCHANGED
        )
        if pixels is None:
            raise ValueError('stored image is corrupt')

        expected_dtype = np.uint8 if encoding.bytes_per_channel == 1 else np.uint16
        actual_channels = 1 if pixels.ndim == 2 else pixels.shape[2]
        if (
            pixels.shape[:2] != (info.height, info.width)
            or actual_channels != channels
            or pixels.dtype != expected_dtype
        ):
            raise ValueError('stored image does not match its metadata')

        if encoding.is_rgb_order:
            pixels = cv2.cvtColor(pixels, _BGR_TO_RGB[channels])
        if encoding.bytes_per_channel == 2:
            pixels = pixels.astype(_sample_dtype(encoding, bool(info.is_bigendian)))

        msg = Image()
        msg.header.frame_id = info.frame_id
        msg.header.stamp.sec = info.stamp_sec
        msg.header.stamp.nanosec = info.stamp_nanosec
        msg.height = info.height
        msg.width = info.width
        msg.encoding = encoding.value
        msg.is_bigendian = info.is_bigendian
        msg.step = info.width * channels * encoding.bytes_per_channel
        msg.data = np.ascontiguousarray(pixels).tobytes()
        return msg
