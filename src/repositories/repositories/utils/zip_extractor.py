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
"""Safe extraction of in-memory ZIP archives into a target directory."""

import io
import shutil
import uuid
import zipfile
from pathlib import Path


def extract_zip_bytes(payload: bytes, target_dir: Path) -> Path:
    """Extract a ZIP archive held in memory into ``target_dir`` atomically.

    The archive is first extracted into a hidden sibling staging directory and
    then renamed onto ``target_dir``, so a partially extracted archive is never
    visible under the final name. Members that would escape the staging
    directory (absolute paths or ``..`` traversal) are rejected.

    Args:
        payload: Raw bytes of the ZIP archive.
        target_dir: Final directory to create; must not already exist.

    Returns:
        Path: The absolute path of the extracted directory.

    Raises:
        FileExistsError: If ``target_dir`` already exists.
        zipfile.BadZipFile: If ``payload`` is not a valid ZIP archive.
        ValueError: If any archive member would escape the target directory.
    """
    target_dir = target_dir.resolve()
    if target_dir.exists():
        raise FileExistsError(f"target '{target_dir}' already exists")

    staging_dir = target_dir.parent / f'.{target_dir.name}.partial-{uuid.uuid4().hex}'
    staging_dir.mkdir(parents=True)
    try:
        with zipfile.ZipFile(io.BytesIO(payload)) as archive:
            staging_root = staging_dir.resolve()
            for member in archive.infolist():
                member_path = (staging_root / member.filename).resolve()
                if not member_path.is_relative_to(staging_root):
                    raise ValueError(
                        f"archive member '{member.filename}' escapes the "
                        'target directory'
                    )
            archive.extractall(staging_root)
        staging_dir.rename(target_dir)
    except BaseException:
        shutil.rmtree(staging_dir, ignore_errors=True)
        raise
    return target_dir
