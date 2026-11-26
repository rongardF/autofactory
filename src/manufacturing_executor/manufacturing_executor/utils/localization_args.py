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
"""Per-operation localization overrides read from the operation arguments."""

from collections.abc import Callable, Sequence

from repositories.enums.operation_repository import ArgumentTypeEnum
from repositories.models.operation_repository.argument_dto import ArgumentDTO

SKIP_LOCALIZATION_ARG = 'skip_localization'
FORCE_LOCALIZATION_ARG = 'force_localization'


def localization_override(
    operation_args: Sequence[ArgumentDTO] | None, warn: Callable[[str], None]
) -> bool | None:
    """Return the localization override requested by the operation arguments.

    Only a ``true`` boolean value overrides; ``force_localization`` wins over ``skip_localization``.
    Arguments of another type are ignored with a warning.

    :returns: ``True`` to force, ``False`` to skip, ``None`` when nothing is overridden.
    """
    if _flag(operation_args, FORCE_LOCALIZATION_ARG, warn):
        return True
    if _flag(operation_args, SKIP_LOCALIZATION_ARG, warn):
        return False
    return None


def _flag(operation_args: Sequence[ArgumentDTO] | None, name: str, warn: Callable[[str], None]) -> bool:
    for argument in operation_args or ():
        if argument.argument_name != name:
            continue
        if argument.argument_type is not ArgumentTypeEnum.BOOL:
            warn(f"Ignoring operation argument '{name}': expected bool, got {argument.argument_type.value}")
            return False
        return bool(argument.argument_value_bool)
    return False
