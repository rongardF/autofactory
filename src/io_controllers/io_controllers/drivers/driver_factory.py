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
"""Driver factory: pick the concrete :class:`AbstractIODriver` for a config.

Selection rule (see spec):

* ``simulated=True`` -> :class:`MockIODriver`, regardless of ``device_type``.
* otherwise -> the real driver for ``config.device_type``.

Adding a real device family is one new entry in ``_REAL_DRIVERS`` plus its
driver module. Real driver modules are imported lazily so a vendor dependency
(e.g. ``ur_msgs``) is only required when that device is actually selected.
"""

from io_controllers.enums.device_type_enum import DeviceTypeEnum
from io_controllers.interfaces.io_driver import AbstractIODriver
from io_controllers.models.io_config_dto import IOControllerConfigDTO


def create_driver(
    config: IOControllerConfigDTO,
    node: object,
    *,
    simulated: bool,
) -> AbstractIODriver:
    """Construct the driver instance appropriate for *config*.

    Args:
        config: The validated configuration envelope; ``device_type`` selects
            the real driver when not simulating.
        node: The owning lifecycle node, passed to real drivers so they can
            create subscriptions and service clients on its executor. Ignored
            by the device-agnostic mock driver.
        simulated: When ``True``, return a :class:`MockIODriver` regardless of
            ``device_type``.

    Returns:
        AbstractIODriver: An unconfigured driver instance (call ``configure``
        next).

    Raises:
        ValueError: If ``device_type`` has no registered real driver.
    """
    if simulated:
        # Imported here to keep the mock optional at import time and mirror the
        # lazy-import treatment of the real drivers.
        from io_controllers.drivers.mock_io_driver import MockIODriver

        return MockIODriver()

    factory = _REAL_DRIVERS.get(config.device_type)
    if factory is None:
        supported = ', '.join(sorted(d.value for d in _REAL_DRIVERS))
        raise ValueError(
            f'unsupported device_type {config.device_type.value!r}; '
            f'supported: {supported}'
        )
    return factory(node)


def _create_ur_driver(node: object) -> AbstractIODriver:
    """Lazily import and construct the UR driver (keeps ``ur_msgs`` optional)."""
    from io_controllers.drivers.ur_io_driver import URIODriver

    return URIODriver(node)


# Maps each supported real device family to a lazy constructor.
_REAL_DRIVERS = {
    DeviceTypeEnum.UR: _create_ur_driver,
}
