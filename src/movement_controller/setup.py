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

from glob import glob

from setuptools import find_packages, setup

PACKAGE_NAME = 'movement_controller'

setup(
    name=PACKAGE_NAME,
    version='0.1.0',
    packages=find_packages(exclude=['tests', 'tests.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         [f'resource/{PACKAGE_NAME}']),
        (f'share/{PACKAGE_NAME}', ['package.xml']),
        (f'share/{PACKAGE_NAME}/launch', glob('launch/*.py')),
        (f'share/{PACKAGE_NAME}/action', glob('action/*.action')),
        (f'share/{PACKAGE_NAME}/scripts', glob('scripts/*.py')),
        (f'share/{PACKAGE_NAME}/msg', glob('msg/*.msg')),
    ],
    install_requires=['setuptools', 'pydantic>=2.0,<3'],
    zip_safe=False,
    maintainer='Ron Freimann',
    maintainer_email='ron.freimann@ericsson.com',
    description=(
        'ROS2 package providing a vendor-agnostic action interface for executing '
        'collision-aware, blended multi-path trajectories on industrial robot arms '
        'using MoveIt2 and the PILZ motion planner.'
    ),
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'movement_controller = movement_controller.movement_controller:main',
        ],
    },
)
