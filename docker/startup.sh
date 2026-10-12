#!/usr/bin/env bash
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
#
# Devcontainer first-start script.
# Runs once via postCreateCommand after the workspace volume is mounted.
# Initialises rosdep and installs all ROS2 dependencies declared in package.xml
# files, then configures the interactive shell via .bashrc.
set -e

WORKSPACE=/workspaces/autofactory

echo "==> Initialising rosdep..."
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    rosdep init
fi
rosdep update --rosdistro=jazzy
apt-get update

echo "==> Installing ROS2 dependencies from package.xml files..."
# shellcheck source=/dev/null
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths "${WORKSPACE}/src" --ignore-src -r -y

echo "==> Installing Pylon SDK"
apt-get install -y --no-install-recommends libxcb-cursor0 libxcb-xinerama0 libxcb-xinput0 wget
mkdir -p /opt/pylon_setup
wget https://downloadbsl.blob.core.windows.net/software/pylon%2025.10.2/pylon-25.10.2_linux-x86_64_debs.tar.gz -P /opt/pylon_setup
wget https://downloads-ctf.baslerweb.com/dg51pdwahxgw/cmMx43uVISAcg4t7IqrrF/83487e4dcccac2c0644ee5fadc4c323c/pylon-supplementary-package-for-blaze-1.7.3.73dbe706a_x86_64_setup.tar.gz -P /opt/pylon_setup
cd /opt/pylon_setup
tar -C /opt/pylon_setup -xzf ./pylon-25.10.2_linux-x86_64_debs.tar.gz
tar -C /opt/pylon_setup -xf pylon-supplementary-package-for-blaze-1.7.3.73dbe706a_x86_64_setup.tar.gz
rm pylon-25.10.2_linux-x86_64_debs.tar.gz pylon-supplementary-package-for-blaze-1.7.3.73dbe706a_x86_64_setup.tar.gz
apt --fix-broken install --assume-yes ./pylon_25.10.2-deb0_amd64.deb
cd pylon-supplementary-package-for-blaze-1.7.3.73dbe706a_x86_64_setup
tar -C /opt/pylon -xzf \
      ./pylon-supplementary-package-for-blaze-1.7.3.73dbe706a_x86_64.tar.gz
cd ../..
rm -r ./pylon_setup

echo "==> Configuring interactive shell (.bashrc)..."
grep -qF '. /opt/venv/bin/activate' ~/.bashrc \
  || echo '. /opt/venv/bin/activate' >> ~/.bashrc
grep -qF 'source /opt/ros/jazzy/setup.bash' ~/.bashrc \
  || echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
grep -qF 'install/setup.bash' ~/.bashrc \
  || echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> ~/.bashrc

# add PYTHONPATH to .bashrc
echo 'PYTHONPATH=/opt/venv/lib/python3.12/site-packages:$PYTHONPATH' >> ~/.bashrc

echo "==> Devcontainer setup complete."
