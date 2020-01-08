#!/usr/bin/env bash

###############################################################################
# Copyright 2018 The Apollo Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################

# intel realsense
# Fail on first error.
set -e

cd "$(dirname "${BASH_SOURCE[0]}")"

ARCH=$(uname -m)

if [ "$ARCH" == "x86_64" ]; then
	apt-key adv --keyserver keys.gnupg.net --recv-key C8B3A55A6F3EFCDE || apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C8B3A55A6F3EFCDE
	add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main" -u
	apt install librealsense2-dkms -y
	apt install librealsense2-utils -y
	apt install librealsense2-dev -y
	apt install librealsense2-dbg -y

	export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
elif [ "$ARCH" == "aarch64" ]; then
	git clone https://github.com/JetsonHacksNano/installLibrealsense
	pushd installLibrealsense
	bash installLibrealsense.sh
	bash buildLibrealsense.sh
	popd
	rm -rf installLibrealsense
	rm -rf ${HOME}/librealsense
else
	echo "not support $ARCH"
fi
