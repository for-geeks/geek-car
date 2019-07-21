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

# Fail on first error.
set -e

cd "$(dirname "${BASH_SOURCE[0]}")"

echo "Build and install apriltag 3"

#git clone https://github.com/AprilRobotics/apriltag
# sudo works
# pushd apriltag
#   make
#   make install
# popd
# rm -fr apriltag


wget https://github.com/AprilRobotics/apriltag/archive/3.1.1.tar.gz

tar zxzf 3.1.1.tar.gz
mkdir apriltag-build
pushd apriltag-build
cmake -DCMAKE_INSTALL_PREFIX=~/apriltag/ ../apriltag-3.1.1
make install
popd
rm -rf apriltag-3.1.1