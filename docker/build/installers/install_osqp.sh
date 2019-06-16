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

# apt-get install -y libblas-dev liblapack-dev gfortran

wget https://github.com/ApolloAuto/osqp-contrib/archive/master.zip
unzip master.zip

pushd osqp-contrib-master
  mkdir -p /usr/local/include/osqp
  cp -r osqp/include /usr/local/include/osqp/
  cp osqp/libosqp.so /usr/local/lib/
popd

rm -fr master.zip osqp-contrib-master
