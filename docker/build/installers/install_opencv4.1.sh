#!/usr/bin/env bash
git clone https://github.com/JetsonHacksNano/buildOpenCV

pushd buildOpenCV

./buildOpenCV.sh |& tee openCV_build.log

popd
