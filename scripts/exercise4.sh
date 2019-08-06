#!/usr/bin/env bash

# test 4 localization and apriltag
bash chassis.sh stop
bash localization.sh stop
bash realsense.sh stop

bash realsense.sh
bash localization.sh
bash chassis.sh