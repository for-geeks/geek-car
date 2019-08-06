#!/usr/bin/env bash

# test 5 kalman filter
bash chassis.sh stop
bash localization.sh stop
bash realsense.sh stop

bash realsense.sh
bash localization.sh
bash chassis.sh