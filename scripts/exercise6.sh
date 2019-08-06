#!/usr/bin/env bash

# test planning dwa
bash chassis.sh stop
bash localization.sh stop
bash realsense.sh stop

bash realsense.sh
bash localization.sh
bash chassis.sh