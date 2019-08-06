#!/usr/bin/env bash

# test 8 lat control
bash chassis.sh stop
bash realsense.sh stop

bash realsense.sh
bash chassis.sh
