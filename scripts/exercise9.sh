#!/usr/bin/env bash

# test 9 intergrated all modules
bash chassis.sh stop
bash realsense.sh stop

bash realsense.sh
bash chassis.sh