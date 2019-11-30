#!/usr/bin/env bash

# test 5 kalman filter
function start() {
    bash /apollo/scripts/realsense.sh
    # apriltag localization
    # bash /apollo/scripts/localization.sh
    bash /apollo/scripts/nooploop.sh
    bash /apollo/scripts/chassis.sh
}

function stop() {
    # apriltag localization
    # bash /apollo/scripts/localization.sh stop
    bash /apollo/scripts/nooploop.sh stop
    bash /apollo/scripts/chassis.sh stop
    bash /apollo/scripts/realsense.sh stop
}


case $1 in
  start)
    stop
    start
    ;;
  stop)
    stop
    ;;
  *)
    start
    ;;
esac