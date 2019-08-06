#!/usr/bin/env bash

# test 8 lat control

function start() {
    bash /apollo/scripts/realsense.sh
    bash /apollo/scripts/chassis.sh
}

function stop() {
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
