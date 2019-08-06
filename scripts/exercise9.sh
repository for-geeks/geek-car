#!/usr/bin/env bash

# test 9 intergrated all modules

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