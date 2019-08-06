#!/usr/bin/env bash

# test for cyber 

function start() {
    bash /apollo/scripts/realsense_exercise1.sh 
    bash /apollo/scripts/chassis.sh
}

function stop() {
    bash /apollo/scripts/chassis.sh stop
    bash /apollo/scripts/realsense_exercise1.sh stop
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