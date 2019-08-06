#!/usr/bin/env bash

# test 2
function start() {
    bash /apollo/scripts/realsense.sh 
}

function stop() {
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