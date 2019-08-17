#!/usr/bin/env bash

# test 7 longitude control

function start() {
    bash /apollo/scripts/chassis.sh
}

function stop() {
    bash /apollo/scripts/chassis.sh stop
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