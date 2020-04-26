#!/usr/bin/env bash

xhost +local:root 1>/dev/null 2>&1
docker exec \
    -u geek \
    -it geek_release_$USER \
    /bin/bash
xhost -local:root 1>/dev/null 2>&1
