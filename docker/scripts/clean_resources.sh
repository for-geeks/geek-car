#!/usr/bin/env bash

# Usage:
#   ./clean_resources.sh

if [ ! -z ${DOCKER_IMG} ]; then
  echo "This script is expected to be run on host instead of the container."
  echo "Please exit."
  exit 1
fi

# Credit to https://stackoverflow.com/questions/38118791/can-t-delete-docker-image-with-dependent-child-images
echo "Cleanup containers..."
docker rm $(docker ps -qa --no-trunc --filter "status=exited")

echo "Cleanup images..."
docker rmi $(docker images --filter "dangling=true" -q --no-trunc)

echo "Cleanup volumes..."
docker volume rm $(docker volume ls -qf dangling=true)

#echo "Cleanup images not used"
# docker rmi geekstyle/geek_lite:geek_pcl