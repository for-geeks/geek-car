#!/usr/bin/env bash

LOCAL_IMAGE="no"
FAST_BUILD_MODE="no"
FAST_TEST_MODE="no"
VERSION=""
ARCH=$(uname -m)
VERSION_X86_64="geek-release-x86_64-18.04-20200427_0036"
VERSION_OPT=""

function show_usage()
{
cat <<EOF
Usage: $(basename $0) [options] ...
OPTIONS:
    -b, --fast-build       Light mode for building without pulling all the map volumes
    -f, --fast-test        Light mode for testing without pulling limited set of map volumes
    -h, --help             Display this help and exit.
    -t, --tag <version>    Specify which version of a docker image to pull.
    -l, --local            Use local docker image.
    stop                   Stop all running Apollo containers.
EOF
exit 0
}

function stop_containers()
{
running_containers=$(docker ps --format "{{.Names}}")

for i in ${running_containers[*]}
do
  if [[ "$i" =~ geek_* ]];then
    printf %-*s 70 "stopping container: $i ..."
    docker stop $i > /dev/null
    if [ $? -eq 0 ];then
      printf "\033[32m[DONE]\033[0m\n"
    else
      printf "\033[31m[FAILED]\033[0m\n"
    fi
  fi
done
}

APOLLO_ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/../.." && pwd -P )"

if [ -e /proc/sys/kernel ]; then
    echo "/apollo/data/core/core_%e.%p" | sudo tee /proc/sys/kernel/core_pattern > /dev/null
fi

source ${APOLLO_ROOT_DIR}/scripts/apollo_base.sh

VOLUME_VERSION="latest"

OTHER_VOLUME_CONF=""

while [ $# -gt 0 ]
do
    case "$1" in
    -image)
        echo -e "\033[093mWarning\033[0m: This option has been replaced by \"-t\" and \"--tag\", please use the new one.\n"
        show_usage
        ;;
    -t|--tag)
        VAR=$1
        [ -z $VERSION_OPT ] || echo -e "\033[093mWarning\033[0m: mixed option $VAR with $VERSION_OPT, only the last one will take effect.\n"
        shift
        VERSION_OPT=$1
        [ -z ${VERSION_OPT// /} ] && echo -e "Missing parameter for $VAR" && exit 2
        [[ $VERSION_OPT =~ ^-.* ]] && echo -e "Missing parameter for $VAR" && exit 2
        ;;
    dev-*) # keep backward compatibility, should be removed from further version.
        [ -z $VERSION_OPT ] || echo -e "\033[093mWarning\033[0m: mixed option $1 with -t/--tag, only the last one will take effect.\n"
        VERSION_OPT=$1
        echo -e "\033[93mWarning\033[0m: You are using an old style command line option which may be removed from"
        echo -e "further versoin, please use -t <version> instead.\n"
        ;;
    -b|--fast-build)
        FAST_BUILD_MODE="yes"
        ;;
    -f|--fast-test)
        FAST_TEST_MODE="yes"
        ;;
    -h|--help)
        show_usage
        ;;
    -l|--local)
        LOCAL_IMAGE="yes"
        ;;
    --map)
        map_name=$2
        shift
        source ${APOLLO_ROOT_DIR}/docker/scripts/restart_map_volume.sh \
            "${map_name}" "${VOLUME_VERSION}"
        ;;
    stop)
	stop_containers
	exit 0
	;;
    *)
        echo -e "\033[93mWarning\033[0m: Unknown option: $1"
        exit 2
        ;;
    esac
    shift
done

if [ ! -z "$VERSION_OPT" ]; then
    VERSION=$VERSION_OPT
elif [ ${ARCH} == "x86_64" ]; then
    VERSION=${VERSION_X86_64}
else
    echo "Unknown architecture: ${ARCH}"
    exit 0
fi

if [ -z "${DOCKER_REPO}" ]; then
    DOCKER_REPO=geekstyle/geek_lite
fi

if [ "$LOCAL_IMAGE" == "yes" ] && [ -z "$VERSION_OPT" ]; then
    VERSION="local_dev"
fi


IMG=${DOCKER_REPO}:$VERSION

function local_volumes() {
    # Apollo root and bazel cache dirs are required.
    volumes="-v $APOLLO_ROOT_DIR:/apollo \
             -v $HOME/.cache:${DOCKER_HOME}/.cache"
    case "$(uname -s)" in
        Linux)
            volumes="${volumes} -v /dev:/dev \
                                -v /media:/media \
                                -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
                                -v /etc/localtime:/etc/localtime:ro \
                                -v /usr/src:/usr/src \
                                -v /lib/modules:/lib/modules"
            ;;
        Darwin)
            # MacOS has strict limitations on mapping volumes.
            chmod -R a+wr ~/.cache/bazel
            ;;
    esac
    echo "${volumes}"
}

function main(){

    if [ "$LOCAL_IMAGE" = "yes" ];then
        info "Start docker container based on local image : $IMG"
    else
        info "Start pulling docker image $IMG ..."
        docker pull $IMG
        if [ $? -ne 0 ];then
            error "Failed to pull docker image."
            exit 1
        fi
    fi

    APOLLO_RELEASE="geek_release_${USER}"
    docker ps -a --format "{{.Names}}" | grep "$APOLLO_RELEASE" 1>/dev/null
    if [ $? == 0 ]; then
        docker stop $APOLLO_RELEASE 1>/dev/null
        docker rm -v -f $APOLLO_RELEASE 1>/dev/null
    fi

    local display=""
    if [[ -z ${DISPLAY} ]];then
        display=":0"
    else
        display="${DISPLAY}"
    fi

    setup_device

    USER_ID=$(id -u)
    GRP=$(id -g -n)
    GRP_ID=$(id -g)
    LOCAL_HOST=`hostname`
    DOCKER_HOME="/home/$USER"
    if [ "$USER" == "root" ];then
        DOCKER_HOME="/root"
    fi
    if [ ! -d "$HOME/.cache" ];then
        mkdir "$HOME/.cache"
    fi

    info "Starting docker container \"${APOLLO_RELEASE}\" ..."

    DOCKER_CMD="nvidia-docker"
    USE_GPU=1
    if ! [ -x "$(command -v ${DOCKER_CMD})" ]; then
        DOCKER_CMD="docker"
        USE_GPU=0
    fi

    set -x

    ${DOCKER_CMD} run -it \
        -d \
        --privileged \
        --name $APOLLO_RELEASE \
        -e DISPLAY=$display \
        --net host \
        -w /home/geek/geek-car \
        --add-host in_dev_docker:127.0.0.1 \
        --add-host ${LOCAL_HOST}:127.0.0.1 \
        --hostname in_release_docker \
        --shm-size 2G \
        --pid=host \
        -v /dev/null:/dev/raw1394 \
        $IMG \
        /bin/bash
    set +x
    if [ $? -ne 0 ];then
        error "Failed to start docker container \"${APOLLO_RELEASE}\" based on image: $IMG"
        exit 1
    fi

    # if [ "${USER}" != "root" ]; then
    #     docker exec $APOLLO_RELEASE bash -c '/apollo/scripts/docker_adduser.sh'
    #     ok "Add user $USER"
    # fi

    ok "Finished setting up Apollo docker environment. Now you can enter with: \nbash docker/scripts/into_geek_release.sh"
    ok "Enjoy!"
}

main
