#!/bin/bash

# Linux X server setup
XAUTH=/tmp/docker.xauth
if [ ! -f $XAUTH ]
then
    xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
    if [ ! -z "$xauth_list" ]
    then
        echo $xauth_list | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

# Build husky_image from Dockerfile in this directory
docker build -t husky_image ./docker/

# Run husky_image with specified environment variables, volume mounting, and host networking
docker run \
    -it \
    --env="DISPLAY=$DISPLAY" \
    --env="LIBGL_ALWAYS_INDIRECT=$LIBGL_ALWAYS_INDIRECT" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$XAUTH:$XAUTH" \
    --privileged \
    --network=host \
    husky_image \
    bash