curDir=$(readlink -f `dirname $0`)
rm -rf "$curDir/target"
./generate.sh

# make sure docker can display gui
XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth
touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

docker run -it --rm \
        -v $curDir:/project/ \
        -v $XSOCK:$XSOCK:rw \
        -v $XAUTH:$XAUTH:rw \
        -e XAUTHORITY=${XAUTH} \
        -e DISPLAY \
        ema-coincar bash -c "cd project; ./compile.sh; ./startAll.sh"

chown -R `who am i | awk '{print $1}'` "$curDir/target/"