#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/arar/Documents/rectbot_ws/src/rectbot_sound"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/arar/Documents/rectbot_ws/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/arar/Documents/rectbot_ws/install/lib/python2.7/dist-packages:/home/arar/Documents/rectbot_ws/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/arar/Documents/rectbot_ws/build" \
    "/usr/bin/python2" \
    "/home/arar/Documents/rectbot_ws/src/rectbot_sound/setup.py" \
     \
    build --build-base "/home/arar/Documents/rectbot_ws/build/rectbot_sound" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/arar/Documents/rectbot_ws/install" --install-scripts="/home/arar/Documents/rectbot_ws/install/bin"
