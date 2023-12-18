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

echo_and_run cd "/home/ubuntu/workspaces/catkin_ws/src/universal_robot/ur_kinematics"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/ubuntu/workspaces/catkin_ws/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/ubuntu/workspaces/catkin_ws/install/lib/python3/dist-packages:/home/ubuntu/workspaces/catkin_ws/build/ur_kinematics/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/ubuntu/workspaces/catkin_ws/build/ur_kinematics" \
    "/usr/bin/python3" \
    "/home/ubuntu/workspaces/catkin_ws/src/universal_robot/ur_kinematics/setup.py" \
     \
    build --build-base "/home/ubuntu/workspaces/catkin_ws/build/ur_kinematics" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/ubuntu/workspaces/catkin_ws/install" --install-scripts="/home/ubuntu/workspaces/catkin_ws/install/bin"
