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

echo_and_run cd "/home/jlukas/Desktop/My_Project/ROS1/aruco_drone/src/ros_numpy"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/jlukas/Desktop/My_Project/ROS1/aruco_drone/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/jlukas/Desktop/My_Project/ROS1/aruco_drone/install/lib/python2.7/dist-packages:/home/jlukas/Desktop/My_Project/ROS1/aruco_drone/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/jlukas/Desktop/My_Project/ROS1/aruco_drone/build" \
    "/usr/bin/python2" \
    "/home/jlukas/Desktop/My_Project/ROS1/aruco_drone/src/ros_numpy/setup.py" \
     \
    build --build-base "/home/jlukas/Desktop/My_Project/ROS1/aruco_drone/build/ros_numpy" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/jlukas/Desktop/My_Project/ROS1/aruco_drone/install" --install-scripts="/home/jlukas/Desktop/My_Project/ROS1/aruco_drone/install/bin"
