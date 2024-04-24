#!/bin/bash
sed -i '/exec "$@"/i export ROS_PACKAGE_PATH="/opt/ros/${ROS_DISTRO}/share:${GNSS_SI_ROOT}/Examples/ROS"' /ros_entrypoint.sh
