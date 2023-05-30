#!/bin/bash
set -e

if [ ${ROS_DISTRO} == "foxy" ] ; then
	source "$ROS_WS/install/local_setup.bash"
else
	source "/opt/ros/$ROS_DISTRO/setup.bash" 
	source "$ROS_WS/devel/setup.bash"
fi
exec "$@"
