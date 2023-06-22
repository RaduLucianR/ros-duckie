#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launching app
dt-exec roslaunch object_detection object_detection_node.launch veh:=$VEHICLE_NAME

# wait for app to end
dt-launchfile-join
