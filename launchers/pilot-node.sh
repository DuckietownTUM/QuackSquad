#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launch subscriber
rosrun test_package pilot_node.py

# wait for app to end
dt-launchfile-join
