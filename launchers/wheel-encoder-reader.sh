#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launch subscriber
rosrun test_package wheel_encoder_reader_node.py

# wait for app to end
dt-launchfile-join