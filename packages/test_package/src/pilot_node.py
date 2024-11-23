#!/usr/bin/env python3
import os

class PilotNode(DTROS):

    def __init__(self, node_name):
        super().__init__(node_name=node_name, node_type=NodeType.CONTROL)

        self._vehicle_name = os.environ.get('VehicleName')
        self._topic = f"/{self._vehicle_name}/wheels_driver_node/wheels_cmd"
        self._publisher = rospy.Publisher(topic, WheelsCmdStamped, queue_size=1)

    def run(self):
        rate = rospy.Rate(1)
        message = WheelsCmdStamped(vel_left=1.0, vel_right=1.0)

        while not rospy.is_shutdown():
            self._publisher.publish(message)
            rate.sleep()

    def on_shutdown(self):
        stop = WheelsCmdStamped(vel_left=0.0, vel_right=0.0)
        self._publisher.publish(stop)
