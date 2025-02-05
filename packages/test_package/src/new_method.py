#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped

class PilotNode(DTROS):

    def __init__(self, node_name):
        super().__init__(node_name=node_name, node_type=NodeType.CONTROL)

        vehicle_name = os.environ['VEHICLE_NAME']
        topic = f"/{vehicle_name}/wheels_driver_node/wheels_cmd"

        self.gain = rospy.get_param('/duckie/kinematics_node/gain', 1.0)
        self.trim = rospy.get_param(f"/{vehicle_name}/kinematics_node/trim", 0.0)
        self.vel = 0.5  # Base velocity
        self.correction_factor = 1.0  # Dynamic correction factor

        self._publisher = rospy.Publisher(topic, WheelsCmdStamped, queue_size=1)

    def goStraight(self):
        rate = rospy.Rate(10)  # Loop at 10 Hz

        while not rospy.is_shutdown():
            # Calculate calibrated velocities
            calibratedVelLeft = self.vel * self.gain * (1 - self.trim)
            calibratedVelRight = self.vel * self.gain * (1 + self.trim * self.correction_factor)

            # Ensure values are within bounds
            calibratedVelLeft = max(0.0, min(1.0, calibratedVelLeft))
            calibratedVelRight = max(0.0, min(1.0, calibratedVelRight))

            # Publish the wheel commands
            msg = WheelsCmdStamped(vel_left=calibratedVelLeft, vel_right=calibratedVelRight)
            self._publisher.publish(msg)

            rate.sleep()

    def get_correction_factor(self):
        # Placeholder logic for dynamic correction factor
        left_wheel_speed = 1.0  # Example: Read encoder
        right_wheel_speed = 0.9  # Example: Read encoder

        if left_wheel_speed > right_wheel_speed:
            self.correction_factor -= 0.01
        elif right_wheel_speed > left_wheel_speed:
            self.correction_factor += 0.01

        return self.correction_factor

    def turnLeft(self):
        message = WheelsCmdStamped(vel_left=0.5, vel_right=1.0)
        self._publisher.publish(message)
        rospy.sleep(2)

    def turnRight(self):
        message = WheelsCmdStamped(vel_left=1.0, vel_right=0.5)
        self._publisher.publish(message)
        rospy.sleep(2)

    def stay(self):
        stop = WheelsCmdStamped(vel_left=0.0, vel_right=0.0)
        self._publisher.publish(stop)
        rospy.sleep(2)

    def run(self):
        while not rospy.is_shutdown():
            self.goStraight()
            self.stay()

            rospy.sleep(10)

    def on_shutdown(self):
        stop = WheelsCmdStamped(vel_left=0.0, vel_right=0.0)
        self._publisher.publish(stop)


# Place this block at the top level, outside the class definition
if __name__ == '__main__':
    node = PilotNode(node_name='pilot_node')
    node.run()
    rospy.spin()