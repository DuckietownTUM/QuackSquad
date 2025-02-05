#!/usr/bin/env python3
import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped

import math

class PilotNode(DTROS):

    def __init__(self, node_name):
        super().__init__(node_name=node_name, node_type=NodeType.CONTROL)

        vehicle_name = os.environ['VEHICLE_NAME']
        topic = f"/{vehicle_name}/wheels_driver_node/wheels_cmd"
        self._publisher = rospy.Publisher(topic, WheelsCmdStamped, queue_size=1)

        self.trim = rospy.get_param(f"/{vehicle_name}/kinematics_node/trim")

    def goStraight(self):
        message = WheelsCmdStamped(vel_left=1.0, vel_right=1.0)
        self._publisher.publish(message)
        rospy.sleep(2)

    # WORK AREA - START

    def getLeftWheelVelocity(self, x):
        v = 1
        d = 10
        r = 18

        startTime = 1.5
        endTime = 4.5

        if (0 <= x < startTime):
            return v
        elif (startTime <= x < endTime):
            return v * ((d/(2 * r)) * math.sin(math.pi * (x - startTime)/(endTime - startTime)) + 1.0)
        else:
            return v

    def getRightWheelVelocity(self, x):
        startTime = 1.5
        endTime = 4.5

        if (0 <= x < startTime):
            return self.getLeftWheelVelocity(x)
        elif (startTime <= x < endTime):
            return 2-self.getLeftWheelVelocity(x)
        else:
            return self.getLeftWheelVelocity(x)

    def turnRight(self, x):
        calibratedVelLeft = self.getLeftWheelVelocity(x) * (1 - self.trim)
        calibratedVelRight = self.getRightWheelVelocity(x) * (1 + self.trim)
        message = WheelsCmdStamped(vel_left=calibratedVelLeft, vel_right=calibratedVelRight)
        self._publisher.publish(message)

    # WORK AREA - END

    def turnLeft(self):
        message = WheelsCmdStamped(vel_left=1.0, vel_right=1.0)
        self._publisher.publish(message)
        rospy.sleep(2)

    def stay(self):
        stop = WheelsCmdStamped(vel_left=0.0, vel_right=0.0)
        self._publisher.publish(stop)
        rospy.sleep(2)

    def run(self):
        rate = rospy.Rate(10)
        start_time = rospy.Time.now()

        while not rospy.is_shutdown():
            current_time = rospy.Time.now()  # Current ROS time
            elapsed_time = (current_time - start_time).to_sec()
            self.turnRight(elapsed_time)

            rate.sleep()

    def on_shutdown(self):
        stop = WheelsCmdStamped(vel_left=0.0, vel_right=0.0)
        self._publisher.publish(stop)


if __name__ == '__main__':
    node = PilotNode(node_name='pilot_node')
    node.run()
    rospy.spin()
