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
        self._publisher = rospy.Publisher(topic, WheelsCmdStamped, queue_size=1)

    def goStraight(self):
        message = WheelsCmdStamped(vel_left=1.0, vel_right=1.0)
        self._publisher.publish(message)
        rospy.sleep(2)

    def turnLeft(self):
        message = WheelsCmdStamped(vel_left=1.0, vel_right=1.0)
        self._publisher.publish(message)
        rospy.sleep(1)

    def turnRight(self):
        message = WheelsCmdStamped(vel_left=1.0, vel_right=1.0)
        self._publisher.publish(message)
        rospy.sleep(1)

    def run(self):
        self.goStraight()
        self.on_shutdown()
        self.goStraight()
        self.on_shutdown()

    def on_shutdown(self):
        stop = WheelsCmdStamped(vel_left=0.0, vel_right=0.0)
        self._publisher.publish(stop)
        rospy.sleep(1) # delete this


if __name__ == '__main__':
    node = PilotNode(node_name='pilot_node')
    node.run()
    rospy.spin()
