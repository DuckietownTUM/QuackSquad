#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped


class WheelCmdNode(DTROS):

	def __init__(self, node_name):
		# initialize the DTROS parent class
		super(WheelCmdNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
		# static parameters
		self._vehicle_name = os.environ['VEHICLE_NAME']
		self._topic = f"/{self._vehicle_name}/wheels_driver_node/wheels_cmd_executed"

		# temporary data storage
		self.vel_left = None
		self.vel_right = None

		# construct subscriber
		self.sub = rospy.Subscriber(self._topic, WheelsCmdStamped, self.callback)

		self.const_dist = 10
		self.dist_x = 0
		self.dist_y = 0
		self.flag_first = True

	def callback(self, data):
		# store data value
		self.vel_left = data.vel_left
		self.vel_right = data.vel_right

		msg = f"Wheel velocity [LEFT, RIGHT]: {self.vel_left}, {self.vel_right}"
		rospy.loginfo(msg)

	def run(self):
		# publish received tick messages every 0.05 second (20 Hz)
		rate = rospy.Rate(20)
		print(rate)

		while not rospy.is_shutdown():
			if self.vel_left is not None and self.vel_right is not None:
				# start printing values when received from both encoders
				msg = f"Wheel velocity [LEFT, RIGHT]: {self.vel_left}, {self.vel_right}"
				rospy.loginfo(msg)

			rate.sleep()

if __name__ == '__main__':
	# create the node
	node = WheelCmdNode(node_name='wheel_command_node')
	print(node)
	# run the timer in node
	node.run()
	# keep spinning
	rospy.spin()
