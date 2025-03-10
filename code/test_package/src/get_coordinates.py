#!/usr/bin/env python3

import os
import rospy
import math
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped, WheelEncoderStamped

TICKS_PER_REVOLUTION = 135
STRAIGHT_THRESHOLD = 0.05 # in rad
MM_PER_TICK = 1.55
BASE_LENGTH = 100     	# in mm
WHEEL_RADIUS = 31.8 	# in mm
RATE = 10
SMOOTH_FACTOR = 0.95
HEADING_RANGE = math.pi/8 # 22.5°

class OdometryNode(DTROS):

	def __init__(self, node_name):
		# initialize the DTROS parent class
		super(OdometryNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

		# static parameters
		vehicle_name = os.getenv("VEHICLE_NAME", "duckie") # 'duckie' as default, useful when lauching from outside the container
		left_encoder_topic = f"/{vehicle_name}/left_wheel_encoder_node/tick"
		right_encoder_topic = f"/{vehicle_name}/right_wheel_encoder_node/tick"

		# Previously recorded encoder tick's values
		self.prev_ticks_left = None
		self.prev_ticks_right = None

		# New encoder tick's values
		self.new_ticks_left = None
		self.new_ticks_right = None

		# Counter for message received
		self.cb_left = 0
		self.cb_right = 0
		self.cb_angle = 0

		# Robot position and orientation
		self.pos = {"x":300.0, "y":300.0} # [x, y] (in mm)
		self.angle = 0.0  # in radians
		self.heading = 0.0

		# Subscribers
		self.sub_left = rospy.Subscriber(left_encoder_topic, WheelEncoderStamped, self.callback_left)
		self.sub_right = rospy.Subscriber(right_encoder_topic, WheelEncoderStamped, self.callback_right)

	def callback_left(self, data):
		# log general information once at the beginning
		rospy.loginfo_once(f"Left encoder resolution: {data.resolution}")
		rospy.loginfo_once(f"Left encoder type: {data.type}")

		if self.prev_ticks_left is None:
			self.prev_ticks_left = data.data

		# store data value
		self.new_ticks_left = data.data
		self.cb_left += 1

	def callback_right(self, data):
		# log general information once at the beginning
		rospy.loginfo_once(f"Right encoder resolution: {data.resolution}")
		rospy.loginfo_once(f"Right encoder type: {data.type}")

		if self.prev_ticks_right is None:
			self.prev_ticks_right = data.data

		# store data value
		self.new_ticks_right = data.data
		self.cb_right += 1

	def run(self):
		# publish received tick messages every 0.1 second (10 Hz)
		rate = rospy.Rate(RATE)

		while not rospy.is_shutdown():
			#if self.new_ticks_left is None or self.new_ticks_right is None:
				#return

			self.compute_pos()
			rate.sleep()

	def compute_pos(self):
		# Don't compute if no message received from one wheel (could be a threshold)
		if self.cb_left == 0 or self.cb_right == 0:
			return

		# Compute the linear distances traveled by the wheels
		delta_tick_left = self.new_ticks_left - self.prev_ticks_left
		delta_tick_right = self.new_ticks_right - self.prev_ticks_right

		dist_left = 2 * math.pi * WHEEL_RADIUS * delta_tick_left / TICKS_PER_REVOLUTION
		dist_right = 2 * math.pi * WHEEL_RADIUS * delta_tick_right / TICKS_PER_REVOLUTION
		avg_dist = (dist_left + dist_right) / 2
		delta_angle = (dist_right - dist_left) / BASE_LENGTH # theta

		# Update after computing new position
		self.pos['x'] += avg_dist * math.cos(self.angle)
		self.pos['y'] -= avg_dist * math.sin(self.angle)
		#self.angle = SMOOTH_FACTOR*(self.angle + delta_angle) + (1 - SMOOTH_FACTOR)*self.angle
		self.angle += delta_angle
		self.angle = (self.angle) % (2 * math.pi)
		self.heading = (((self.angle + HEADING_RANGE/2)%(2*math.pi)) // HEADING_RANGE) * HEADING_RANGE
		#self.angle = self.heading

		# Update the ticks values
		self.prev_ticks_left = self.new_ticks_left
		self.prev_ticks_right = self.new_ticks_right

		# Update the message received counters
		self.cb_left = 0
		self.cb_right = 0


		# log information about the position
		msg = f"{self.pos['x']//600}|{self.pos['y']//600}(X:{self.pos['x']/10:.2f}/Y:{self.pos['y']/10:.2f}) | Angle:{self.angle:.4f} | Heading:{self.heading}"
		rospy.loginfo(msg)


	def on_shutdown(self):
		# store information before shutdown (maybe, we'll see)
		pass

if __name__ == '__main__':
	# create the node
	node = OdometryNode(node_name='odometry_node')
	# run node
	node.run()
	rospy.spin()
