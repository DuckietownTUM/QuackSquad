#!/usr/bin/env python3

import os
import rospy
import math
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped, WheelEncoderStamped

STRAIGHT_THRESHOLD = 0.05 # in rad
MM_PER_TICK = 1.55
BASE_LENGTH = 68     # in mm

class KinematicsNode(DTROS):

	def __init__(self, node_name):
		# initialize the DTROS parent class
		super(KinematicsNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

		# static parameters
		vehicle_name = os.getenv("VEHICLE_NAME", "duckie") # 'duckie' as default, useful when lauching from outside the container
		left_encoder_topic = f"/{vehicle_name}/left_wheel_encoder_node/tick"
		right_encoder_topic = f"/{vehicle_name}/right_wheel_encoder_node/tick"

		self.flag_first = True # Flag to skip first loop

		# Current encoder tick counters
		self.ticks_left = None
		self.ticks_right = None

		# New encoder tick values
		self.new_ticks_left = None
		self.new_ticks_right = None

		# Last timestamp values
		self.last_timestamp_left = None
		self.last_timestamp_right = None

		# New timestamp values
		self.new_timestamp_left = None
		self.new_timestamp_right = None

		# Robot position and orientation
		self.pos = [0.0, 0.0] # [x, y] (in mm)
		self.angle = 0.0  # in radians

		# Subscribers
		self.sub_left = rospy.Subscriber(left_encoder_topic, WheelEncoderStamped, self.callback_left)
		self.sub_right = rospy.Subscriber(right_encoder_topic, WheelEncoderStamped, self.callback_right)

	def callback_left(self, data):
		# log general information once at the beginning
		rospy.loginfo_once(f"Left encoder resolution: {data.resolution}")
		rospy.loginfo_once(f"Left encoder type: {data.type}")

		# store data value
		self.new_ticks_left = data.data
		self.new_timestamp_left = data.header.stamp

	def callback_right(self, data):
		# log general information once at the beginning
		rospy.loginfo_once(f"Right encoder resolution: {data.resolution}")
		rospy.loginfo_once(f"Right encoder type: {data.type}")

		# store data value
		self.new_ticks_right = data.data
		self.new_timestamp_right = data.header.stamp

	def run(self):
		# publish received tick messages every 0.05 second (20 Hz)
		rate = rospy.Rate(20)

		while not rospy.is_shutdown():
			if self.new_ticks_left is None or self.new_ticks_right is None:
				rate.sleep()
				continue

			if self.flag_first:
				self.ticks_left = self.new_ticks_left
				self.ticks_right = self.new_ticks_right
				self.last_timestamp_left = self.new_timestamp_left
				self.last_timestamp_right = self.new_timestamp_right
				self.flag_first = False
				rate.sleep()
				continue  # Skip first loop to avoid incorrect deltas

			# Compute the delta times
			delta_time_left = (self.new_timestamp_left - self.last_timestamp_left).to_sec()
			delta_time_right = (self.new_timestamp_right - self.last_timestamp_right).to_sec()
			avg_delta_time = (delta_time_left + delta_time_right) / 2

			# Compute the linear distances traveled by the wheels
			vel_left = (self.new_ticks_left - self.ticks_left)*MM_PER_TICK / delta_time_left 
			vel_right = (self.new_ticks_right - self.ticks_right)*MM_PER_TICK / delta_time_right

			# Update the ticks values
			self.ticks_left = self.new_ticks_left
			self.ticks_right = self.new_ticks_right

			# Compute linear and angular velocities
			avg_vel = (vel_left + vel_right) / 2
			angular_vel = (vel_right - vel_left) / BASE_LENGTH # omega (ω)

			if abs(angular_vel) < STRAIGHT_THRESHOLD:  # Going straight
				self.pos[0] += avg_vel * avg_delta_time * math.cos(self.angle + angular_vel * avg_delta_time)
				self.pos[1] += avg_vel * avg_delta_time * math.sin(self.angle + angular_vel * avg_delta_time)
			else:  # Turning
				# Compute the distance to the instantaneous center of rotation (ICR)
				dist_to_icr = avg_vel / angular_vel # from formula R = B/2 * (Vr + Vl) / (Vr - Vl)

				self.pos[0] += dist_to_icr * (math.sin(self.angle + angular_vel*avg_delta_time) - math.sin(self.angle))
				self.pos[1] += dist_to_icr * (-math.cos(self.angle + angular_vel*avg_delta_time) + math.cos(self.angle))

			# Update after computing new position
			self.angle += angular_vel * avg_delta_time

			# log information about the position
			msg = f"{self.pos[0]//600}|{self.pos[1]//600}(X:{self.pos[0]/10:.2f}/Y:{self.pos[1]/10:.2f}) | Angle:{self.angle*180/math.pi:.4f}"
			rospy.loginfo(msg)

			rate.sleep()


	# def on_shutdown(self):
		# store information before shutdown (maybe, we'll see)

if __name__ == '__main__':
	# create the node
	node = KinematicsNode(node_name='kinematics_node')
	# run node
	node.run()
