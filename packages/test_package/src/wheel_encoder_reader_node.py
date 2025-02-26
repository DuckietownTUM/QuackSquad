#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelEncoderStamped


class WheelEncoderReaderNode(DTROS):

	def __init__(self, node_name):
		# initialize the DTROS parent class
		super(WheelEncoderReaderNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
		# static parameters
		self._vehicle_name = os.environ['VEHICLE_NAME']
		self._left_encoder_topic = f"/{self._vehicle_name}/left_wheel_encoder_node/tick"
		self._right_encoder_topic = f"/{self._vehicle_name}/right_wheel_encoder_node/tick"
		# temporary data storage
		self._ticks_left = None
		self._ticks_right = None
		# construct subscriber
		self.sub_left = rospy.Subscriber(self._left_encoder_topic, WheelEncoderStamped, self.callback_left)
		self.sub_right = rospy.Subscriber(self._right_encoder_topic, WheelEncoderStamped, self.callback_right)
		self.const_dist = 10
		self.dist_x = 0
		self.dist_y = 0
		self.flag_first = True

	def callback_left(self, data):
		# log general information once at the beginning
		rospy.loginfo_once(f"Left encoder resolution: {data.resolution}")
		rospy.loginfo_once(f"Left encoder type: {data.type}")
		# store data value
		self._ticks_left = data.data

	def callback_right(self, data):
		# log general information once at the beginning
		rospy.loginfo_once(f"Right encoder resolution: {data.resolution}")
		rospy.loginfo_once(f"Right encoder type: {data.type}")
		# store data value
		self._ticks_right = data.data

	def run(self):
		# publish received tick messages every 0.05 second (20 Hz)
		rate = rospy.Rate(0.2)
		prev_tick_l = 0
		prev_tick_r = 0
		ratio = 1
		self.dist_x = 0
		self.dist_y = 0
		dir_x = 1
		dir_y = 1
		angle = 0

		while not rospy.is_shutdown():
			if self._ticks_right is not None and self._ticks_left is not None:
				# start printing values when received from both encoders

				delta_tick_l = self._ticks_left - prev_tick_l
				delta_tick_r = self._ticks_right - prev_tick_r
				delta_left_right = delta_tick_l - delta_tick_r

				if not self.flag_first:
					angle += (delta_tick_l - delta_tick_r) * 0.9
					angle = angle % 360

				if angle >= 0 and angle < 90:
					dir_x = 1
					dir_y = 1
				elif angle >= 90 and angle < 180:
					dir_x = 1
					dir_y = -1
				elif angle >= 180 and angle < 270:
					dir_x = -1
					dir_y = -1
				elif angle >= 270 and angle < 360:
					dir_x = -1
					dir_y = 1

				msg = f"Wheel encoder ticks [LEFT, RIGHT | D_LEFT, D_RIGHT | DELTA]: {self._ticks_left}, {self._ticks_right} | {delta_tick_l}, {delta_tick_r} | {delta_left_right}"

				if delta_left_right > 3 and not self.flag_first:
					ratio = delta_tick_r / delta_tick_l
					msg = f"D_LEFT, D_RIGHT: [{delta_tick_l}, {delta_tick_r}] | Ratio L/R: {ratio} | Angle: {angle}"
					self.dist_x += self.const_dist * ratio * dir_x
					self.dist_y += self.const_dist * (1-ratio) * dir_y
				elif delta_left_right < -3 and not self.flag_first:
					ratio = delta_tick_l / delta_tick_r
					msg = f"D_LEFT, D_RIGHT: [{delta_tick_l}, {delta_tick_r}] | Ratio R/L: {ratio} | Angle: {angle}"
					self.dist_x += self.const_dist * ratio * dir_x
					self.dist_y += self.const_dist * (1-ratio) * dir_y

				self.flag_first = False
				rospy.loginfo(msg)
				rospy.loginfo(f"Dist from start: X:{self.dist_x}|Y:{self.dist_y}")
				prev_tick_l = self._ticks_left
				prev_tick_r = self._ticks_right

			rate.sleep()

if __name__ == '__main__':
	# create the node
	node = WheelEncoderReaderNode(node_name='wheel_encoder_reader_node')
	# run the timer in node
	node.run()
	# keep spinning
	rospy.spin()
