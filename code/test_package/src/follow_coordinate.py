#!/usr/bin/env python3

import os
import rospy
import math
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped, WheelEncoderStamped


# throttle and direction for each wheel
THROTTLE_LEFT = 0.25        # 50% throttle
DIRECTION_LEFT = 1         # forward
THROTTLE_RIGHT = 0.25       # 30% throttle
DIRECTION_RIGHT = 1       # backward
MM_PER_TICK = 1.55
BASE_LENGTH = 6.8     #in mm


class WheelControlNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(WheelControlNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # static parameters
        self._vehicle_name = os.environ['VEHICLE_NAME']
        wheels_topic = f"/{self._vehicle_name}/wheels_driver_node/wheels_cmd"
        left_encoder_topic = f"/{self._vehicle_name}/left_wheel_encoder_node/tick"
        right_encoder_topic = f"/{self._vehicle_name}/right_wheel_encoder_node/tick"
        # form the message
        self.flag_first = True
        self._ticks_left = None
        self._ticks_right = None
        self._vel_left = THROTTLE_LEFT * DIRECTION_LEFT
        self._vel_right = THROTTLE_RIGHT * DIRECTION_RIGHT
        # construct publisher
        self._publisher = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)
        self.sub_left = rospy.Subscriber(left_encoder_topic, WheelEncoderStamped, self.callback_left)
        self.sub_right = rospy.Subscriber(right_encoder_topic, WheelEncoderStamped, self.callback_right)

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
        prev_tick_l = 0
        prev_tick_r = 0
        dist_l = 0
        dist_r = 0
        dist_to_icr = 0
        angle = 0
        prev_angle = 0

        x_coord = 0
        y_coord = 0

        # publish received tick messages every 0.05 second (20 Hz)
        rate = rospy.Rate(20)
        message = WheelsCmdStamped(vel_left=self._vel_left, vel_right=self._vel_right)
        while not rospy.is_shutdown():
            if self._ticks_right is not None and self._ticks_left is not None:
                delta_tick_l = self._ticks_left - prev_tick_l
                delta_tick_r = self._ticks_right - prev_tick_r

                if self.flag_first:
                    prev_tick_l = self._ticks_left
                    prev_tick_r = self._ticks_right
                    self.flag_first = False
                    continue  # Skip first loop to avoid incorrect deltas

                dist_l = delta_tick_l * MM_PER_TICK
                dist_r = delta_tick_r * MM_PER_TICK

                if abs(dist_r - dist_l) < 1e-6:  # Going straight
                    x_coord += (dist_l + dist_r) / 2 * math.cos(prev_angle)
                    y_coord += (dist_l + dist_r) / 2 * math.sin(prev_angle)
                else:  # Turning
                    angle = (dist_r - dist_l) / BASE_LENGTH
                    dist_to_icr = (BASE_LENGTH / 2) * ((dist_r + dist_l) / (dist_r - dist_l))

                    x_coord += dist_to_icr * (math.sin(prev_angle + angle) - math.sin(prev_angle))
                    y_coord += dist_to_icr * (-math.cos(prev_angle + angle) + math.cos(prev_angle))

                    prev_angle += angle  # Update after computing new position

                prev_tick_l = self._ticks_left
                prev_tick_r = self._ticks_right

                msg = f"X:{x_coord:.2f} | Y:{y_coord:.2f} | R:{dist_to_icr:.2f} | w:{angle:.4f}"
                rospy.loginfo(msg)

            rate.sleep()


    def on_shutdown(self):
        stop = WheelsCmdStamped(vel_left=0, vel_right=0)
        self._publisher.publish(stop)

if __name__ == '__main__':
    # create the node
    node = WheelControlNode(node_name='wheel_control_node')
    # run node
    node.run()
    # keep the process from terminating
    rospy.spin()
