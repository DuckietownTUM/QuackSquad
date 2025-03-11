#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy
import rospy
from duckietown_msgs.msg import FSMState
from std_msgs.msg import Int16


class DijkstraTurnsNode:
    def __init__(self):
        # Save the name of the node
        self.node_name = rospy.get_name()
        self.turn_type = -1
        self.fsm_mode = None

        rospy.loginfo(f"[{self.node_name}] Initializing.")

        # Setup publishers
        self.pub_turn_type = rospy.Publisher("~turn_type", Int16, queue_size=1, latch=True)

        # Setup subscribers
        self.sub_topic_mode = rospy.Subscriber("~mode", FSMState, self.cbMode, queue_size=1)
        # self.sub_topic_tag = rospy.Subscriber("~pos", AprilTagsWithInfos, self.cbPos, queue_size=1)

        # Read parameters
        self.pub_timestep = self.setupParameter("~pub_timestep", 1.0)

        rospy.loginfo(f"[{self.node_name}] Initialzed.")
        self.rate = rospy.Rate(30)  # 10hz

    def cbMode(self, mode_msg):
        self.fsm_mode = mode_msg.state
        if self.fsm_mode != mode_msg.INTERSECTION_CONTROL:
            self.turn_type = -1
            self.pub_turn_type.publish(self.turn_type)

    def cbPos(self, pos_msgs):
        return
        # self.turn_type = chosenTurn
        # self.pub_turn_type.publish(self.turn_type)

    def setupParameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)
        return value

    def on_shutdown(self):
        rospy.loginfo(f"[{self.node_name}] Shutting down.")


if __name__ == "__main__":
    # Initialize the node with rospy
    rospy.init_node("dijkstra_turns_node", anonymous=False)

    # Create the NodeName object
    node = DijkstraTurnsNode()

    # Setup proper shutdown behavior
    rospy.on_shutdown(node.on_shutdown)

    # Keep it spinning to keep the node alive
    rospy.spin()