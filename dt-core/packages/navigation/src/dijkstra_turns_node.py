#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy
import rospy
from duckietown_msgs.msg import FSMState, BoolStamped
from std_msgs.msg import Int16

from dijkstra.components.Graph import Graph
from dijkstra.components.Route import Route
from dijkstra.map.Map import DUCKIETOWN_CITY
from dijkstra.Dijkstra import Dijkstra


class DijkstraTurnsNode:
    def __init__(self):
        # Save the name of the node
        self.node_name = rospy.get_name()
        self.turn_type = -1
        self.fsm_mode = None

        rospy.loginfo(f"[{self.node_name}] Initializing.")

        # Setup publishers
        self.pub_turn_type = rospy.Publisher("~turn_type_d", Int16, queue_size=1, latch=True)

        # Setup subscribers
        self.sub_topic_mode = rospy.Subscriber("~mode", FSMState, self.cbMode, queue_size=1)
        # self.sub_topic_tag = rospy.Subscriber("~pos", AprilTagsWithInfos, self.cbPos, queue_size=1)
        self.sub_stop_line = rospy.Subscriber("lane_controller_node/intersection_done", BoolStamped, self.cbIntersectionDone, queue_size=1)

        # Read parameters
        self.pub_timestep = self.setupParameter("~pub_timestep", 1.0)

        rospy.loginfo(f"[{self.node_name}] Initialized.")
        self.rate = rospy.Rate(30)  # 10hz

        # Dijkstra implementation
        graph = Graph()
        graph.generate_from_map(DUCKIETOWN_CITY)

        start_coordinates = (3, 0)
        to_coordinates = (3, 5)
        start = next((node for node in graph.nodes if node.coordinates == start_coordinates))
        to = next((node for node in graph.nodes if node.coordinates == to_coordinates))
        route = Route(start, to)

        dijkstra = Dijkstra(graph)
        self.path = dijkstra.get_shortest_path(route)
        self.intersections = [i for i, val in enumerate(self.path) if val.value.type.value in {"3W", "4W"}]
        self.intersection_index = 0
        print(f"Path: {self.path}, intersections: {self.intersections}, intersection_index: {self.intersection_index}")

    def cbMode(self, mode_msg):
        self.fsm_mode = mode_msg.state
        if self.fsm_mode != mode_msg.INTERSECTION_CONTROL:
            self.turn_type = -1
            self.pub_turn_type.publish(self.turn_type)

    def cbPos(self):
        # Dijkstra implementation
        def get_direction(current_node, intersection_node, next_node):
            cur_x, cur_y = current_node.coordinates
            inter_x, inter_y = intersection_node.coordinates
            next_x, next_y = next_node.coordinates

            if cur_x == next_x or cur_y == next_y:
                return "STRAIGHT"
            elif cur_x == inter_x:
                if ((cur_y < next_y) and (cur_x > next_x)) or ((cur_x < next_x) and (cur_y > next_y)):
                    return "RIGHT"
                else:
                    return "LEFT"
            elif cur_y == inter_y:
                if ((cur_x < next_x) and (cur_y < next_y)) or ((cur_y > next_y) and (cur_x > next_x)):
                    return "RIGHT"
                else:
                    return "LEFT"

        location = self.intersections[self.intersection_index]
        self.turn_type = get_direction(self.path[location - 1], self.path[location], self.path[location + 1])
        self.pub_turn_type.publish(self.turn_type)

        print(f"location: {location}, intersection_index: {self.intersection_index}, turn_type: {self.turn_type}")

    def cbIntersectionDone(self, intersection_done_msg):
        self.intersection_index += 1

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
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        node.cbPos()
        rate.sleep()
        
    # Setup proper shutdown behavior
    rospy.on_shutdown(node.on_shutdown)

    # Keep it spinning to keep the node alive
    # rospy.spin()