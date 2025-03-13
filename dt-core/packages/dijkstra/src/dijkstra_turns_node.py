#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy
import rospy
from duckietown_msgs.msg import FSMState, BoolStamped
from std_msgs.msg import Int16
from geometry_msgs.msg import Point

from dijkstra_utils.components.Graph import Graph
from dijkstra_utils.components.Route import Route
from dijkstra_utils.map.Map import DUCKIETOWN_CITY
from dijkstra_utils.Dijkstra import Dijkstra
from dijkstra.srv import DijkstraSrv, DijkstraSrvResponse


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
        self.sub_stop_line = rospy.Subscriber("lane_controller_node/intersection_done", BoolStamped, self.cbIntersectionDone, queue_size=1)

        # Services
        rospy.Service("~compute_path", DijkstraSrv, self.srv_start_dijkstra)

        # Read parameters
        self.pub_timestep = self.setupParameter("~pub_timestep", 1.0)

        # Dijkstra implementation
        self.is_following_path = False
        self.intersection_index = 0
        self.intersections = []
        self.path = []

        rospy.loginfo(f"[{self.node_name}] Initialized.")

    def compute_path(self, start, dest):
        graph = Graph()
        graph.generate_from_map(DUCKIETOWN_CITY)

        start_point = (start.x, start.y)
        dest_point = (dest.x, dest.y)
        start_node = next((node for node in graph.nodes if node.coordinates == start_point))
        dest_node = next((node for node in graph.nodes if node.coordinates == dest_point))
        route = Route(start_node, dest_node)

        dijkstra = Dijkstra(graph)
        self.path = dijkstra.get_shortest_path(route)
        self.intersections = [i for i, val in enumerate(self.path) if val.value.type.value in {"3W", "4W"}]
        self.intersection_index = 0

        self.is_following_path = True
        path_tiles = [tile.value.type.value for tile in self.path]
        rospy.loginfo(f"[{self.node_name}] Path: {path_tiles}, intersections: {self.intersections}")

    def srv_start_dijkstra(self, req):
        res = DijkstraSrvResponse()
        if self.is_following_path:
            res.type = "err"
            res.msg = "Duckiebot is already following a path"
            res.path = [Point(node.coordinates[0], node.coordinates[1], 0) for node in self.path]
        else:
            self.compute_path(req.start_point, req.dest_point)
            res.type = "succes"
            res.path = [Point(node.coordinates[0], node.coordinates[1], 0) for node in self.path]
            vec = [Point(node.coordinates[0], node.coordinates[1], 0) for node in self.path]
            #print(vec)

        return res

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
                return 1
            elif cur_x == inter_x:
                if ((cur_y < next_y) and (cur_x > next_x)) or ((cur_x < next_x) and (cur_y > next_y)):
                    return 2
                else:
                    return 0
            elif cur_y == inter_y:
                if ((cur_x < next_x) and (cur_y < next_y)) or ((cur_y > next_y) and (cur_x > next_x)):
                    return 2
                else:
                    return 0

        if self.intersection_index >= len(self.intersections):
            self.is_following_path = False
            self.intersection_index = 0
            self.intersections = []
            self.path = []

        if self.is_following_path:
            location = self.intersections[self.intersection_index]
            self.turn_type = get_direction(self.path[location - 1], self.path[location], self.path[location + 1])
            direction = "STRAIGHT" if self.turn_type == 1 else "LEFT" if self.turn_type == 0 else "RIGHT"
            #print(f"{self.intersection_index}: at tile {location} go {direction}")

        else:
            self.turn_type = -1
            
        self.pub_turn_type.publish(self.turn_type)

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

