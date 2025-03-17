#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy
import rospy
from duckietown_msgs.msg import FSMState, BoolStamped
from geometry_msgs.msg import Point
from std_msgs.msg import Int16
from dijkstra.msg import TileProgress

from dijkstra_utils.components.Graph import Graph
from dijkstra_utils.components.Route import Route
from dijkstra_utils.map.Map import DUCKIETOWN_CITY
from dijkstra_utils.Dijkstra import Dijkstra
from dijkstra.srv import SetRoute, SetRouteResponse
from deadreckoning.srv import SetPoint, SetPointResponse


class DijkstraTurnsNode:
    def __init__(self):
        # Save the name of the node
        self.node_name = rospy.get_name()
        self.turn_type = -1
        self.fsm_mode = None

        rospy.loginfo(f"[{self.node_name}] Initializing.")

        # Setup publishers
        self.pub_turn_type = rospy.Publisher("~turn_type", Int16, queue_size=1, latch=True)
        self.pub_idle_mode = rospy.Publisher("/duckie/joy_mapper_node/idle_mode", BoolStamped, queue_size=1)
        self.pub_tile_progress = rospy.Publisher("~tile_progress", TileProgress, queue_size=1)

        # Setup subscribers
        self.sub_topic_mode = rospy.Subscriber("~mode", FSMState, self.cbMode, queue_size=1)
        self.sub_stop_line = rospy.Subscriber("lane_controller_node/intersection_done", BoolStamped, self.cbIntersectionDone, queue_size=1)
        self.sub_coord = rospy.Subscriber("deadreckoning_node/coordinates", Point, self.cb_update_coord, queue_size=10)

        # Services
        rospy.Service("~compute_path", SetRoute, self.srv_start_dijkstra)
        self.srv_update_pos = rospy.ServiceProxy("/duckie/deadreckoning_node/set_start_point", SetPoint)

        # Read parameters
        self.pub_timestep = self.setupParameter("~pub_timestep", 1.0)

        # Dijkstra implementation
        self.is_following_path = False
        self.intersection_index = 0
        self.intersections = []
        self.path = []
        self.coord = None
        self.current_tile = 0

        self.compute_path(Point(x=3,y=0,z=0), Point(x=3,y=5,z=0))
        rospy.sleep(2)
        self.update_pos(Point(x=3,y=0,z=0))
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
        self.current_tile = 0

        self.is_following_path = True
        path_tiles = [tile.value.type.value for tile in self.path]
        rospy.loginfo(f"[{self.node_name}] Path: {path_tiles}, intersections: {self.intersections}")

    def srv_start_dijkstra(self, req):
        res = SetRouteResponse()
        if self.is_following_path:
            res.type = "err"
            res.msg = "Duckiebot is already following a path"
            res.path = [Point(node.coordinates[0], node.coordinates[1], 0) for node in self.path]
        else:
            self.compute_path(req.start_point, req.dest_point)
            res.path = [Point(node.coordinates[0], node.coordinates[1], 0) for node in self.path]
            
            if self.update_pos(req.start_point):
                res.type = "success"
                res.msg = f"Path computed and postion for odometry changed at {req.start_point.x}; {req.start_point.y}"
            else:
                res.type = "error"
                res.msg = "Path computed but unable to update position for odometry"

        return res
    
    def update_pos(self, new_pos):
        res = SetPointResponse()
        try:
            res = self.srv_update_pos(point=new_pos)
        except rospy.ServiceException as e:
            rospy.logwarn(f"Could not set start position: {e}")

        return res.success

    def cb_update_coord(self, coord_msg):
        if self.is_following_path and self.coord != coord_msg and coord_msg.x != 0 and coord_msg.y != 0:
            self.current_tile += 1
            self.pub_tile_progress.publish(TileProgress(current_tile=self.current_tile, total_tile=len(self.path)))

        self.coord = coord_msg

    def cbMode(self, mode_msg):
        self.fsm_mode = mode_msg.state
        if self.fsm_mode != mode_msg.INTERSECTION_CONTROL:
            self.turn_type = -1
            self.pub_turn_type.publish(self.turn_type)

    def get_next_turn(self):
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

        if self.check_at_dest():
            self.pub_idle_mode.publish(BoolStamped(data=True))

        if self.intersection_index >= len(self.intersections):
            self.is_following_path = False
            self.intersection_index = 0
            self.current_tile = 0
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

    def check_at_dest(self):
        if len(self.path) == 0 or self.coord is None:
            return

        dest_coord = self.path[-1].coordinates
        #print(f"{dest_coord}|{self.coord}")
        return dest_coord[0] == self.coord.x and dest_coord[1] == self.coord.y

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
        node.get_next_turn()
        rate.sleep()
        
    # Setup proper shutdown behavior
    rospy.on_shutdown(node.on_shutdown)

    # Keep it spinning to keep the node alive
    # rospy.spin()

