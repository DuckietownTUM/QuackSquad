#!/usr/bin/env python3
import numpy as np

import rospy
from duckietown.dtros import DTParam, DTROS, NodeType, ParamType
from duckietown_msgs.msg import BoolStamped, FSMState, LanePose, SegmentList, StopLineReading
from geometry_msgs.msg import Point
from std_msgs.msg import Float32

class StopLineFilterNode(DTROS):
    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(StopLineFilterNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)

        #& Initialize the parameters (see `config/[file_name]/default.yaml`)
        self.should_stop_dist = DTParam("~should_stop_dist", param_type=ParamType.FLOAT)
        self.stop_dist = DTParam("~stop_dist", param_type=ParamType.FLOAT)
        self.min_segs = DTParam("~min_segs", param_type=ParamType.INT)
        self.off_time = DTParam("~off_time", param_type=ParamType.FLOAT)
        self.min_y = DTParam("~min_y", param_type=ParamType.FLOAT)

        # state vars
        self.lane_pose = LanePose()
        self.sleep = False
        self.should_stop = False
        self.at_stop = False
        self.delta_dist = 0
        self.prev_dist = 0

        # Subscribers
        self.sub_segs = rospy.Subscriber("~segment_list", SegmentList, self.cb_segments)
        self.sub_lane = rospy.Subscriber("~lane_pose", LanePose, self.cb_lane_pose)
        self.sub_total_dist = rospy.Subscriber("deadreckoning_node/total_dist", Float32, self.cb_total_dist)
        self.sub_intersection_done = rospy.Subscriber("lane_controller_node/intersection_done", BoolStamped, self.cb_after_intersection)

        # Publishers
        self.pub_stop_line_reading = rospy.Publisher("~stop_line_reading", StopLineReading, queue_size=1)
        self.pub_at_stop_line = rospy.Publisher("~at_stop_line", BoolStamped, queue_size=1)


    def cb_total_dist(self, dist_msg):
        if not self.should_stop and not self.at_stop:
            self.prev_dist = dist_msg.data

        elif self.should_stop and not self.at_stop:
            #DEBUG print(dist_msg.data - self.prev_dist + 0.2 >= self.should_stop_dist.value + self.stop_dist.value)
            if dist_msg.data - self.prev_dist + 0.2 >= self.should_stop_dist.value + self.stop_dist.value: #stop_distance is from the center of the bot
                self.at_stop = True
                self.prev_dist  = dist_msg.data

        else:
            print(f"{dist_msg.data} | {self.prev_dist + 0.1}")
            if dist_msg.data > self.prev_dist + 0.1:
                print("put stop filter node to sleep")
                self.at_stop = False
                self.should_stop = False

                stop_line_reading_msg = StopLineReading()
                stop_line_reading_msg.stop_line_detected = False
                stop_line_reading_msg.at_stop_line = False
                self.pub_stop_line_reading.publish(stop_line_reading_msg)

                self.sleep = True

    def cb_after_intersection(self, done_msg):
        if not done_msg.data:
            return

        rospy.sleep(self.off_time.value)
        self.sleep = False

        self.loginfo("Resuming stop line detection after the intersection")

    def cb_lane_pose(self, lane_pose_msg):
        self.lane_pose = lane_pose_msg

    def cb_segments(self, segment_list_msg):
        if self.sleep:
            return

        good_seg_count = 0
        stop_line_x_accumulator = 0.0
        stop_line_y_accumulator = 0.0
        debug_seg = []
        flag_seg_too_far = False
        for segment in segment_list_msg.segments:
            if segment.color != segment.RED:
                continue
            if segment.points[0].x < 0 or segment.points[1].x < 0:  # the point is behind us
                continue
            if segment.points[0].x > 0.45 or segment.points[1].x > 0.45: # the point is too far away
                flag_seg_too_far = True
                continue

            p1_lane = self.to_lane_frame(segment.points[0])
            p2_lane = self.to_lane_frame(segment.points[1])
            avg_x = 0.5 * (p1_lane[0] + p2_lane[0])
            avg_y = 0.5 * (p1_lane[1] + p2_lane[1])
            stop_line_x_accumulator += avg_x
            #DEBUG debug_seg.append(f"{segment.points[0].y};{segment.points[1].y}")
            stop_line_y_accumulator += avg_y
            good_seg_count += 1.0

        stop_line_reading_msg = StopLineReading()
        stop_line_reading_msg.header.stamp = segment_list_msg.header.stamp
        if good_seg_count < self.min_segs.value:
            if not flag_seg_too_far:
                self.should_stop = False
                self.at_stop = False

            stop_line_reading_msg.stop_line_detected = False
            stop_line_reading_msg.at_stop_line = self.at_stop
            self.pub_stop_line_reading.publish(stop_line_reading_msg)


        else:
            stop_line_reading_msg.stop_line_detected = True
            stop_line_point = Point()
            stop_line_point.x = stop_line_x_accumulator / good_seg_count
            stop_line_point.y = stop_line_y_accumulator / good_seg_count
            stop_line_reading_msg.stop_line_point = stop_line_point

            #DEBUG print(f"{stop_line_point.y >= self.min_y.value}|{stop_line_point.x}")
            if stop_line_point.x < self.should_stop_dist.value and stop_line_point.y >= self.min_y.value and not self.should_stop:
                print("Should stop")
                self.should_stop = True

            print(self.at_stop)
            stop_line_reading_msg.at_stop_line = self.at_stop

            #DEBUG print(f"{stop_line_point.x < self.stop_distance.value} | {np.abs(stop_line_point.y) < self.max_y.value}")
            #DEBUG print(f"{stop_line_point.y}| {debug_seg}")

            if stop_line_reading_msg.at_stop_line:
                msg = BoolStamped()
                msg.header.stamp = stop_line_reading_msg.header.stamp
                self.pub_at_stop_line.publish(msg)

            self.pub_stop_line_reading.publish(stop_line_reading_msg)

    def to_lane_frame(self, point):
        p_homo = np.array([point.x, point.y, 1])
        phi = self.lane_pose.phi
        d = self.lane_pose.d
        T = np.array([[np.cos(phi), -np.sin(phi), 0], [np.sin(phi), np.cos(phi), d], [0, 0, 1]])
        p_new_homo = T.dot(p_homo)
        p_new = p_new_homo[0:2]
        return p_new

    def onShutdown(self):
        rospy.loginfo("[StopLineFilterNode] Shutdown.")


if __name__ == "__main__":
    lane_filter_node = StopLineFilterNode(node_name="stop_line_filter")
    rospy.spin()
