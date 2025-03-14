#!/usr/bin/env python3

import numpy as np
import rospy

from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.srv import ChangePatternRequest, ChangePattern
from duckietown_msgs.msg import (
    Twist2DStamped,
    LanePose,
    WheelsCmdStamped,
    BoolStamped,
    StopLineReading,
    VehicleCorners,
)
from std_msgs.msg import String, Int16, Header, Float32
from lane_controller.controller import LaneController

class LaneControllerNode(DTROS):
    """Computes control action.
    The node compute the commands in form of linear and angular velocities, by processing the estimate error in
    lateral deviationa and heading.
    The configuration parameters can be changed dynamically while the node is running via ``rosparam set`` commands.
    Args:
        node_name (:obj:`str`): a unique, descriptive name for the node that ROS will use
    Configuration:
        ~v_bar (:obj:`float`): Nominal velocity in m/s
        ~k_d (:obj:`float`): Proportional term for lateral deviation
        ~k_theta (:obj:`float`): Proportional term for heading deviation
        ~k_Id (:obj:`float`): integral term for lateral deviation
        ~k_Iphi (:obj:`float`): integral term for lateral deviation
        ~d_thres (:obj:`float`): Maximum value for lateral error
        ~theta_thres_max (:obj:`float`): Maximum value for heading error
        ~theta_thres_min (:obj:`float`): Minimum value for heading error
        ~d_offset (:obj:`float`): Goal offset from center of the lane
        ~integral_bounds (:obj:`dict`): Bounds for integral term
        ~d_resolution (:obj:`float`): Resolution of lateral position estimate
        ~phi_resolution (:obj:`float`): Resolution of heading estimate
        ~omega_ff (:obj:`float`): Feedforward part of controller
        ~verbose (:obj:`bool`): Verbosity level (0,1,2)
        ~stop_line_slowdown (:obj:`dict`): Start and end distances for slowdown at stop lines

    Publisher:
        ~car_cmd (:obj:`Twist2DStamped`): The computed control action

    Subscribers:
        ~lane_pose (:obj:`LanePose`): The lane pose estimate from the lane filter
        ~intersection_navigation_pose (:obj:`LanePose`): The lane pose estimate from intersection navigation
        ~wheels_cmd_executed (:obj:`WheelsCmdStamped`): Confirmation that the control action was executed
        ~stop_line_reading (:obj:`StopLineReading`): Distance from stopline, to reduce speed
        ~obstacle_distance_reading (:obj:`stop_line_reading`): Distancefrom obstacle virtual stopline, to reduce speed
    """

    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(LaneControllerNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)

        #& Initialize the parameters (see `config/[file_name]/default.yaml`)
        self.params = dict()
        self.params["~v_bar"] = DTParam("~v_bar", param_type=ParamType.FLOAT, min_value=0.0, max_value=5.0)
        self.params["~k_d"] = DTParam("~k_d", param_type=ParamType.FLOAT, min_value=-100.0, max_value=100.0)
        self.params["~k_theta"] = DTParam("~k_theta", param_type=ParamType.FLOAT, min_value=-100.0, max_value=100.0)
        self.params["~k_Id"] = DTParam("~k_Id", param_type=ParamType.FLOAT, min_value=-100.0, max_value=100.0)
        self.params["~k_Iphi"] = DTParam("~k_Iphi", param_type=ParamType.FLOAT, min_value=-100.0, max_value=100.0)
        self.params["~d_offset"] = DTParam("~d_offset", param_type=ParamType.FLOAT)
        self.params["~d_thres"] = DTParam("~d_thres", param_type=ParamType.FLOAT, min_value=0.0, max_value=100.0)
        self.params["~theta_thres_max"] = DTParam("~theta_thres_max", param_type=ParamType.FLOAT, min_value=0.0, max_value=100.0)
        self.params["~theta_thres_min"] = DTParam("~theta_thres_min", param_type=ParamType.FLOAT, min_value=0.0, max_value=100.0)
        self.params["~omega_ff"] = DTParam("~omega_ff", param_type=ParamType.FLOAT, min_value=0.0, max_value=8.0)
        self.params["~deriv_type"] = rospy.get_param("~deriv_type", "error")
        self.params["~integral_bounds"] = rospy.get_param("~integral_bounds", None)
        self.params["~d_resolution"] = rospy.get_param("~d_resolution", None)
        self.params["~phi_resolution"] = rospy.get_param("~phi_resolution", None)
        self.params["~verbose"] = rospy.get_param("~verbose", None)
        self.params["~stop_line_slowdown"] = rospy.get_param("~stop_line_slowdown", None)

        self.stop_time = DTParam("~stop_time", param_type=ParamType.FLOAT, default=3.0)

        self.l_turn_v = DTParam("~l_turn_v")
        self.l_turn_omega = DTParam("~l_turn_omega")
        self.l_turn_dist = DTParam("~l_turn_dist")

        self.r_turn_v = DTParam("~r_turn_v")
        self.r_turn_omega = DTParam("~r_turn_omega")
        self.r_turn_dist = DTParam("~r_turn_dist")

        # Useful if the bot doesn't go straight on its own
        self.s_turn_v = DTParam("~s_turn_v")
        self.s_turn_omega = DTParam("~s_turn_omega")
        self.s_turn_dist = DTParam("~s_turn_dist")

        # Need to create controller object before updating parameters, otherwise it will fail
        self.controller = LaneController(self.params)

        # Initialize variables
        self.wheels_cmd_executed = WheelsCmdStamped()
        self.pose_msg = None
        self.pose_msg_dict = dict()
        self.last_s = None
        self.stop_line_distance = None
        self.at_stop_line = False
        self.obstacle_stop_line_distance = None
        self.obstacle_stop_line_detected = False
        self.at_obstacle_stop_line = False
        self.prev_at_stop_line_time = None
        self.current_pose_source = "lane_filter"
        self.turn_type = -1
        self.total_dist = 0
        self.dist_when_stopped = 0
        self.turn_dist = None
        self.is_turning = False
        self.drive_running = False

        self.led_signals = [
            String("CAR_SIGNAL_LEFT"),
            String("CAR_DRIVING"),
            String("CAR_SIGNAL_RIGHT"),
        ]

        self.turn_params = [
            (self.l_turn_v, self.l_turn_omega, self.l_turn_dist),
            (self.s_turn_v, self.s_turn_omega, self.s_turn_dist),
            (self.r_turn_v, self.r_turn_omega, self.r_turn_dist),
        ]

        # Construct publishers
        self.pub_car_cmd = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1, dt_topic_type=TopicType.CONTROL)
        self.pub_wheels_cmd = rospy.Publisher("~wheels_cmd", WheelsCmdStamped, queue_size=1, dt_topic_type=TopicType.CONTROL)
        self.pub_intersection_done = rospy.Publisher("~intersection_done", BoolStamped, queue_size=1)
        self.pub_idle_mode = rospy.Publisher("/duckie/joy_mapper_node/idle_mode", BoolStamped, queue_size=1)

        # Construct subscribers
        self.sub_lane_reading = rospy.Subscriber("~lane_pose", LanePose, self.cb_all_poses, "lane_filter", queue_size=1)
        self.sub_wheels_cmd_executed = rospy.Subscriber("~wheels_cmd", WheelsCmdStamped, self.cb_wheels_cmd_executed, queue_size=1)
        self.sub_stop_line = rospy.Subscriber("~stop_line_reading", StopLineReading, self.cb_stop_line_reading, queue_size=1)
        self.sub_obstacle_stop_line = rospy.Subscriber("~obstacle_distance_reading", StopLineReading, self.cb_obstacle_stop_line_reading, queue_size=1)
        self.sub_turn_type = rospy.Subscriber("dijkstra_turns_node/turn_type", Int16, self.cb_turn_type)
        self.sub_total_dist = rospy.Subscriber("deadreckoning_node/total_dist", Float32, self.cb_total_dist)

        # LED control service
        rospy.wait_for_service("/duckie/led_emitter_node/set_pattern", timeout=5)
        self.led_svc = rospy.ServiceProxy("/duckie/led_emitter_node/set_pattern", ChangePattern)

        #rospy.sleep(2)  # Wait for other nodes to start
        self.log("Initialized!")

    def cb_turn_type(self, msg):
        if msg.data == -1:
            idle_msg = BoolStamped()
            idle_msg.header = Header()
            idle_msg.data = True
            #self.pub_idle_mode.publish(idle_msg)
            #rospy.sleep(3)  #stop the node for the transition time

        self.turn_type = msg.data

    def cb_obstacle_stop_line_reading(self, msg):
        """
        Callback storing the current obstacle distance, if detected.

        Args:
            msg(:obj:`StopLineReading`): Message containing information about the virtual obstacle stopline.
        """
        self.obstacle_stop_line_distance = np.sqrt(msg.stop_line_point.x**2 + msg.stop_line_point.y**2)
        self.obstacle_stop_line_detected = msg.stop_line_detected
        self.at_obstacle_stop_line = msg.at_stop_line

    def cb_stop_line_reading(self, msg):
        """Callback storing current distance to the next stopline, if one is detected.

        Args:
            msg (:obj:`StopLineReading`): Message containing information about the next stop line.
        """

        # Only stop at stop lines at minimum s second intervals
        if msg.at_stop_line and self.prev_at_stop_line_time is not None:
           if msg.header.stamp.to_sec() - self.prev_at_stop_line_time.to_sec() < 2:
                   return

        self.at_stop_line = msg.at_stop_line
        self.prev_at_stop_line_time = msg.header.stamp

    def cb_all_poses(self, input_pose_msg, pose_source):
        """Callback receiving pose messages from multiple topics.

        If the source of the message corresponds with the current wanted pose source, it computes a control command.

        Args:
            input_pose_msg (:obj:`LanePose`): Message containing information about the current lane pose.
            pose_source (:obj:`String`): Source of the message, specified in the subscriber.
        """

        if pose_source == self.current_pose_source:
            self.pose_msg_dict[pose_source] = input_pose_msg
            self.pose_msg = input_pose_msg
            self.save_pose(self.pose_msg)

    def cb_wheels_cmd_executed(self, msg_wheels_cmd):
        """Callback that reports if the requested control action was executed.

        Args:
            msg_wheels_cmd (:obj:`WheelsCmdStamped`): Executed wheel commands
        """
        self.wheels_cmd_executed = msg_wheels_cmd

        if msg_wheels_cmd.vel_left == 0 and msg_wheels_cmd.vel_right == 0:
            self.dist_when_stopped = self.total_dist

    def cb_total_dist(self, dist_msg):
        self.total_dist = dist_msg.data

    def publish_cmd(self, car_cmd_msg):
        """Publishes a car command message.

        Args:
            car_cmd_msg (:obj:`Twist2DStamped`): Message containing the requested control action.
        """
        self.pub_car_cmd.publish(car_cmd_msg)

    def change_leds(self, leds):
        # Set the LED signal lights
        self.log(f"Setting leds: {leds}")
        msg = ChangePatternRequest(leds)

        try:
            self.led_svc(msg)
        except rospy.ServiceException as e:
            self.log(f"Could not set LEDs: {e}", "warn")

    def save_pose(self, pose_msg):
        self.pose_msg = pose_msg

    def lane_following(self, pose_msg, dt):
        # Compute errors
        d_err = pose_msg.d - self.params["~d_offset"].value
        phi_err = pose_msg.phi

        # We cap the error if it grows too large
        if np.abs(d_err) > self.params["~d_thres"].value:
            self.log("d_err too large, thresholding it!", "warn")
            d_err = np.sign(d_err) * self.params["~d_thres"].value

        if phi_err > self.params["~theta_thres_max"].value or phi_err < self.params["~theta_thres_min"].value:
            self.log(f"phi_err too small/large({phi_err}), thresholding it!", "warn")
            phi_err = np.clip(phi_err, self.params["~theta_thres_min"].value, self.params["~theta_thres_max"].value)

        wheels_cmd_exec = [self.wheels_cmd_executed.vel_left, self.wheels_cmd_executed.vel_right]

        if self.obstacle_stop_line_detected:
            v, omega = self.controller.compute_control_action(d_err, phi_err, dt, wheels_cmd_exec, self.obstacle_stop_line_distance)

            # TODO: This is a temporarily fix to avoid vehicle image detection latency caused unable to stop in time.
            v = v * 0.25
            omega = omega * 0.25

        else:
            # stop_line_distance is always None to avoid unwanted very slow speed 
            v, omega = self.controller.compute_control_action(d_err, phi_err, dt, wheels_cmd_exec, self.stop_line_distance)

        # For feedforward action (i.e. during intersection navigation)
        omega += self.params["~omega_ff"].value

        # Initialize car control msg, add header from input message
        car_control_msg = Twist2DStamped()
        car_control_msg.header = pose_msg.header

        # Add commands to car message
        car_control_msg.v = v
        car_control_msg.omega = omega
        self.publish_cmd(car_control_msg)
    
    def at_intersection(self):
        self.at_stop_line = False
        self.log("At stop line")

        self.log(f"Selecting turn: {self.turn_type}")
        self.change_leds(self.led_signals[self.turn_type])

        # Wait at the stop line
        self.log(f"Sleeping for {self.stop_time.value} seconds")
        rospy.sleep(self.stop_time.value)

        if self.turn_type == 1:
            return

        # Construct turning command
        v, omega, turn_dist = self.turn_params[self.turn_type]
        self.turn_dist = turn_dist

        car_control_msg = Twist2DStamped()
        car_control_msg.header.stamp = rospy.Time.now()
        car_control_msg.v = v.value
        car_control_msg.omega = omega.value

        # # Turn
        self.publish_cmd(car_control_msg)
        self.log("Turning now")
        self.is_turning = True

    def turns(self):
        if self.dist_when_stopped is None:
            return

        if self.total_dist - self.dist_when_stopped < self.turn_dist:
            return

        # Construct the turn stopping command
        car_stop_msg = Twist2DStamped()
        car_stop_msg.v = 0
        car_stop_msg.omega = 0

        # # Stop turn
        self.publish_cmd(car_stop_msg)
        self.log("Stopping turn")
        self.is_turning = False
        self.dist_when_stopped = None

        self.change_leds(String("CAR_DRIVING"))

        done_msg = BoolStamped()
        done_msg.header = Header()
        done_msg.data = True
        self.pub_intersection_done.publish(done_msg)

    def drive(self):
        if self.drive_running:
            rospy.logfatal("drive is already running")
            return

        self.drive_running = True

        if self.pose_msg is None:
            self.drive_running = False
            return

        dt = None
        pose_msg = self.pose_msg
        current_s = rospy.Time.now().to_sec()

        if self.last_s is not None:
            dt = current_s - self.last_s

        # Stop
        if self.at_stop_line or self.at_obstacle_stop_line:
            print("stop in lane control")
            car_control_msg = Twist2DStamped()
            car_control_msg.header = pose_msg.header
            car_control_msg.v = 0
            car_control_msg.omega = 0
            self.publish_cmd(car_control_msg)

        if self.at_stop_line:
            self.at_intersection()

        elif self.is_turning:
            self.turns()

        else:  # Lane following
            self.lane_following(pose_msg, dt)

        # Set the current time stamp, needed for lane following
        # Important: this needs to be set whether we're doing lane following or
        # intersection navigation, otherwise when we go back to lane following
        # from intersection navigation the first step of lane following will
        # break
        self.last_s = current_s
        self.drive_running = False

    def on_shutdown(self):
        rospy.sleep(0.5)

        stop_msg = WheelsCmdStamped()
        stop_msg.header = Header()
        stop_msg.header.stamp = rospy.Time.now()
        stop_msg.vel_left = 0.0
        stop_msg.vel_right = 0.0
        self.pub_wheels_cmd.publish(stop_msg)

        rospy.loginfo("Published zero velocities to stop the Duckiebot.")

if __name__ == "__main__":
    # Initialize the node
    node = LaneControllerNode(node_name="lane_controller_node")
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        node.drive()
        rate.sleep()
