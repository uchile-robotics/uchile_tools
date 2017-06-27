#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'Diego Bano'
__email__ = 'diego.bano@ug.uchile.cl'

import rospy
from math import sin, cos, atan2, pi, sqrt, pow as mpow, isnan
import numpy
import tf

from geometry_msgs.msg import Twist, Point, PoseStamped
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from uchile_srvs.srv import Transformer


class CmdVelSafety(object):
    """
    This class intercepts cmd_vel messages, checks for dangerous
    outcomes and then publishes a safe cmd_vel.

    - Listens laser messages to check for obstacles.
    - Listens cmd_vel to check for dangerous commands
    - Listens odometry to check for current velocity

    - Publishes a safe cmd_vel.
    - Publishes a marker to visualize dangerous outcomes on rviz.

    # TODO: callbacks are too CPU heavy
    # TODO: do not process sensors when stopped
    # TODO: online version
    # TODO: actualizar accel y deccel... usar desde params
    
    # obs:
    # - 0.3/vel = factor de ajusta para la decel.
    # - >= es importante para interceptar mensajes enviados estando al lado de un obst.
    # 		TODO: permite seguir avanzando de a poco, hasta eventualmente chocar
    # - 
    """

    def __init__(self):

        # =====================================================================
        # Logic Variables

        # default laser obstacles
        self.laser_front_closest_point = [float("inf"), pi]
        self.laser_rear_closest_point = [float("inf"), pi]

        # default laser positions
        self.laser_front_base_dist = None
        self.laser_rear_base_dist = None

        # clock
        self.rate_pub = rospy.Rate(10)
        self.laser_front_cb_rate = rospy.Rate(10)
        # self.laser_rear_cb_rate = rospy.Rate(5)

        # last message
        self.last_msg = Twist()
        self.last_msg_time = rospy.Time.now()

        # ????
        self.prev_front_scan = [0.01] * 1500 # ojo con el size y con los nans/inf/max+1
        self.prev_rear_scan = [0.01] * 500

        # Security tune-up variables
        self.robot_radius = 0.6
        self.laser_range = pi / 9
        self.stoping_acc = 0.35  # must be greater than 0
        self.epsilon = 0.01

        # Subscriber variables
        self.curr_vel = Twist()
        self.sent_linear_vel = 0
        self.sent_angular_vel = 0
        self.cnt_front = 0
        self.cnt_rear = 0

        # =====================================================================
        # Setup ROS Interface

        #  Parameter Server
        self.laser_front_frame = rospy.get_param("laser_front_link", "/bender/sensors/laser_front_link")
        self.laser_rear_frame = rospy.get_param("laser_rear_link", "/bender/sensors/laser_rear_link")
        self.base_frame = rospy.get_param("base_link", "/bender/base_link")

        # Service Clients
        self.tf_client = rospy.ServiceProxy("/bender/tf/simple_pose_transformer/transform", Transformer)

        # Topic Subscribers (avoid computing until setup is done)
        self.laser_front_sub = None
        self.laser_rear_sub = None
        self.vel_sub = None
        self.odom_sub = None

        # Topic Publishers
        self.vel_pub = rospy.Publisher('/bender/nav/safety/low_level/cmd_vel', Twist, queue_size=2)
        self.marker_pub = rospy.Publisher("/bender/nav/safety/markers", Marker, queue_size=1)
        # self.safety_pub = rospy.Publisher("/bender/nav/low_level_mux/obstacle", Empty, queue_size=1) # enma

    # =========================================================================
    # Setup Methods
    # =========================================================================

    def _setup_subscribers(self):
        self.laser_front_sub = rospy.Subscriber('/bender/sensors/laser_front/scan_filtered', LaserScan, self.laser_front_input_cb, queue_size=1)
        self.laser_rear_sub = rospy.Subscriber('/bender/sensors/laser_rear/scan', LaserScan, self.laser_rear_input_cb, queue_size=1)
        self.vel_sub = rospy.Subscriber("/bender/nav/low_level_mux/cmd_vel", Twist, self.vel_output_cb, queue_size=1)
        self.odom_sub = rospy.Subscriber("/bender/nav/odom", Odometry, self.odom_input_cb, queue_size=1)

    def get_laser_to_base_transform(self, dist, ang, input_frame, target_frame):
        """
        This method returns the transform between a point in the laser frame to the base frame.

        Args:
            dist (float): Distance from laser.
            ang (float): Angle from laser.

        Returns:
            PoseStamped: Transformed to base_link
        """
        pose = PoseStamped()
        pose.header.frame_id = input_frame

        pose.pose.position.x = dist * cos(ang)
        pose.pose.position.y = dist * sin(ang)

        orien = tf.transformations.quaternion_from_euler(0, 0, ang)
        pose.pose.orientation.z = orien[2]
        pose.pose.orientation.w = orien[3]

        transformer = Transformer()
        transformer.pose_in = pose
        transformer.frame_out = target_frame

        return self.tf_client(transformer.pose_in, transformer.frame_out).pose_out

    def setup(self):
        """
        Retrieves the transformation between base and laser frames.
        """

        # marker
        self.marker = self.setup_marker()

        while not rospy.is_shutdown():
            try:
                front_laser_pose = self.get_laser_to_base_transform(0, 0, self.laser_front_frame, self.base_frame)
                rear_laser_pose = self.get_laser_to_base_transform(0, 0, self.laser_rear_frame, self.base_frame)
                self.laser_front_base_dist = _distance(front_laser_pose.pose.position, Point())
                self.laser_rear_base_dist = _distance(rear_laser_pose.pose.position, Point())
                self._setup_subscribers()
                return True
            except Exception as e:
                rospy.logerr_throttle(
                    5.0,  # every 5 seconds
                    "Safety layer cannot start yet: Could not transform pose from between laser ({} or {}) and base ({}) frames. Because {}."
                    .format(self.laser_front_frame, self.laser_rear_frame, self.base_frame, e))

        return False

    # =========================================================================
    # ROS Callbacks
    # =========================================================================

    def laser_front_input_cb(self, msg):
        """
        Laser Callback. Computes the distance and angle to the closest detected point.
        """
        min_dist = float("inf")
        min_ang = pi
        mean = sum(msg.ranges) * 1.0 / len(msg.ranges)

        curr_values = [0, msg.ranges[0], msg.ranges[1]]
        if not isnan(msg.ranges[0]):
            self.prev_front_scan[0] = msg.ranges[0]
        if not isnan(msg.ranges[1]):
            self.prev_front_scan[1] = msg.ranges[1]

        curr_mean = mean

        for i in range(2, len(msg.ranges)):
            if not isnan(msg.ranges[i]):
                self.prev_front_scan[i] = msg.ranges[i]

            # moving average distance
            curr_values[0] = curr_values[1]
            curr_values[1] = curr_values[2]
            curr_values[2] = self.prev_front_scan[i]
            curr_mean = numpy.mean(curr_values)

            curr_ang = msg.angle_min + i * msg.angle_increment

            # Computes turn radius based on current linear and angular velocity using r = lin_vel/ang_vel
            turn_r = abs(self.curr_vel.linear.x / self.curr_vel.angular.z) if abs(self.curr_vel.angular.z) > self.epsilon else 0
            base_ang = atan2(sin(curr_ang) * curr_mean, self.laser_front_base_dist + cos(curr_ang) * curr_mean)
            # Computes distance of point to turning radius and only considers it if it's inside the current path
            curr_dist = sqrt(mpow(self.laser_front_base_dist, 2) + mpow(curr_mean, 2) - 2 * self.laser_front_base_dist * curr_mean * cos(pi - curr_ang))
            curve_dist = sqrt(mpow(curr_dist, 2) + mpow(turn_r, 2) - 2 * curr_dist * turn_r * cos(pi / 2 + base_ang)) if turn_r != 0 else 0

            if curr_dist < min_dist and abs(curve_dist - turn_r) < self.robot_radius and abs(base_ang) < self.laser_range:
                min_ang = base_ang
                min_dist = curr_dist

        # Update closest point variable
        self.laser_front_closest_point = [min_dist, min_ang]
        self.laser_front_cb_rate.sleep()

    def laser_rear_input_cb(self, msg):
        """
        This method is the callback function for the rear laser subscriber. It calculates the distance and angle to the closest detected point.

        Args:
            msg (LaserScan): Laser scan message from rear laser

        Returns:
            None
        """
        min_dist = float("inf")
        min_ang = pi
        mean = sum(msg.ranges) * 1.0 / len(msg.ranges)

        curr_values = [0, msg.ranges[0], msg.ranges[1]]
        if not isnan(msg.ranges[0]):
            self.prev_rear_scan[0] = msg.ranges[0]
        if not isnan(msg.ranges[1]):
            self.prev_rear_scan[1] = msg.ranges[1]

        curr_mean = mean

        for i in range(2, len(msg.ranges)):
            if isnan(msg.ranges[i]):
                continue

            curr_values[0] = curr_values[1]
            curr_values[1] = curr_values[2]
            curr_values[2] = msg.ranges[i]

            curr_mean = numpy.mean(curr_values)

            curr_ang = msg.angle_min + i * msg.angle_increment

            turn_r = abs(self.curr_vel.linear.x / self.curr_vel.angular.z) if abs(self.curr_vel.angular.z) > self.epsilon else 0

            base_ang = atan2(sin(curr_ang) * curr_mean, self.laser_rear_base_dist + cos(curr_ang) * curr_mean)

            curr_dist = sqrt(mpow(self.laser_rear_base_dist, 2) + mpow(curr_mean, 2) - 2 * self.laser_rear_base_dist * curr_mean * cos(pi - curr_ang))

            curve_dist = sqrt(mpow(curr_dist, 2) + mpow(turn_r, 2) - 2 * curr_dist * turn_r * cos(pi / 2 + base_ang)) if turn_r != 0 else 0

            if curr_dist < min_dist and abs(curve_dist - turn_r) < self.robot_radius and abs(base_ang) < self.laser_range:
                min_ang = base_ang
                min_dist = curr_dist

        # Update closest point variable
        self.laser_rear_closest_point = [min_dist, min_ang]

    def odom_input_cb(self, msg):
        self.curr_vel = msg.twist.twist
        # self.rate_pub.sleep()

    def vel_output_cb(self, msg):
        self.sent_linear_vel = msg.linear.x
        self.sent_angular_vel = msg.angular.z

    # =========================================================================
    # Aux methods
    # =========================================================================

    def setup_marker(self):
        marker = Marker()
        marker.header.frame_id = self.base_frame
        marker.ns = "safety/expanded_footprint"
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.robot_radius
        marker.scale.y = self.robot_radius
        marker.scale.z = 0.01
        marker.color.a = 0.5
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        return marker

    def publish_marker(self):
        self.marker.header.stamp = rospy.get_rostime()
        self.marker_pub.publish(self.marker)

    def get_expansion_factor(self, obj_rotation):
        """
        Computes a footprint expansion factor for linear movements.

        It considers the current velocity and the stopping acceleration.
        The accel is computed by hand using the real robot.

        Args:
            obj_rotation (float): Angle position of the closest obstacle

        Returns:
            float: expansion factor, > 0 if closest point is in the front, < 0 if it's in the back
        """
        factor = 0
        if self.curr_vel.linear.x != 0:
            # obs: avoid self.sent_linear_vel. because it is not the real robot velocity
            # linear_vel = max(abs(self.curr_vel.linear.x), abs(self.sent_linear_vel))
            linear_vel = abs(self.curr_vel.linear.x)
            a = mpow(linear_vel, 3)  # [m^2/s^2]
            b = (2.0 * self.stoping_acc * 0.3)
            factor = a / b

        if cos(obj_rotation) > 0:
            side_sign = 1
        else:
            side_sign = -1
        return factor * side_sign

    # =========================================================================
    # Main Processing method
    # =========================================================================

    def spin_once(self):
        a_point = Point()

        try:
            # get closest distance to front obstacle
            trans_front = self.laser_front_closest_point[0]
            rot_front = self.laser_front_closest_point[1]
            a_point.x = trans_front * cos(rot_front)
            a_point.y = trans_front * sin(rot_front)
            dist_front = _distance(a_point, Point())

            # get closest distance to rear obstacle
            trans_rear = self.laser_rear_closest_point[0]
            rot_rear = self.laser_rear_closest_point[1]
            a_point.x = trans_rear * cos(rot_rear)
            a_point.y = trans_rear * sin(rot_rear)
            dist_rear = _distance(a_point, Point())

            # Choose between frontal and back point between front and back
            if self.sent_linear_vel > 0:
                closest = dist_front
                clos_ang = rot_front
            elif self.sent_linear_vel < 0:
                closest = dist_rear
                clos_ang = rot_rear + pi
            else:
                closest = min(dist_front, dist_rear)
                clos_ang = rot_front if dist_front < dist_rear else rot_rear + pi

            # calculating expansion factor
            expansion_factor = self.get_expansion_factor(clos_ang)
            expanded_radius = self.robot_radius + abs(expansion_factor)

            # visualize expansion factor
            self.marker.scale.x = self.robot_radius + expansion_factor
            self.publish_marker()

            # rospy.loginfo_throttle(
            #     0.4,
            #     " - Closest point is %.2f[m] from %s." % (closest, self.base_frame)
            #     + " Robot radius is %.2f[m] (expanded to %.2f[m], factor: %.2f)." % (self.robot_radius, expanded_radius, expansion_factor)
            #     + " Nearest obstacles: (front: %.2f[m]), (rear: %.2f[m])" % (dist_front, dist_rear)
            #     + " Linear velocity: %.2f[m/s]" % self.sent_linear_vel
            # )
            # rospy.loginfo_throttle(
            #     0.4,
            #     "INFO:"
            #     + "\n- Current linear velocity: %.2f[m/s]." % (self.curr_vel.linear.x)
            #     + "\n- Expanded current linear velocity: %.2f[m/s] " % (self.curr_vel.linear.x * expansion_factor)
            #     + "\n- Required linear velocity: %.2f[m/s]." % (self.sent_linear_vel)
            #     + "\n- Expanded Required linear velocity: %.2f[m/s] " % (self.sent_linear_vel * expansion_factor)
            #     + "\n- Closest point is %.2f[m] from %s." % (closest, self.base_frame)
            #     + "\n- Robot radius is %.2f[m] (expanded to %.2f[m], factor: %.2f)." % (self.robot_radius, expanded_radius, expansion_factor)
            #     + "\n- Nearest obstacles: (front: %.2f[m]), (rear: %.2f[m])" % (dist_front, dist_rear)
            # )

            # Check if closest point is inside the safety area,
            # in which case, stop movement if velocity moves the base in that direction
            if closest <= expanded_radius and self.sent_linear_vel * expansion_factor >= 0:
                rospy.logwarn(
                    "Dangerous cmd_vel, deleting linear component."
                    + "\n- Current linear velocity: %.2f[m/s]." % (self.curr_vel.linear.x)
                    + "\n- Expanded current linear velocity: %.2f[m/s] " % (self.curr_vel.linear.x * expansion_factor)
                    + "\n- Required linear velocity: %.2f[m/s]." % (self.sent_linear_vel)
                    + "\n- Expanded Required linear velocity: %.2f[m/s] " % (self.sent_linear_vel * expansion_factor)
                    + "\n- Closest point is %.2f[m] from %s." % (closest, self.base_frame)
                    + "\n- Robot radius is %.2f[m] (expanded to %.2f[m], factor: %.2f)." % (self.robot_radius, expanded_radius, expansion_factor)
                    + "\n- Nearest obstacles: (front: %.2f[m]), (rear: %.2f[m])" % (dist_front, dist_rear)
                )
                msg = Twist()
                msg.angular.z = self.curr_vel.angular.z
                self.vel_pub.publish(msg)
                # self.safety_pub.publish(Empty())

        except Exception as e:
            rospy.logerr("Stopping safety controller . Because %s" % e)

        self.rate_pub.sleep()


def _distance(p1, p2):
        """
        Returns the euclidean distance between 2 geometry_msgs/Point objects
        """
        dx = (p1.x - p2.x)
        dy = (p1.y - p2.y)
        dz = (p1.z - p2.z)
        return sqrt(dx * dx + dy * dy + dz * dz)


def main():
    rospy.init_node('cmd_vel_low_level_safety')
    safe = CmdVelSafety()
    if not safe.setup():
        return False

    while not rospy.is_shutdown():
        safe.spin_once()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print("Stopping safety node due to rospy shutdown signal.")
