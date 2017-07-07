#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'Diego Baño, Matías Pavez'
__email__ = 'diego.bano@ug.uchile.cl, matias.pavez@ing.uchile.cl'

import rospy
from math import sin, cos, atan2, pi, sqrt
import tf

from std_msgs.msg import Empty, ColorRGBA
from geometry_msgs.msg import Twist, Point, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
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
    """
    # TODO: revisar para el caso en que radio curvatura caiga dentro del robot!
    # TODO: considerar velocidad enviada o delta para  computo de inflación
    # TODO: evitar dejar pegado el nodo al intentar procesar demasiados obstáculos!..
    # .. no es necesario procesarlos todos, sólo basta encontrar uno que incomode
    # TODO: Manejar ruido de sensores ... moving average?
    # TODO: se puede evitar revisar obstáculos innecesarios, limitando el rango angular de los sensores
    # TODO: manejo de puntos ciegos ... que pasa si estaba previamente inscrito
    # TODO: manejo de curr_vel vs. cmd_vel cuando son parecidas. self.linear_acceleration = abs(0.35)  # m/s/s
    # TODO: ROS Parameter Server
    # TODO: mutex para obstáculos y cmd_vel
    # TODO: No considera cmd_vel en caso de que sea obsoleta.
    # TODO: use other params to make sure this range is not too low.
    # TODO: Use to max velocities to complement obstacle optimization
    # UNSURE: do not process sensors when stopped?
    # UNSURE: reducir la velocidad en vez de dejarla en cero abruptamente

    # COLORS
    COLOR_GREEN = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.5)
    COLOR_ORANGE = ColorRGBA(r=0.9, g=0.5, b=0.2, a=0.5)
    COLOR_RED = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.5)

    def __init__(self):

        # =====================================================================
        # Logic Variables

        # default obstacles
        self.front_obstacles = list()
        self.rear_obstacles = list()

        # default laser positions
        self.laser_front_base_dist = None
        self.laser_rear_base_dist = None

        # robot cinematic parameters
        self.min_linear_velocity = abs(0.01)   # [m/s] The robot is considered to be stopped if issued velocity is lesser than this.
        self.min_angular_velocity = abs(0.01)  # [m/s] The robot is considered to be stopped if issued velocity is lesser than this.
        self.max_linear_velocity = abs(0.8)    # [m/s] Used to enforce velocity constraints.
        self.max_angular_velocity = abs(0.8)   # [rad/s] Used to enforce velocity constraints.
        self.linear_deacceleration = abs(0.3)  # m/s/s 0.3 is the nominal value for Pioneer 3AT

        # robot geometry
        self.robot_radius = 0.4

        # misc
        self.is_debug_enabled = True
        self.epsilon = 0.01
        self.spin_rate = rospy.Rate(10)
        self.max_obstacle_range = 2.0  # ignore obstacles outside this radius

        # Subscriber variables
        self.curr_vel = Twist()
        self.cmd_vel = Twist()
        self.last_cmd_time = rospy.Time.now()

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
        self.marker_pub = rospy.Publisher("/bender/nav/safety/markers", MarkerArray, queue_size=1)
        self.safety_pub = rospy.Publisher("/bender/nav/low_level_mux/obstacle", Empty, queue_size=1)  # enma

    # =========================================================================
    # Setup Methods
    # =========================================================================

    def _setup_subscribers(self):
        self.laser_front_sub = rospy.Subscriber('/bender/sensors/laser_front/scan_filtered', LaserScan, self.laser_front_input_cb_v2, queue_size=1)
        self.laser_rear_sub = rospy.Subscriber('/bender/sensors/laser_rear/scan', LaserScan, self.laser_rear_input_cb_v2, queue_size=1)
        self.vel_sub = rospy.Subscriber("/bender/nav/low_level_mux/cmd_vel", Twist, self.velocity_input_cb, queue_size=1)
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

        # query required initial information
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

    def laser_scan_to_obstacles(self, msg, laser_distance, rotation=0.0):
        obstacles = list()
        max_distance = self.max_obstacle_range + laser_distance
        theta_ = msg.angle_min
        for r in msg.ranges:
            if r < max_distance:
                """
                a first filter, deletes points before transformation to base frame
                the filter is: r < max_obstacle_range + distance(base, laser)
                """
                dx_ = r * cos(theta_)
                dy_ = r * sin(theta_ + rotation)

                dx = laser_distance + dx_
                dy = dy_

                d = sqrt(dx * dx + dy * dy)
                theta = atan2(dy, dx)

                if d < self.max_obstacle_range:
                    obstacles.append((d, theta + rotation))

            theta_ += msg.angle_increment
        return obstacles

    def laser_front_input_cb_v2(self, msg):
        self.front_obstacles = self.laser_scan_to_obstacles(msg, self.laser_front_base_dist)

    def laser_rear_input_cb_v2(self, msg):
        self.rear_obstacles = self.laser_scan_to_obstacles(msg, self.laser_rear_base_dist, pi)

    def odom_input_cb(self, msg):
        self.curr_vel = msg.twist.twist

    def velocity_input_cb(self, msg):
        self.last_cmd_time = rospy.Time.now()
        self.cmd_vel = msg

    # =========================================================================
    # Aux methods
    # =========================================================================

    def get_inflated_radius(self, cmd_vel):
        """
        Computes an inflated footprint for linear movements.

        It considers the current velocity and the stopping acceleration.
        The accel is computed by hand using the real robot.

        Inflation radius is computed using the following equation
          v_f^2 - v_i^2 = 2 a d
        to simulate how long it takes to stop the robot NOW.

        Then:
        - v_f: 0
        - v_i: current velocity
        - a = deacel
        - d = desired

        The inflated radius is always greater than the robot_radius,
        because the computed stop distance must the added to the
        external most point of the footprint!.

        Returns:
            float: inflated radius.
        """
        inflated = 0.0
        linear_vel = abs(cmd_vel.linear.x)
        if linear_vel > self.min_linear_velocity:
            a = linear_vel * linear_vel
            b = (2.0 * self.linear_deacceleration)
            inflated = a / b
        return self.robot_radius + inflated

    def get_curvature_radius(self, cmd_vel):
        if abs(cmd_vel.angular.z) < self.epsilon:
            return 0
        return abs(cmd_vel.linear.x / cmd_vel.angular.z)

    def get_linear_movement_orientation(self, cmd_vel):
        """
        Returns:
            >  +1 if the cmd_vel moves the robot forwards
            >   0 if the cmd_vel moves does not move the robot (just linear!)
            >  -1 if the cmd_vel moves the robot backwards
        """
        if abs(cmd_vel.linear.x) < self.min_linear_velocity:
            return 0
        if cmd_vel.linear.x > 0:
            return 1
        if cmd_vel.linear.x < 0:
            return -1
        return 0

    def get_angular_movement_orientation(self, cmd_vel):
        """
        Returns:
            >  +1 if the cmd_vel moves the robot clockwise
            >   0 if the cmd_vel moves does not rotates the robot
            >  -1 if the cmd_vel moves the robot counter-clockwise
        """
        if abs(cmd_vel.angular.z) < self.min_angular_velocity:
            return 0
        if cmd_vel.angular.z > 0:
            return 1
        if cmd_vel.angular.z < 0:
            return -1
        return 0

    def attempts_to_move(self):
        vx = abs(self.curr_vel.linear.x)
        va = abs(self.curr_vel.angular.z)

        wx = abs(self.cmd_vel.linear.x)
        wa = abs(self.cmd_vel.angular.z)
        if (vx > self.min_linear_velocity or wx > self.min_linear_velocity
           or va > self.min_angular_velocity or wa > self.min_angular_velocity):
            return True
        return False

    def is_point_inside_circle(self, r, x0, y0, x, y):
        dx = (x - x0)
        dy = (y - y0)
        if dx * dx + dy * dy <= r * r:
            return True
        return False

    def get_obstacle_in_toroid_section(self, obstacles, inflated_radius, curvature_radius, linear_orientation, angular_orientation):
        """
        returns a list of inscribed obstacles on the toroid section
        it depends on the current robot heading.

        this assumes the obstacles are already filtered to be at at most
        rm meters away
        """
        real = list()
        dangerous = list()
        candidates = list()

        # no obstacles to proccess
        if not obstacles:
            return real, dangerous, candidates

        # no movement => no obstacle
        if linear_orientation == 0 and angular_orientation == 0:
            return real, dangerous, candidates

        # compute small and big toroid circle params
        rm = curvature_radius
        r1 = curvature_radius - self.robot_radius
        r2 = curvature_radius + self.robot_radius
        x0 = 0
        y0 = linear_orientation * angular_orientation * rm

        # check obstacles
        for obstacle in obstacles:
            d = obstacle[0]

            # avoid computation
            if d > self.max_obstacle_range:
                continue

            # only consider inscribed obstacles
            if d > inflated_radius:
                candidates.append(obstacle)
                continue

            theta = obstacle[1]
            x = d * cos(theta)
            y = d * sin(theta)

            # only forward/backward movement
            if curvature_radius == 0.0:
                # obstacle must be on the correct side
                if x * linear_orientation < 0:
                    dangerous.append(obstacle)
                    continue

                # obstacle inside the rectangle
                if abs(y) > self.robot_radius:
                    dangerous.append(obstacle)
                    continue

                # at this point, the obstacle is inside a rectangle
                real.append(obstacle)
                continue

            # only process obstacles on the required quadrant
            if x * linear_orientation < 0:
                dangerous.append(obstacle)
                continue

            # obstacle must be outside the small circle
            if self.is_point_inside_circle(r1, x0, y0, x, y):
                dangerous.append(obstacle)
                continue

            # obstacle must be inside the big circle
            if not self.is_point_inside_circle(r2, x0, y0, x, y):
                dangerous.append(obstacle)
                continue
            real.append(obstacle)
        return real, dangerous, candidates

    # =========================================================================
    # Main Processing method
    # =========================================================================

    def spin_once(self):

        try:
            # --- velocity is too small to move ---
            if not self.attempts_to_move():
                # stop the robot... just in case. enforce
                self.send_velocity(Twist())
                rospy.loginfo_throttle(5.0, "Robot is stopped and no command has been issued. Enforcing stop state. This is just informative.")
                self.spin_rate.sleep()
                return

            # --- select target velocity ---
            curr_linear_orientation = self.get_linear_movement_orientation(self.curr_vel)
            curr_angular_orientation = self.get_angular_movement_orientation(self.curr_vel)
            req_linear_orientation = self.get_linear_movement_orientation(self.cmd_vel)
            req_angular_orientation = self.get_angular_movement_orientation(self.cmd_vel)
            cmd_vel = self.curr_vel
            if curr_linear_orientation == 0 and curr_angular_orientation == 0:
                # robot is stopped => use cmd_vel
                cmd_vel = self.cmd_vel
            # TODO: analyze this:
            # else:
            #     # increment/decrement velocity by acceleration
            #     cmd_vel = ...

            # --- movement properties ---
            linear_orientation = self.get_linear_movement_orientation(cmd_vel)
            angular_orientation = self.get_angular_movement_orientation(cmd_vel)
            inflated_radius = self.get_inflated_radius(cmd_vel)
            curvature_radius = self.get_curvature_radius(cmd_vel)
            # if linear_orientation == 0 and angular_orientation == 0:
            #     # the required velocity is too small, because of a in(de)crement
            #     self.send_velocity(Twist())
            #     self.spin_rate.sleep()
            #     return

            # -- separate obstacles into useful categories --
            # lists of tuples (distance, angle) to obstacles
            obstacles = self.front_obstacles + self.rear_obstacles

            # get real obstacles vs. not really obstacles.
            real_obstacles, dangerous_obstacles, candidate_obstacles = self.get_obstacle_in_toroid_section(
                obstacles,
                inflated_radius, curvature_radius,
                linear_orientation, angular_orientation
            )

            # -- just for debugging --
            if self.is_debug_enabled:
                remaining_obstacles = len(obstacles) - len(real_obstacles) - len(dangerous_obstacles) - len(candidate_obstacles)

                def linear_to_string(x):
                    return ("FORWARD" if x > 0 else ("BACKWARDS" if x < 0 else "NOT LINEAR"))

                def angular_to_string(a):
                    return ("CLOCKWISE" if a > 0 else ("COUNTER CLOCKWISE" if a < 0 else "NOT ANGULAR"))

                rospy.loginfo_throttle(
                    1.0, ""
                    + "\n> Movement properties:"
                    + "\n  - inflated_radius: %.2f [m]." % (inflated_radius)
                    + "\n  - curvature_radius: %.2f [m]." % (curvature_radius)
                    + "\n> Movement direction:"
                    + "\n  - current  : %s, %s" % (linear_to_string(curr_linear_orientation), angular_to_string(curr_angular_orientation))
                    + "\n  - requested: %s, %s" % (linear_to_string(req_linear_orientation), angular_to_string(req_angular_orientation))
                    + "\n> Velocities:"
                    + "\n  - current : %.2f[m/s], %.2f[rad/s]" % (self.curr_vel.linear.x, self.curr_vel.angular.z)
                    + "\n  - required: %.2f[m/s], %.2f[rad/s]." % (self.cmd_vel.linear.x, self.cmd_vel.angular.z)
                    + "\n> There are %d relevant obstacles." % (len(real_obstacles) + len(dangerous_obstacles))
                    + "\n  - %3d real" % (len(real_obstacles))
                    + "\n  - %3d dangerous" % (len(dangerous_obstacles))
                    + "\n  - %3d other" % (len(candidate_obstacles))
                    + "\n  - %3d ignored" % (remaining_obstacles)
                )

            is_required_cmd_obsolete = False
            if is_required_cmd_obsolete:
                # just stop the robot... no signal required
                self.send_velocity(Twist())
                rospy.logwarn_throttle(5.0, "Required cmd vel is obsolete ... stopping the robot.")
            else:
                if real_obstacles:
                    # Send stop signals!
                    self.stop_motion()
                    rospy.logwarn_throttle(1.0, "Dangerous cmd_vel wont be applied!")
                else:
                    # The velocity command is safe... send it!
                    self.send_velocity(self.cmd_vel)

            # visualization
            self.visualize_markers(
                inflated_radius,
                curvature_radius,
                linear_orientation,
                angular_orientation,
                real_obstacles,
                dangerous_obstacles,
                candidate_obstacles
            )

        except Exception as e:
            rospy.logerr("An error occurred. Sending stop cmd_vel to the base. Error description: %s" % e)
            self.stop_motion()

        self.spin_rate.sleep()

    # =========================================================================
    # Base motion methods
    # =========================================================================

    def stop_motion(self):
        self.send_velocity(Twist())
        self.safety_pub.publish(Empty())

    def send_velocity(self, cmd):
        cmd.linear.x = min(cmd.linear.x, self.max_linear_velocity)
        cmd.linear.x = max(cmd.linear.x, -self.max_linear_velocity)
        cmd.angular.x = min(cmd.angular.x, self.max_angular_velocity)
        cmd.angular.x = max(cmd.angular.x, -self.max_angular_velocity)
        self.vel_pub.publish(cmd)

    # =========================================================================
    # Visualization Methods
    # =========================================================================

    def genmarker_toroid(self, time, inflated_radius, curvature_radius, linear_orientation, angular_orientation):
        marker = Marker()
        marker.header.frame_id = self.base_frame
        marker.header.stamp = time
        marker.lifetime = rospy.Duration(0.3)
        marker.ns = "safety/obstacle_area"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.02
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        sections = 20
        r1 = curvature_radius - self.robot_radius
        r2 = curvature_radius + self.robot_radius

        # only linear velocity
        if curvature_radius == 0.0:
            dx = linear_orientation * inflated_radius / (sections - 1)
            for i in range(sections):
                marker.points.append(Point(x=i * dx, y=r1, z=0.05))
                marker.points.append(Point(x=i * dx, y=r2, z=0.05))

            return marker

        # mixed velocities
        rm = curvature_radius
        max_theta = inflated_radius / rm  # [rad] simple proportional rule
        dtheta = max_theta / (sections - 1)
        x0 = 0
        y0 = linear_orientation * angular_orientation * rm
        for i in range(sections):
            theta = i * dtheta

            dx1 = r1 * cos(theta)
            dy1 = r1 * sin(theta)

            dx2 = r2 * cos(theta)
            dy2 = r2 * sin(theta)

            x1 = x0 + linear_orientation * dy1
            y1 = y0 - linear_orientation * angular_orientation * dx1

            x2 = x0 + linear_orientation * dy2
            y2 = y0 - linear_orientation * angular_orientation * dx2

            marker.points.append(Point(x=x1, y=y1, z=0.05))
            marker.points.append(Point(x=x2, y=y2, z=0.05))
        return marker

    def genmarker_inflated_radius(self, time, inflated_radius):
        marker = Marker()
        marker.header.frame_id = self.base_frame
        marker.header.stamp = time
        marker.lifetime = rospy.Duration(0.3)
        marker.ns = "safety/expanded_footprint"
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.scale.x = 2 * inflated_radius
        marker.scale.y = 2 * inflated_radius
        marker.scale.z = 0.01
        return marker

    def genmarker_obstacles(self, time, obstacles, color):
        marker = Marker()
        marker.header.frame_id = self.base_frame
        marker.header.stamp = time
        marker.lifetime = rospy.Duration(0.3)
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.1
        marker.color.r = color.r
        marker.color.g = color.g
        marker.color.b = color.b
        marker.color.a = 1.0
        for obstacle in obstacles:
            d = obstacle[0]
            theta = obstacle[1]
            dx = d * cos(theta)
            dy = d * sin(theta)
            marker.points.append(Point(x=dx, y=dy, z=0.0))
            marker.points.append(Point(x=dx, y=dy, z=1.0))
        return marker

    def visualize_markers(self, inflated_radius, curvature_radius, linear_orientation, angular_orientation, real_obstacles, dangerous_obstacles, candidate_obstacles):

        msg = MarkerArray()
        now = rospy.get_rostime()

        # -- inflated_radius marker --
        marker_inflated_radius = self.genmarker_inflated_radius(now, inflated_radius)
        if real_obstacles:
            marker_inflated_radius.color = CmdVelSafety.COLOR_RED
        elif dangerous_obstacles:
            marker_inflated_radius.color = CmdVelSafety.COLOR_ORANGE
        else:
            marker_inflated_radius.color = CmdVelSafety.COLOR_GREEN
        msg.markers.append(marker_inflated_radius)

        # -- inscribed obstacles --
        marker_real_obstacles = self.genmarker_obstacles(now, real_obstacles, CmdVelSafety.COLOR_RED)
        marker_real_obstacles.ns = "safety/real_obstacles"
        msg.markers.append(marker_real_obstacles)

        # -- dangerous obstacles --
        marker_dangerous_obstacles = self.genmarker_obstacles(now, dangerous_obstacles, CmdVelSafety.COLOR_ORANGE)
        marker_dangerous_obstacles.ns = "safety/dangerous_obstacles"
        msg.markers.append(marker_dangerous_obstacles)

        # -- potential obstacles --
        marker_candidate_obstacles = self.genmarker_obstacles(now, candidate_obstacles, CmdVelSafety.COLOR_GREEN)
        marker_candidate_obstacles.ns = "safety/candidate_obstacles"
        msg.markers.append(marker_candidate_obstacles)

        # -- toroid --
        marker_toroid = self.genmarker_toroid(now, inflated_radius, curvature_radius, linear_orientation, angular_orientation)
        msg.markers.append(marker_toroid)
        self.marker_pub.publish(msg)


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
