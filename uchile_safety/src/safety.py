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
from threading import Lock


class CmdVelSafety(object):
    """
    This class intercepts cmd_vel messages, checks for dangerous
    outcomes and then publishes a safe cmd_vel.

    - Listens laser messages to check for obstacles.
    - Listens cmd_vel to check for dangerous commands
    - Listens odometry to check for current velocity

    - Publishes a safe cmd_vel.
    - Publishes a marker to visualize dangerous outcomes on rviz.

    To Do List:
    ------------------------------
    TODO: add pointcloud
    TODO: Manejar ruido de sensores ... moving average?
    TODO: Manejo de puntos ciegos ... que pasa si estaba previamente inscrito
    TODO: Caso en que radio curvatura cae dentro del robot

    Stalled:
    ------------------------------
    TODO: mutex empeora la cosa?
    TODO: Use to max velocities to complement obstacle optimization
    TODO: Evitar dejar pegado el nodo al intentar procesar demasiados obstáculos
          no es necesario procesarlos todos, sólo basta encontrar uno que incomode
    TODO: Evitar revisar obstáculos innecesarios, limitando el rango angular de los sensores
          casos simples: forward veloz => no revisar rear.
    TODO: computos innecesarios (p.e visualización) en otro thread. Evitar afectar el thread principal.

    Unsure:
    ------------------------------
    TODO: Reducir la velocidad en vez de dejarla en cero abruptamente
    TODO: Use other params to make sure this range is not too low.
    """

    # COLORS
    COLOR_GREEN = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.5)
    COLOR_ORANGE = ColorRGBA(r=0.9, g=0.5, b=0.2, a=0.5)
    COLOR_RED = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.5)

    def __init__(self):

        # =====================================================================
        # ROS Parameters

        # robot cinematic parameters
        self.min_linear_vel = rospy.get_param("~min_linear_vel", 0.01)
        self.max_linear_vel = rospy.get_param("~max_linear_vel", 0.80)
        self.min_angular_vel = rospy.get_param("~min_angular_vel", 0.01)
        self.max_angular_vel = rospy.get_param("~max_angular_vel", 0.80)
        self.linear_decel = rospy.get_param("~linear_decel", 0.3)
        self.linear_accel = rospy.get_param("~linear_accel", 0.3)
        self.angular_decel = rospy.get_param("~angular_decel", 1.74)
        self.angular_accel = rospy.get_param("~angular_accel", 1.74)

        # robot frames
        self.base_frame = rospy.get_param("~base_frame", "/bender/base_link")
        self.laser_front_frame = rospy.get_param("~laser_front_frame", "/bender/sensors/laser_front_link")
        self.laser_rear_frame = rospy.get_param("~laser_rear_frame", "/bender/sensors/laser_rear_link")
        self.laser_rgbd_frame = rospy.get_param("~laser_rgbd_frame", "/bender/sensors/laser_rear_link")
        self.laser_front_base_dist = None
        self.laser_rear_base_dist = None
        self.laser_rgbd_base_dist = None

        # misc
        control_rate = rospy.get_param("~control_rate", 10)
        odom_timeout = rospy.get_param("~odom_timeout", 0.5)
        cmd_timeout = rospy.get_param("~cmd_timeout", 0.5)
        sim_lookahead_factor = rospy.get_param("~sim_lookahead_factor", 3.0)
        robot_radius = rospy.get_param("~robot_radius", 0.4)
        min_distance_to_obstacles = rospy.get_param("~min_distance_to_obstacles", 0.1)
        self.min_inflation_radius = robot_radius + min_distance_to_obstacles
        self.sim_time = float(sim_lookahead_factor) / float(control_rate)
        self.is_debug_enabled = rospy.get_param("~is_debug_enabled", False)
        self.is_visualization_enabled = rospy.get_param("~is_visualization_enabled", False)
        self.max_obstacle_range = rospy.get_param("~max_obstacle_range", 2.0)
        self.odom_timeout = rospy.Duration.from_sec(odom_timeout)
        self.cmd_timeout = rospy.Duration.from_sec(cmd_timeout)
        self.spin_rate = rospy.Rate(control_rate)

        # =====================================================================
        # Control variables

        # odometry
        self.odom_vel = Twist()
        self.last_odom_time = rospy.Time.now()
        self.odom_lock = Lock()

        # input commands
        self.cmd_vel = Twist()
        self.last_cmd_time = rospy.Time.now()
        self.cmd_lock = Lock()

        # sensor readings
        self.front_obstacles = list()
        self.rear_obstacles = list()
        self.rgbd_obstacles = list()
        self.sensor_front_lock = Lock()
        self.sensor_rear_lock = Lock()
        self.sensor_rgbd_lock = Lock()

        # =====================================================================
        # ROS Interface

        # Service Clients
        self.tf_client = rospy.ServiceProxy("/bender/tf/simple_pose_transformer/transform", Transformer)

        # Topic Subscribers (avoid computing until setup is done)
        self.laser_front_sub = None
        self.laser_rear_sub = None
        self.laser_rgbd_sub = None
        self.vel_sub = None
        self.odom_sub = None

        # Topic Publishers
        self.vel_pub = rospy.Publisher("~output", Twist, queue_size=2)
        self.marker_pub = rospy.Publisher("~markers", MarkerArray, queue_size=1)
        self.stop_triggered_pub = rospy.Publisher("~triggered", Empty, queue_size=1)

    # =========================================================================
    # Setup Methods
    # =========================================================================

    def _setup_subscribers(self):
        self.laser_front_sub = rospy.Subscriber('~scan_front', LaserScan, self.laser_front_callback, queue_size=1)
        self.laser_rear_sub = rospy.Subscriber('~scan_rear', LaserScan, self.laser_rear_callback, queue_size=1)
        self.laser_rgbd_sub = rospy.Subscriber('~scan_rgbd', LaserScan, self.laser_rgbd_callback, queue_size=1)
        self.vel_sub = rospy.Subscriber("~input", Twist, self.cmd_callback, queue_size=1)
        self.odom_sub = rospy.Subscriber("~odom", Odometry, self.odom_input_callback, queue_size=1)

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
                rgbd_laser_pose = self.get_laser_to_base_transform(0, 0, self.laser_rgbd_frame, self.base_frame)
                self.laser_front_base_dist = _distance(front_laser_pose.pose.position, Point())
                self.laser_rear_base_dist = _distance(rear_laser_pose.pose.position, Point())
                self.laser_rgbd_base_dist = _distance(rgbd_laser_pose.pose.position, Point())
                self._setup_subscribers()
                return True
            except Exception as e:
                rospy.logerr_throttle(
                    5.0,  # every 5 seconds
                    "Safety layer cannot start yet: Could not transform pose from between lasers ({} or {} or {}) and base ({}) frames. Because {}."
                    .format(self.laser_front_frame, self.laser_rear_frame, self.laser_rgbd_frame, self.base_frame, e))

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

    def laser_front_callback(self, msg):
        #with self.sensor_front_lock:
        self.front_obstacles = self.laser_scan_to_obstacles(msg, self.laser_front_base_dist)

    def laser_rear_callback(self, msg):
        #with self.sensor_rear_lock:
        self.rear_obstacles = self.laser_scan_to_obstacles(msg, self.laser_rear_base_dist, pi)

    def laser_rgbd_callback(self, msg):
        #with self.sensor_rgbd_lock:
        self.rgbd_obstacles = self.laser_scan_to_obstacles(msg, self.laser_rgbd_base_dist)

    def odom_input_callback(self, msg):
        #with self.odom_lock:
        self.last_odom_time = msg.header.stamp
        self.odom_vel = msg.twist.twist

    def cmd_callback(self, msg):
        #with self.cmd_lock:
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

        The inflated radius is always greater than the min_inflation_radius,
        because the computed stop distance must the added to the
        external most point of the footprint!.

        Returns:
            float: inflated radius.
        """
        inflated = 0.0
        linear_vel = abs(cmd_vel.linear.x)
        if linear_vel > self.min_linear_vel:
            a = linear_vel * linear_vel
            b = (2.0 * self.linear_decel)
            inflated = a / b
        return self.min_inflation_radius + inflated

    def get_curvature_radius(self, cmd_vel):
        if abs(cmd_vel.angular.z) < self.min_angular_vel:
            return 0
        return abs(cmd_vel.linear.x / cmd_vel.angular.z)

    def get_linear_movement_orientation(self, cmd_vel):
        """
        Returns:
            >  +1 if the cmd_vel moves the robot forwards
            >   0 if the cmd_vel moves does not move the robot (just linear!)
            >  -1 if the cmd_vel moves the robot backwards
        """
        if abs(cmd_vel.linear.x) < self.min_linear_vel:
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
        if abs(cmd_vel.angular.z) < self.min_angular_vel:
            return 0
        if cmd_vel.angular.z > 0:
            return 1
        if cmd_vel.angular.z < 0:
            return -1
        return 0

    def attempts_to_move(self, cmd_vel):
        vx = abs(cmd_vel.linear.x)
        va = abs(cmd_vel.angular.z)
        if (vx > self.min_linear_vel or va > self.min_angular_vel):
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
        r1 = curvature_radius - self.min_inflation_radius
        r2 = curvature_radius + self.min_inflation_radius
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
                if abs(y) > self.min_inflation_radius:
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
            if _is_point_inside_circle(r1, x0, y0, x, y):
                dangerous.append(obstacle)
                continue

            # obstacle must be inside the big circle
            if not _is_point_inside_circle(r2, x0, y0, x, y):
                dangerous.append(obstacle)
                continue
            real.append(obstacle)
        return real, dangerous, candidates

    def get_next_vel(self, v1, v2, accel, decel, min_vel):
        # just no movement
        if abs(v2) < min_vel:
            return 0
        if v1 == v2:
            return v2
        dv = v2 - v1
        if v1 * v2 > 0 and abs(v1) < abs(v2):
            # only forward or backwards and accelerating
            target_accel = accel
        else:
            # TODO: is this right?
            # decelerating: just keep v2
            # Only increment velocity when accelerating.
            # (this is done this way, because of weird approaching behavior)
            # target_accel = decel
            return v2

        # use simulated d_velocity if lesser than actual dv.
        sim_dv = target_accel * self.sim_time
        if abs(sim_dv) < abs(dv):
            dv_sign = dv / abs(dv)
            dv = dv_sign * sim_dv

        vf = v1 + dv
        if vf == 0:
            return 0
        # send a velocity just a little higher than min_vel.
        # this avoids to stay in a infinite loop trying to
        # exit the local
        # Use the target velocity sign!, not the final velocity.
        if abs(vf) <= min_vel:
            vf = min_vel * 1.1 * v2 / abs(v2)
        return vf

    # =========================================================================
    # Main Processing method
    # =========================================================================

    def spin_once(self):

        # work on fixed and consistent data!
        # with self.odom_lock:
        _odom_vel = self.odom_vel
        _odom_time = self.last_odom_time
        # with self.cmd_lock:
        _cmd_vel = self.cmd_vel
        _cmd_time = self.last_cmd_time

        try:
            now = rospy.Time.now()
            is_odom_obsolete = _is_something_obsolete(now, _odom_time, self.odom_timeout)
            is_cmd_obsolete = _is_something_obsolete(now, _cmd_time, self.cmd_timeout)
            is_moving = (not is_cmd_obsolete) and self.attempts_to_move(_odom_vel)
            wants_to_move = (not is_odom_obsolete) and self.attempts_to_move(_cmd_vel)

            # --- odometry is obsolete ---
            if is_odom_obsolete:
                self.send_velocity(Twist())
                rospy.logwarn_throttle(5.0, "Robot odometry is obsolete. Won't move.")
                self.spin_rate.sleep()
                return

            # --- velocity is too small to move ---
            if not is_moving and not wants_to_move:
                # stop the robot... just in case. enforce
                self.send_velocity(Twist())
                rospy.loginfo_throttle(5.0, "Robot is stopped and no command has been issued. Enforcing stop state. This is just informative.")
                self.spin_rate.sleep()
                return

            # --- select target velocity ---
            target_vel = Twist()

            target_vel.linear.x = self.get_next_vel(
                _odom_vel.linear.x,
                _cmd_vel.linear.x,
                self.linear_accel,
                self.linear_decel,
                self.min_linear_vel
            )
            target_vel.angular.z = self.get_next_vel(
                _odom_vel.angular.z,
                _cmd_vel.angular.z,
                self.angular_accel,
                self.angular_decel,
                self.min_angular_vel
            )

            # --- movement properties ---
            linear_orientation = self.get_linear_movement_orientation(target_vel)
            angular_orientation = self.get_angular_movement_orientation(target_vel)
            inflated_radius = self.get_inflated_radius(target_vel)
            curvature_radius = self.get_curvature_radius(target_vel)
            if linear_orientation == 0 and angular_orientation == 0:
                # target velocity is the required velocity is too small, because of a in(de)crement
                self.send_velocity(Twist())
                self.spin_rate.sleep()
                return

            # -- separate obstacles into useful categories --
            # lists of tuples (distance, angle) to obstacles
            with self.sensor_front_lock:
                obstacles = self.front_obstacles
            with self.sensor_rear_lock:
                obstacles += self.rear_obstacles
            with self.sensor_rgbd_lock:
                obstacles += self.rgbd_obstacles

            # get real obstacles vs. not really obstacles.
            real_obstacles, dangerous_obstacles, candidate_obstacles = self.get_obstacle_in_toroid_section(
                obstacles,
                inflated_radius, curvature_radius,
                linear_orientation, angular_orientation
            )

            # -- just for debugging --
            if self.is_debug_enabled:
                curr_linear_orientation = self.get_linear_movement_orientation(_odom_vel)
                curr_angular_orientation = self.get_angular_movement_orientation(_odom_vel)
                req_linear_orientation = self.get_linear_movement_orientation(_cmd_vel)
                req_angular_orientation = self.get_angular_movement_orientation(_cmd_vel)
                remaining_obstacles = len(obstacles) - len(real_obstacles) - len(dangerous_obstacles) - len(candidate_obstacles)

                def linear_to_string(x):
                    return ("FORWARD" if x > 0 else ("BACKWARDS" if x < 0 else "NOT LINEAR"))

                def angular_to_string(a):
                    return ("CLOCKWISE" if a > 0 else ("COUNTER CLOCKWISE" if a < 0 else "NOT ANGULAR"))

                rospy.loginfo_throttle(
                    1.0, ""
                    + "\n> Movement properties:"
                    + "\n  - inflated_radius : %.2f [m]." % (inflated_radius)
                    + "\n  - curvature_radius: %.2f [m]." % (curvature_radius)
                    + "\n> Movement direction:"
                    + "\n  - current  : %s, %s" % (linear_to_string(curr_linear_orientation), angular_to_string(curr_angular_orientation))
                    + "\n  - requested: %s, %s" % (linear_to_string(req_linear_orientation), angular_to_string(req_angular_orientation))
                    + "\n  - target   : %s, %s" % (linear_to_string(linear_orientation), angular_to_string(angular_orientation))
                    + "\n> Velocities:"
                    + "\n  - current  : %.2f[m/s], %.2f[rad/s]." % (_odom_vel.linear.x, _odom_vel.angular.z)
                    + "\n  - required : %.2f[m/s], %.2f[rad/s]." % (_cmd_vel.linear.x, _cmd_vel.angular.z)
                    + "\n  - target   : %.2f[m/s], %.2f[rad/s]." % (target_vel.linear.x, target_vel.angular.z)
                    + "\n> Simulation:"
                    + "\n  - sim_time: %.2f[m/s]." % (self.sim_time)
                    + "\n  - dv accel: %.2f[m/s], %.2f[rad/s]." % (self.sim_time * self.linear_accel, self.sim_time * self.angular_accel)
                    + "\n  - dv decel: %.2f[m/s], %.2f[rad/s]." % (self.sim_time * self.linear_decel, self.sim_time * self.angular_decel)
                    + "\n> There are %d relevant obstacles." % (len(real_obstacles) + len(dangerous_obstacles))
                    + "\n  - %3d real" % (len(real_obstacles))
                    + "\n  - %3d dangerous" % (len(dangerous_obstacles))
                    + "\n  - %3d other" % (len(candidate_obstacles))
                    + "\n  - %3d ignored" % (remaining_obstacles)
                )

            if real_obstacles:
                # Send stop signals!
                self.stop_motion()
                rospy.logwarn_throttle(1.0, "Dangerous input cmd_vel wont be applied!")
            else:
                # The velocity command is safe... send it!
                self.send_velocity(_cmd_vel)

            # visualization
            if self.is_visualization_enabled:
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
        self.stop_triggered_pub.publish(Empty())

    def send_velocity(self, cmd):
        cmd.linear.x = min(cmd.linear.x, self.max_linear_vel)
        cmd.linear.x = max(cmd.linear.x, -self.max_linear_vel)
        cmd.angular.x = min(cmd.angular.x, self.max_angular_vel)
        cmd.angular.x = max(cmd.angular.x, -self.max_angular_vel)
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
        r1 = curvature_radius - self.min_inflation_radius
        r2 = curvature_radius + self.min_inflation_radius

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


def _is_point_inside_circle(r, x0, y0, x, y):
        dx = (x - x0)
        dy = (y - y0)
        if dx * dx + dy * dy <= r * r:
            return True
        return False


def _is_something_obsolete(now, last, timeout):
        if (now - last) > timeout:
            return True
        return False


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
