#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'Diego Baño, Matías Pavez'
__email__ = 'diego.bano@ug.uchile.cl, matias.pavez@ing.uchile.cl'

import rospy
from math import sin, cos, atan2, pi, sqrt, pow as mpow, isnan
import numpy
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

    # TODO: callbacks are too CPU heavy 22% CPU
    # TODO: do not process sensors when stopped?
    # TODO: online version
    # TODO: actualizar accel y deccel... usar desde params
    # - previamente inscrito (en caso de puntos ciegos)
    # TODO: reducir la velocidad en vez de dejarla en cero abruptamente
    # TODO: escapar automáticamente?

    # obs:
    # - 0.3/vel = factor de ajusta para la decel.
    # - >= es importante para interceptar mensajes enviados estando al lado de un obst.
    #       TODO: permite seguir avanzando de a poco, hasta eventualmente chocar
    # -
    """

    # COLORS
    COLOR_GREEN = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.5)
    COLOR_ORANGE = ColorRGBA(r=0.9, g=0.5, b=0.2, a=0.5)
    COLOR_RED = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.5)

    def __init__(self):

        # =====================================================================
        # Logic Variables

        # default laser obstacles
        self.laser_front_closest_point = [float("inf"), pi]
        self.laser_rear_closest_point = [float("inf"), pi]
        self.front_obstacles = list()
        self.rear_obstacles = list()

        # default laser positions
        self.laser_front_base_dist = None
        self.laser_rear_base_dist = None

        # clock
        self.spin_rate = rospy.Rate(10)
        self.laser_front_cb_rate = rospy.Rate(10)
        self.laser_rear_cb_rate = rospy.Rate(10)

        # last message
        self.last_msg = Twist()
        self.last_msg_time = rospy.Time.now()

        # ????
        self.prev_front_scan = [0.01] * 1500  # ojo con el size y con los nans/inf/max+1
        self.prev_rear_scan = [0.01] * 500

        # robot cinematic parameters
        self.min_linear_velocity = abs(0.01)   # [m/s] The robot wont move if issued velocity is lesser than this.
        self.max_linear_velocity = abs(0.8)    # [m/s] Used to ignore too far obstacles. And to enforce velocity constraints. TODO
        self.max_angular_velocity = abs(0.8)   # [rad/s] Used to enforce velocity constraints. TODO
        self.linear_deacceleration = abs(0.3)  # m/s/s 0.3 is the nominal value for Pioneer 3AT
        # self.linear_acceleration = abs(0.35)  # m/s/s

        # robot geometry
        self.robot_radius = 0.4

        # sensor parameters
        self.laser_range = pi / 9  # THIS CAN BE RETRIEVED FROM THE LaserScan Message!

        # misc
        self.epsilon = 0.01
        self.max_obstacle_range = 2.0  # ignore obstacles outside this radius
                                       # TODO: use other params to make sure this range is not too low.
        # obs: la simulación puede ser evitada y agilizar los cómputos,
        # computando una sección de toroide por donde se moverá el robot en
        # ese tiempo y a la velocidad deseada.
        self.simulation_time = 0.5
        self.simulation_dt = 0.1

        # Subscriber variables
        self.curr_vel = Twist()
        self.sent_vel = Twist()
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

    def laser_scan_to_obstacles(self, msg, laser_distance, rotation=0.0):
        obstacles = list()
        theta_ = msg.angle_min
        for r in msg.ranges:
            if r < self.max_obstacle_range + laser_distance:
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
                    """
                    real filter
                    """
                    obstacles.append((d, theta + rotation))

            theta_ += msg.angle_increment
        return obstacles

    def laser_front_input_cb_v2(self, msg):
        self.front_obstacles = self.laser_scan_to_obstacles(msg, self.laser_front_base_dist)

    def laser_rear_input_cb_v2(self, msg):
        self.rear_obstacles = self.laser_scan_to_obstacles(msg, self.laser_rear_base_dist, pi)

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
        self.laser_rear_cb_rate.sleep()

    def odom_input_cb(self, msg):
        self.curr_vel = msg.twist.twist

    def velocity_input_cb(self, msg):
        self.sent_vel = msg
        self.sent_linear_vel = msg.linear.x
        self.sent_angular_vel = msg.angular.z

    # =========================================================================
    # Aux methods
    # =========================================================================

    def get_inflated_radius(self):
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
        linear_vel = abs(self.curr_vel.linear.x)
        if linear_vel > self.min_linear_velocity:
            a = mpow(linear_vel, 2)
            b = (2.0 * self.linear_deacceleration)
            inflated = a / b
        return self.robot_radius + inflated

    def get_linear_movement_direction(self, cmd_vel):
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

    def is_the_obstacle_in_front(self, obstacle_angle):
        """
        Returns:
            >  True, if the obstacle in front of the robot
            >  False if the obstacle is behind the robot

        Args:
            obstacle_angle (float): Angle position of the closest obstacle
        """
        if cos(obstacle_angle) > 0:
            return True
        return False

    def predict_movement(self, cmd_vel, dt):
        return Point()

    def is_point_inside_circle(self, center, radius, point):
        return False

    def simulate_movement(self, cmd_vel, obstacles):
        """
        """
        return False

    # =========================================================================
    # Main Processing method
    # =========================================================================

    def spin_once(self):

        try:
            # PS> actualmente hay obstáculos que no son considerados,
            # y que pueden servir para determinar si el robot chocaría o no.??.
            # ... se supone que para eso es lo del radio de curvatura.
            #
            # Ojo.. sólo se consideran obstáculos en el plan deseado.. pero no según la velocidad actual!
            # pues se asume que nunca se dejarán pasar velocidades malas.
            # (sin embargo, puede aparecer un nuevo obstáculo en el laser)

            # list of tuples (distance, angle) to obstacles
            # obstacles = list()
            obstacles = self.front_obstacles + self.rear_obstacles

            # --- filling obstacle list ---
            # from frontal laser (only closest obstacle in desired plan)
            # closest_front_dist = self.laser_front_closest_point[0]
            # closest_front_ang = self.laser_front_closest_point[1]
            # obstacles.append((closest_front_dist, closest_front_ang))

            # Frontal Laser readings (only closest obstacle in desired plan)
            # closest_rear_dist = self.laser_rear_closest_point[0]
            # closest_rear_ang = self.laser_rear_closest_point[1]
            # obstacles.append((closest_rear_dist, closest_rear_ang + pi))

            # --- movement properties ---
            curr_direction = self.get_linear_movement_direction(self.curr_vel)
            req_direction = self.get_linear_movement_direction(self.sent_vel)
            inflated_radius = self.get_inflated_radius()

            # -- separate obstacles into useful categories --
            inscribed_obstacles = list()  # inside robot radius
            dangerous_obstacles = list()  # inside inflated radius but not inscribed
            potential_obstacles = list()  # inside max_obstacle_range radius but neither inflated nor inscribed
            remaining_obstacles = 0

            # just for logging purposes
            for obstacle in obstacles:
                d = obstacle[0]
                if d <= self.robot_radius:
                    inscribed_obstacles.append(obstacle)
                elif d <= inflated_radius:
                    dangerous_obstacles.append(obstacle)
                elif d <= self.max_obstacle_range:
                    potential_obstacles.append(obstacle)
                else:
                    remaining_obstacles += 1

            rospy.loginfo_throttle(
                1.0, ""
                + "\n> Robot radius:"
                + "\n  - baseline: %.2f[m]." % (self.robot_radius)
                + "\n  - inflated: %.2f[m]." % (inflated_radius)
                + "\n> Movement direction:"
                + "\n  - current  : %s." % ("FORWARD" if curr_direction > 0 else ("BACKWARDS" if curr_direction < 0 else "STOPPED"))
                + "\n  - requested: %s." % ("FORWARD" if req_direction > 0 else ("BACKWARDS" if req_direction < 0 else "STOPPED"))
                + "\n> Linear velocities:"
                + "\n  - current : %.2f[m/s]." % (self.curr_vel.linear.x)
                + "\n  - required: %.2f[m/s]." % (self.sent_linear_vel)
                + "\n> There are %d relevant obstacles." % (len(inscribed_obstacles) + len(dangerous_obstacles) + len(potential_obstacles))
                + "\n  - %3d inscribed" % (len(inscribed_obstacles))
                + "\n  - %3d dangerous" % (len(dangerous_obstacles))
                + "\n  - %3d potential" % (len(potential_obstacles))
                + "\n  - %3d ignored" % (remaining_obstacles)
            )

            # visualization
            self.visualize_markers(inflated_radius, inscribed_obstacles, dangerous_obstacles, potential_obstacles)

            # # Uso de sent_vel requiere mutex!
            # # justo podría llegar velocidad distinta!

            is_required_cmd_obsolete = False
            # todo: modify inflation radius ... depends on this
            # TODO: no procesar nada, en caso de que la velocidad sea obsoleta
            # ...s asegurar que no se pierdan mensajes

            if is_required_cmd_obsolete:
                # just stop the robot... no signal required
                self.send_velocity(Twist())
                rospy.logwarn_throttle(
                    5.0,
                    "Required cmd vel is obsolete ... stopping the robot."
                    # + "\n- Current linear velocity: %.2f[m/s]." % (self.curr_vel.linear.x)
                    # + "\n- Expanded current linear velocity: %.2f[m/s] " % (self.curr_vel.linear.x * expansion_factor)
                    # + "\n- Required linear velocity: %.2f[m/s]." % (self.sent_linear_vel)
                    # + "\n- Expanded Required linear velocity: %.2f[m/s] " % (self.sent_linear_vel * expansion_factor)
                    # + "\n- Closest point is %.2f[m] from %s." % (closest, self.base_frame)
                    # + "\n- Robot radius is %.2f[m] (expanded to %.2f[m], factor: %.2f)." % (self.robot_radius, expanded_radius, expansion_factor)
                    # + "\n- Nearest obstacles: (front: %.2f[m]), (rear: %.2f[m])" % (closest_front_dist, closest_rear_dist)
                )
            else:
                # TODO: simulate movement
                generates_collision = False

                if generates_collision:
                    # Send stop signals!
                    self.stop_motion()
                    rospy.logwarn(
                        "Dangerous cmd_vel wont be applied!"
                        # + "\n- Current linear velocity: %.2f[m/s]." % (self.curr_vel.linear.x)
                        # + "\n- Expanded current linear velocity: %.2f[m/s] " % (self.curr_vel.linear.x * expansion_factor)
                        # + "\n- Required linear velocity: %.2f[m/s]." % (self.sent_linear_vel)
                        # + "\n- Expanded Required linear velocity: %.2f[m/s] " % (self.sent_linear_vel * expansion_factor)
                        # + "\n- Closest point is %.2f[m] from %s." % (closest, self.base_frame)
                        # + "\n- Robot radius is %.2f[m] (expanded to %.2f[m], factor: %.2f)." % (self.robot_radius, expanded_radius, expansion_factor)
                        # + "\n- Nearest obstacles: (front: %.2f[m]), (rear: %.2f[m])" % (closest_front_dist, closest_rear_dist)
                    )
                else:
                    # The velocity command is safe... send it!
                    self.send_velocity(self.sent_vel)

            # Check if closest point is inside the safety area,
            # in which case, stop movement if velocity moves the base in that direction
            # will_collide = False
            # could_collide = False
            # if moving_direction != 0 and (closest_rear_dist < self.robot_radius or closest_front_dist < self.robot_radius):

            #     if 0 <= inflated_radius:
            #         pass

            #     # at this point, the obstacle is inside the inflated radius
            #     could_collide = True

            #     wants_to_move_ahead = True if self.sent_linear_vel > 0 else False
            #     wants_to_move_backwards = True if self.sent_linear_vel < 0 else False
            #     if ((moving_direction > 0 and wants_to_move_ahead)
            #        or (moving_direction < 0 and wants_to_move_backwards)):
            #         pass

            #     # now, the requested movement is dangerous!
            #     will_collide = True
            #     rospy.logwarn(
            #         "Dangerous cmd_vel, deleting linear component."
            #         + "\n- Current linear velocity: %.2f[m/s]." % (self.curr_vel.linear.x)
            #         + "\n- Expanded current linear velocity: %.2f[m/s] " % (self.curr_vel.linear.x * expansion_factor)
            #         + "\n- Required linear velocity: %.2f[m/s]." % (self.sent_linear_vel)
            #         + "\n- Expanded Required linear velocity: %.2f[m/s] " % (self.sent_linear_vel * expansion_factor)
            #         + "\n- Closest point is %.2f[m] from %s." % (closest, self.base_frame)
            #         + "\n- Robot radius is %.2f[m] (expanded to %.2f[m], factor: %.2f)." % (self.robot_radius, expanded_radius, expansion_factor)
            #         + "\n- Nearest obstacles: (front: %.2f[m]), (rear: %.2f[m])" % (closest_front_dist, closest_rear_dist)
            #     )
            #     msg = Twist()
            #     msg.angular.z = self.curr_vel.angular.z
            #     self.vel_pub.publish(msg)
            #     self.safety_pub.publish(Empty())

        except Exception as e:
            rospy.logerr("An error occurred. Sending stop cmd_vel to the base. Error description: %s" % e)
            self.stop_motion()

        # self.visualize_markers(expanded_radius, closest, will_collide, could_collide)
        self.spin_rate.sleep()

    # =========================================================================
    # Base motion methods
    # =========================================================================

    def stop_motion(self):
        self.send_velocity(Twist())
        self.safety_pub.publish(Empty())

    def send_velocity(self, cmd):
        self.vel_pub.publish(cmd)

    # =========================================================================
    # Visualization Methods
    # =========================================================================

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

    def visualize_markers(self, inflated_radius, inscribed_obstacles, dangerous_obstacles, potential_obstacles):
        msg = MarkerArray()
        now = rospy.get_rostime()

        # -- inflated_radius marker --
        marker_inflated_radius = self.genmarker_inflated_radius(now, inflated_radius)
        if inscribed_obstacles:
            marker_inflated_radius.color = CmdVelSafety.COLOR_RED
        elif dangerous_obstacles:
            marker_inflated_radius.color = CmdVelSafety.COLOR_ORANGE
        else:
            marker_inflated_radius.color = CmdVelSafety.COLOR_GREEN
        msg.markers.append(marker_inflated_radius)

        # -- inscribed obstacles --
        marker_inscribed_obstacles = self.genmarker_obstacles(now, inscribed_obstacles, CmdVelSafety.COLOR_RED)
        marker_inscribed_obstacles.ns = "safety/inscribed_obstacles"
        msg.markers.append(marker_inscribed_obstacles)

        # -- dangerous obstacles --
        marker_dangerous_obstacles = self.genmarker_obstacles(now, dangerous_obstacles, CmdVelSafety.COLOR_ORANGE)
        marker_dangerous_obstacles.ns = "safety/dangerous_obstacles"
        msg.markers.append(marker_dangerous_obstacles)

        # -- potential obstacles --
        marker_potential_obstacles = self.genmarker_obstacles(now, potential_obstacles, CmdVelSafety.COLOR_GREEN)
        marker_potential_obstacles.ns = "safety/potential_obstacles"
        msg.markers.append(marker_potential_obstacles)

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
