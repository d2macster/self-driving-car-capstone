#!/usr/bin/env python

import rospy
import math
import tf
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Int32
from styx_msgs.msg import Lane, Waypoint
from itertools import islice, cycle
from pid import PID

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200  # Number of waypoints we will publish. You can change this number
BUFFER = 5
MPH_TO_MPS = 0.447


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        # Subscribers
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)

        # when taking traffic info into account, need two more subscribers here
        # /traffic_waypoint and /current_velocity (for planning waypoints at traffic light)
        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        # Publisher
        # This topic: /final_waypoints is subscribed from node pure_pursuit in waypoint_follower package
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        self.driving_mode_pub = rospy.Publisher('driving_mode', Int32, queue_size=1)
        self.car_waypoint_id_pub = rospy.Publisher('car_waypoint_id', Int32, queue_size=1)


        # Initialize important parameters
        self.current_velocity = 0.0
        self.current_pose = None
        self.waypoints = None
        self.traffic = -1
        self.final_waypoints = Lane()

        self.max_velocity = None
        self.target_velocity = None
        self.decel_limit = None
        self.accel_limit = None

        self.is_braking = False
        self.pid_controller = PID(2.0, 0.005, 0.0)
        self.prev_time = None

        self.car_wp_pos = None


        rospy.spin()

    def current_velocity_cb(self, msg):
        self.current_velocity = msg.twist.linear.x

    def pose_cb(self, msg):
        self.current_pose = msg.pose
        next_waypoints = self.prepare_lookahead_waypoints()
        if next_waypoints is not None:
            self.publish_final_waypoints(next_waypoints)

    def waypoints_cb(self, msg):
        # Note that the publisher for /base_waypoints publishes only once
        # so we fill in the data only when it receives no prior waypoints info
        if self.waypoints is None:
            kmph2mps = lambda velocity_kmph: (velocity_kmph * 1000.) / (60. * 60.)
            self.waypoints = msg.waypoints
            self.max_velocity = kmph2mps(rospy.get_param('/waypoint_loader/velocity'))

    def traffic_cb(self, msg):
        self.traffic = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        # leave this part
        # no obstacle in the simulator
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, car_pose, waypoint):
        dist_x = car_pose.position.x - waypoint.pose.pose.position.x
        dist_y = car_pose.position.y - waypoint.pose.pose.position.y
        dist_z = car_pose.position.z - waypoint.pose.pose.position.z
        return math.sqrt(dist_x ** 2 + dist_y ** 2 + dist_z ** 2)

    def closest_waypoint_heading(self, car_pose, waypoints, closest_wp_pos):
        closest_wp_x = waypoints[closest_wp_pos].pose.pose.position.x
        closest_wp_y = waypoints[closest_wp_pos].pose.pose.position.y
        heading = math.atan2(closest_wp_y - car_pose.position.y,
                             closest_wp_x - car_pose.position.x)
        return heading

    def car_pose_heading(self, car_pose):
        RPY = self.RPY_from_quaternion(car_pose.orientation)
        # get yaw angle (last sequence of returning list)
        return RPY[-1]

    def min_stopping_distance(self, velocity):
        t = 1.0*velocity/self.decel_limit
        return 0.5*self.decel_limit*(t**2)

    def get_closest_wp_pos(self):
        if self.waypoints is None or self.current_pose is None:
            rospy.loginfo("Base waypoint or current pose info are missing. Not publishing waypoints ...")
            return (None, None, None)

        L = len(self.waypoints)

        if self.car_wp_pos is not None:
            closest_dist = self.distance(self.current_pose, self.waypoints[self.car_wp_pos % L])
            if closest_dist >= 2.0:
                self.car_wp_pos = None

        if self.car_wp_pos is None:
            # no previous location stored
            # do the brute-force run
            # find waypoint that is closest to the car
            closest_dist = float('inf')  # initialize with very large value
            closest_wp_pos = None  # closest waypoint's position (among the waypoint list)
            for i in range(len(self.waypoints)):
                # extract waypoint coordinates
                waypoint = self.waypoints[i]
                # calculate distance between the car's pose to each waypoint
                dist = self.distance(self.current_pose, waypoint)
                # search for position of closest waypoint to the car
                if dist < closest_dist:
                    closest_dist = dist
                    closest_wp_pos = i

            # check if we already passed the closest waypoint we found
            # get heading of closest waypoint
            theta_waypoint = self.closest_waypoint_heading(self.current_pose, self.waypoints, closest_wp_pos)
            # get heading of car (current pose)
            theta_car = self.car_pose_heading(self.current_pose)
            # check if we should skip the current closest waypoint (in case we passed it already)
            diff_angle = math.fabs(theta_car - theta_waypoint)

            if diff_angle > math.pi / 4.0:
                # skip to next closest waypoint
                closest_wp_pos += 1

            self.car_wp_pos = closest_wp_pos

            return (self.car_wp_pos, closest_dist, diff_angle)

        # start from the last best position
        closest_dist = self.distance(self.current_pose, self.waypoints[self.car_wp_pos % L])
        theta_waypoint = self.closest_waypoint_heading(self.current_pose, self.waypoints, self.car_wp_pos)
        theta_car = self.car_pose_heading(self.current_pose)
        diff_angle = math.fabs(theta_car - theta_waypoint)

        for i in range(len(self.waypoints)):
            p_dist = self.distance(self.current_pose, self.waypoints[(self.car_wp_pos + 1) % L])
            if p_dist < closest_dist:
                self.car_wp_pos = (self.car_wp_pos + 1) % L
                closest_dist = p_dist
                theta_waypoint = self.closest_waypoint_heading(self.current_pose, self.waypoints, self.car_wp_pos)
                theta_car = self.car_pose_heading(self.current_pose)
                diff_angle = math.fabs(theta_car - theta_waypoint)
            else:
                break

        if diff_angle > math.pi / 4.0:
            # skip to next closest waypoint
            self.car_wp_pos = (self.car_wp_pos + 1) % L

        return (self.car_wp_pos, closest_dist, diff_angle)


    def prepare_lookahead_waypoints(self):
        # reading parameters set for dbw_node
        if self.decel_limit is None or self.accel_limit is None:
            self.accel_limit = math.fabs(rospy.get_param('/dbw_node/accel_limit'))
            self.decel_limit = math.fabs(rospy.get_param('/dbw_node/decel_limit'))
            return None

        closest_wp_pos, closest_dist, diff_angle = self.get_closest_wp_pos()
        if closest_wp_pos is None:
            return None

        # how far are we from the target waypoint path
        waypoint_error = math.fabs(closest_dist * math.sin(diff_angle))

        if self.prev_time:
            # we have previous time tick
            delta_t = float(rospy.get_time() - self.prev_time)
            waypoint_error = self.pid_controller.step(error=waypoint_error, sample_time=delta_t)
        self.target_velocity = max(MPH_TO_MPS * 5.0, self.max_velocity - waypoint_error)
        self.target_velocity = min(self.target_velocity, self.max_velocity)
        self.prev_time = rospy.get_time()

        # create list of waypoints starting from index: closest_wp_ros
        seq = cycle(self.waypoints)  # loop string of waypoints
        end_pos = closest_wp_pos + LOOKAHEAD_WPS - 1  # to build waypoint list with a fixed size
        next_waypoints = list(islice(seq, closest_wp_pos, end_pos))  # list of lookahead waypoints

        if self.traffic == -1:
            # no red light : lets accelerate
            self.is_braking = False
            self.speed_up(next_waypoints=next_waypoints)
        else:
            # should we stop?
            # do we even have enough distance to the traffic light to stop the car?
            tl_dist = self.distance(self.current_pose, self.waypoints[self.traffic])

            if tl_dist >= self.min_stopping_distance(self.current_velocity):
                self.is_braking = True

            if self.is_braking:
                # yes we have determined we have enough distance to stop
                self.slow_down(tl_dist=tl_dist, next_waypoints=next_waypoints)
            else:
                self.speed_up(next_waypoints=next_waypoints)

        return next_waypoints

    def speed_up(self, next_waypoints):
        vel = self.current_velocity
        for i in range(len(next_waypoints) - 1):
            if i == 0:
                dist = self.distance(self.current_pose, next_waypoints[0])
            else:
                dist = self.distance(next_waypoints[i - 1].pose.pose, next_waypoints[i])

            vel += self.accel_limit * dist
            vel = min(self.target_velocity, vel)
            vel = max(0.0, vel)
            self.set_waypoint_velocity(next_waypoints, i, vel)


    def slow_down(self, tl_dist, next_waypoints):
        tl_dist_effective = max(1.0, tl_dist - BUFFER)

        vel = self.current_velocity
        decel = min(self.decel_limit, vel / tl_dist_effective)

        # if we are not close enough to the traffic light,
        # allow some slow advancement towards the goal
        min_v = MPH_TO_MPS * 3.0 if tl_dist > BUFFER else 0.0

        for i in range(len(next_waypoints) - 1):
            if i == 0:
                dist = self.distance(self.current_pose, next_waypoints[0])
            else:
                dist = self.distance(next_waypoints[i - 1].pose.pose, next_waypoints[i])
            vel -= decel * dist
            vel = max(min_v, vel)

            if vel <= MPH_TO_MPS:
                vel = 0.0
            self.set_waypoint_velocity(next_waypoints, i, vel)

    def publish_final_waypoints(self, waypoints):
        lane = Lane()
        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time(0)
        lane.waypoints = waypoints
        self.final_waypoints_pub.publish(lane)

        self.driving_mode_pub.publish(-1 if self.is_braking else 1)

        if self.car_wp_pos is not None:
            self.car_waypoint_id_pub.publish(self.car_wp_pos)


    def RPY_from_quaternion(self, orientation):
        x, y, z, w = orientation.x, orientation.y, orientation.z, orientation.w
        return tf.transformations.euler_from_quaternion([x, y, z, w])


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
