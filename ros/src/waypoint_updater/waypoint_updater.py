#!/usr/bin/env python

import rospy
import math
import tf
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Int32
from styx_msgs.msg import Lane, Waypoint
from itertools import islice, cycle

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

        # Initialize important parameters
        self.current_velocity = 0.0
        self.current_pose = None
        self.waypoints = None
        self.traffic = -1
        self.final_waypoints = Lane()

        self.max_velocity = None
        self.decel_limit = None
        self.accel_limit = None

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

    def prepare_lookahead_waypoints(self):
        if self.decel_limit is None or self.accel_limit is None:
            self.accel_limit = math.fabs(rospy.get_param('/dbw_node/accel_limit'))
            self.decel_limit = math.fabs(rospy.get_param('/dbw_node/decel_limit'))
            return None

        if self.waypoints is None or self.current_pose is None:
            rospy.loginfo("Base waypoint or current pose info are missing. Not publishing waypoints ...")
            return None
        else:
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
            # rospy.loginfo("Theta Waypoint: %.3f, Theta Car: %.3f, Diff: %.3f" % (theta_waypoint, theta_car, diff_angle))
            if diff_angle > math.pi / 4.0:
                # skip to next closest waypoint
                # rospy.loginfo("closest waypoint skipped. Next closest one picked ...")  # debug only
                closest_wp_pos += 1
            # else:
            # rospy.loginfo("current closest waypoint maintained ...")  # debug only

            # create list of waypoints starting from index: closest_wp_ros
            seq = cycle(self.waypoints)  # loop string of waypoints
            end_pos = closest_wp_pos + LOOKAHEAD_WPS - 1  # to build waypoint list with a fixed size
            next_waypoints = list(islice(seq, closest_wp_pos, end_pos))  # list of lookahead waypoints

            if self.traffic == -1:
                # no red light : lets accelerate
                vel = self.current_velocity

                for i in range(len(next_waypoints) - 1):
                    if i == 0:
                        dist = self.distance(self.current_pose, next_waypoints[0])
                    else:
                        dist = self.distance(next_waypoints[i - 1].pose.pose, next_waypoints[i])

                    vel += self.accel_limit * dist
                    vel = min(self.max_velocity, vel)
                    vel = max(0.0, vel)
                    self.set_waypoint_velocity(next_waypoints, i, vel)
            else:
                # slowing down
                tl_dist = max(1.0, self.distance(self.current_pose, self.waypoints[self.traffic]) - BUFFER)

                vel = self.current_velocity
                decel = min(self.decel_limit, vel / tl_dist)

                for i in range(len(next_waypoints) - 1):
                    if i == 0:
                        dist = self.distance(self.current_pose, next_waypoints[0])
                    else:
                        dist = self.distance(next_waypoints[i - 1].pose.pose, next_waypoints[i])
                    vel -= decel * dist
                    if vel <= 0.447:
                        vel = 0.0
                    self.set_waypoint_velocity(next_waypoints, i, vel)

            return next_waypoints

    def publish_final_waypoints(self, waypoints):
        lane = Lane()
        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time(0)
        lane.waypoints = waypoints
        self.final_waypoints_pub.publish(lane)

    def mph_to_mps(self, data):
        return data * 0.447

    def RPY_from_quaternion(self, orientation):
        x, y, z, w = orientation.x, orientation.y, orientation.z, orientation.w
        return tf.transformations.euler_from_quaternion([x, y, z, w])


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
