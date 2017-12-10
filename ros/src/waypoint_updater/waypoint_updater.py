#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import PoseStamped
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


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        # Subscribers
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        # when taking traffic info into account, need two more subscribers here
        # /traffic_waypoint and /current_velocity (for planning waypoints at traffic light)
        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        # Publisher
        # This topic: /final_waypoints is subscribed from node pure_pursuit in waypoint_follower package
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # Initialize important parameters
        self.current_pose = None
        self.waypoints = None
        self.final_waypoints = Lane()

        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement
        self.current_pose = msg.pose
        next_waypoints = self.prepare_lookahead_waypoints()
        if next_waypoints is not None:
            self.publish_final_waypoints(next_waypoints)

    def waypoints_cb(self, msg):
        # TODO: Implement
        # Note that the publisher for /base_waypoints publishes only once
        # so we fill in the data only when it receives no prior waypoints info
        if self.waypoints is None:
            self.waypoints = msg.waypoints

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        # will implement this later
        pass

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
        return math.sqrt(dist_x**2 + dist_y**2)

    def prepare_lookahead_waypoints(self):
        if self.waypoints is None or self.current_pose is None:
            rospy.loginfo("Base waypoint or current pose info are missing. Not publishing waypoints ...")
            return None
        else:
            closest_dist = 1000000000  # initialize with very large value
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
            seq = cycle(self.waypoints)  # loop string of waypoints
            end_pos = closest_wp_pos + LOOKAHEAD_WPS - 1  # to build waypoint list with a fixed size
            next_waypoints = list(islice(seq, closest_wp_pos, end_pos))  # list of lookahead waypoints

            # without considering any traffic
            # let's make the car move forward with constant velocity (10mph)
            # we have to modify this part later
            for i in range(len(next_waypoints)-1):
                target_vel = self.mph_to_mps(10)
                self.set_waypoint_velocity(next_waypoints, i, target_vel)
            return next_waypoints

    def publish_final_waypoints(self, waypoints):
            lane = Lane()
            lane.header.frame_id = '/world'
            lane.header.stamp = rospy.Time(0)
            lane.waypoints = waypoints
            self.final_waypoints_pub.publish(lane)

    def mph_to_mps(self, data):
        return data*0.447

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
