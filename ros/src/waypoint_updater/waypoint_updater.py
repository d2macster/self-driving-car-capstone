#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math

# PoseStamped message format
#
# std_msgs/Header header
#   uint32 seq
#   time stamp
#   string frame_id
# geometry_msgs/Pose pose
#   geometry_msgs/Point position
#     float64 x
#     float64 y
#     float64 z
#   geometry_msgs/Quaternion orientation
#     float64 x
#     float64 y
#     float64 z
#     float64 w

# Lane message format
#
# std_msgs/Header header
#   uint32 seq
#   time stamp
#   string frame_id
# styx_msgs/Waypoint[] waypoints
#   geometry_msgs/PoseStamped pose
#     std_msgs/Header header
#       uint32 seq
#       time stamp
#       string frame_id
#     geometry_msgs/Pose pose
#       geometry_msgs/Point position
#         float64 x
#         float64 y
#         float64 z
#       geometry_msgs/Quaternion orientation
#         float64 x
#         float64 y
#         float64 z
#         float64 w
#   geometry_msgs/TwistStamped twist
#     std_msgs/Header header
#       uint32 seq
#       time stamp
#       string frame_id
#     geometry_msgs/Twist twist
#       geometry_msgs/Vector3 linear
#         float64 x
#         float64 y
#         float64 z
#       geometry_msgs/Vector3 angular
#         float64 x
#         float64 y
#         float64 z

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

        # /current_pose : geometry_msgs/PoseStamped : Current position of the vehicle, provided by the simulator or localization.
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)

        # /base_waypoints : styx_msgs/Lane : Waypoints as provided by a static .csv file.
        rospy.Subscriber('/base_waypoints', Lane, self.base_waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below

        rate = rospy.Rate(10)  # 10hz
        while not rospy.is_shutdown():
            rate.sleep()
            self.loop()

    def loop(self):

        if hasattr(self, 'base_waypoints') and hasattr(self, 'current_pose'):
            pose = self.current_pose
            wpts = self.base_waypoints.waypoints

            print(pose)

            lane = Lane()
            final_waypoints = []
            # This is a subset of /base_waypoints.
            # The first waypoint is the one in /base_waypoints
            # which is closest to the car.

            for i in range(LOOKAHEAD_WPS):
                wp = Waypoint()
                # FIRST we need to figure out what is the starting index for waypoints
                # wp.pose.pose.position.x = waypoints[index].pose.pose.position.x
                # wp.pose.pose.position.y = waypoints[index].pose.pose.position.y
                # wp.pose.pose.position.z = waypoints[index].pose.pose.position.z
                # wp.pose.pose.orientation = waypoints[index].pose.pose.orientation
                # wp.twist.twist.linear.x = waypoints[index].twist.twist.linear.x
                final_waypoints.append(wp)

            lane.header.stamp = rospy.Time().now()
            lane.header.frame_id = "/world"
            lane.waypoints = final_waypoints
            self.final_waypoints_pub.publish(lane)

    def pose_cb(self, msg):
        self.current_pose = msg

    def base_waypoints_cb(self, waypoints):
        # The /base_waypoints topic publishes a list of all waypoints for the track,
        # so this list includes waypoints both before and after the vehicle
        # (note that the publisher for /base_waypoints publishes only once).
        self.base_waypoints = waypoints

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)
        for i in range(wp1, wp2 + 1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def get_closest_waypoint(self, pose, waypoints):

        closest_dist = float('inf')
        closest_wp = 0

        dl = lambda a, b: math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)
        for i in range(len(waypoints)):
            dist = dl(pose.pose.position, waypoints[i].pose.pose.position)
            if dist < closest_dist:
                closest_dist = dist
                closest_wp = i

        return closest_wp


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
