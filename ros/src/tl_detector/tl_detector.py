#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
from light_classification.tl_classifier_carla import TLClassifierCarla

import tf
import math
import yaml
import os

STATE_COUNT_THRESHOLD = 3
LOOKAHEAD_WPS = 200

DIR_PATH = os.path.dirname(os.path.realpath(__file__))

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.waypoints_L = None
        self.camera_image = None
        # self.lights = []

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/image_color', Image, self.image_cb)
        rospy.Subscriber('/car_waypoint_id', Int32, self.car_waypoint_id_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.classifier_model = rospy.get_param("classification_model")

        self.bridge = CvBridge()
        if self.classifier_model == "sim_classifier":
            self.light_classifier = TLClassifier()
        else:
            self.light_classifier = TLClassifierCarla()

        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        self.car_waypoint_id = None

        self.stop_line_cache = []

        stop_line_positions = self.config['stop_line_positions']

        for index, stop_line in enumerate(stop_line_positions):
            stop_line_wp = self.get_closest_waypoint(stop_line)
            self.stop_line_cache.append((index, stop_line, stop_line_wp))

        rospy.spin()

    def car_waypoint_id_cb(self, msg):
        self.car_waypoint_id = msg.data

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
        self.waypoints_L = len(self.waypoints.waypoints)

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg

        light_wp, state = self.process_traffic_lights()


        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED or state == TrafficLight.YELLOW else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint(self, pose):
        if self.waypoints is None or pose is None:
            rospy.loginfo("Base waypoint or current pose info are missing. Cannot find nearest waypoints ...")
            return None
        else:
            position_x, position_y = pose[0], pose[1]
            closest_index = None
            min_dist = 99999

            for i in range(self.waypoints_L):
                waypoint_x, waypoint_y = self.waypoints.waypoints[i].pose.pose.position.x, self.waypoints.waypoints[i].pose.pose.position.y
                dist = (waypoint_x - position_x)**2 + (waypoint_y - position_y)**2
                if dist < min_dist:
                    min_dist = dist
                    closest_index = i
        return closest_index

    def get_light_state(self):
        """Determines the current color of the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False
        self.camera_image.encoding = "rgb8"

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light_wp = None

        if(self.pose and self.waypoints and self.car_waypoint_id):

            for index, stop_line, stop_line_wp in self.stop_line_cache:
                num_wp_ahead = stop_line_wp - self.car_waypoint_id
                if num_wp_ahead < 0:
                    num_wp_ahead += self.waypoints_L

                if (num_wp_ahead < 200):
                    # only try to classify lights that are a reasonable distance away
                    if ( math.sqrt( (self.pose.pose.position.x - stop_line[0])**2 + (self.pose.pose.position.y - stop_line[1])**2) < 100 ):
                        light_wp = stop_line_wp

        if light_wp is not None:
            state = self.get_light_state()
            # for saving imgs from sim
            # self.camera_image.encoding = "rgb8";
            # cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")
            # rand_img_int = randint(0,100)
            # cv2.imwrite(DIR_PATH + '/light_classification/data/camera_image' + str(state) + "-" + str(rand_img_int) + '.jpg', cv_image)
            return light_wp, state
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
