#!/usr/bin/env python
from pickle import NONE
from numpy.lib.function_base import diff
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math
from scipy.spatial import KDTree
STATE_COUNT_THRESHOLD = 3


class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray,
                                self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint',
                                                      Int32,
                                                      queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        self.waypoint_tree = None
        self.waypoint_2d = None
        rospy.spin()

    def calc_distance(self, a, b):
        return math.sqrt((a.x - b.x)**2 + (a.y - b.y)**2)

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
        if not self.waypoint_2d:
            self.waypoint_2d = [[
                waypoint.pose.pose.position.x, waypoint.pose.pose.position.y
            ] for waypoint in waypoints.waypoints]
        self.waypoint_tree = KDTree(self.waypoint_2d)

    def traffic_cb(self, msg):
        self.lights = msg.lights

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
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
            rospy.loginfo("traffic write state= %s", light_wp)
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints
        min_dist = 999999
        min_index = 0
        for i in range(len(self.waypoints.waypoints)):
            dist = self.calc_distance(self.waypoints.waypoints[
                i].pose.pose.position, pose.position)
            if (dist < min_dist):
                min_dist = dist
                min_index = i
        if (self.waypoints.waypoints[min_index].pose.pose.position.x < pose.position.x):
            min_index += 1
        """
        # TODO implement

        min_index = self.waypoint_tree.query(
            [pose.position.x, pose.position.y], 1)[1]
        return min_index

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        # Get classification
        return self.light_classifier.get_classification(cv_image)
        """
        return light.state

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        cloest_light = None
        line_index = None
        if (self.pose and self.waypoints != None):
            car_position = self.get_closest_waypoint(self.pose.pose)

            # TODO find the closest visible traffic light (if one exists)
            diff = len(self.waypoints.waypoints)
            for i, light in enumerate(self.lights):
                line = stop_line_positions[i]
                light_pose = Pose()
                light_pose.position.x = line[0]
                light_pose.position.y = line[1]
                temp_index = self.get_closest_waypoint(light_pose)
                d = temp_index - car_position

                if ((d >= 0) and (d < diff)):
                    diff = d
                    cloest_light = light
                    line_index = temp_index
            """rospy.loginfo("light_pose= %s", car_position + diff)
            rospy.loginfo("car_position= %s", car_position)
            rospy.loginfo("diff= %s", diff)
            rospy.loginfo("light= %s", light.state)"""

        if cloest_light != None:
            rospy.loginfo("cloest_light state= %s", cloest_light.state)
            state = self.get_light_state(cloest_light)
            return line_index, state
        self.waypoints = None
        return -1, TrafficLight.UNKNOWN


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')