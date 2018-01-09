#!/usr/bin/env python
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

STATE_COUNT_THRESHOLD = 3 

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []


        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        self.log_publish = True

        self.tl_waypoints = None

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb, queue_size=1)
        
        rospy.spin()

    def get_tl_waypoints(self):
        rospy.logerr("tl_detector get_tl_waypoints()")
        stop_line_positions = self.config['stop_line_positions']
        self.tl_waypoints = []
        for light_stop_position in stop_line_positions:
            light_stop_pose = Pose()
            light_stop_pose.position.x = light_stop_position[0]
            light_stop_pose.position.y = light_stop_position[1]
            light_stop_wp = self.get_closest_waypoint(light_stop_pose)     #get the wp closest to each light_position
            self.tl_waypoints.append(light_stop_wp)

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints.waypoints
        self.get_tl_waypoints()

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
        light_wp, state, car_position = self.process_traffic_lights()
        # print("tl_detector light_wp = {}, state = {}".format(light_wp, state))
        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            if self.last_state == TrafficLight.RED and state == TrafficLight.UNKNOWN:
                pass
            else:
                self.state_count = 0
                self.state = state
                rospy.logerr("tl_detector car position: {} state = {}".format(car_position, self.state))
        elif self.last_state == TrafficLight.RED and self.state_count >= STATE_COUNT_THRESHOLD * 2:
            if self.last_state != self.state:
                self.log_publish = True
            self.last_state = self.state
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
            if self.log_publish:
                self.log_publish = False
                rospy.logerr("tl_detector publish car position: {} light_wp = {} state = {}".format(car_position, light_wp, self.state))
        elif self.last_state != TrafficLight.RED and self.state_count >= STATE_COUNT_THRESHOLD:
            if self.last_state != self.state:
                self.log_publish = True
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
            if self.log_publish:
                self.log_publish = False
                rospy.logerr("tl_detector publish car position: {} light_wp = {} state = {}".format(car_position, light_wp, self.state))
        # else:
        #     print("tl_detector: light = {}".format(Int32(self.last_wp)))
        #     self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
        if self.waypoints is None:
            return
        #TODO implement
        min_dist = 10000
        min_loc = None

        pos_x = pose.position.x
        pos_y = pose.position.y
        # check all the waypoints to see which one is the closest to our current position
        for i, waypoint in enumerate(self.waypoints):
            wp_x = waypoint.pose.pose.position.x
            wp_y = waypoint.pose.pose.position.y
            dist = math.sqrt(math.pow((pos_x - wp_x), 2) + math.pow((pos_y - wp_y), 2))
            if (dist < min_dist): #we found a closer wp
                min_loc = i     # we store the index of the closest waypoint
                min_dist = dist     # we save the distance of the closest waypoint

        # returns the index of the closest waypoint
        return min_loc

    def get_light_state(self):
        """Determines the current color of the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

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
        light = None

        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose)

        if (self.pose is None) or (self.tl_waypoints is None):
            return -1, TrafficLight.UNKNOWN, car_position

        closest_light_stop_wp = None
        dist_to_light = 10000

        for light_wp in self.tl_waypoints:
            if light_wp >= car_position:
                if closest_light_stop_wp is None:
                    closest_light_stop_wp = light_wp
                elif light_wp < closest_light_stop_wp:
                    closest_light_stop_wp = light_wp

        if (car_position is None) or (closest_light_stop_wp is None):
            return -1, TrafficLight.UNKNOWN, car_position

        dist_to_light = abs(car_position - closest_light_stop_wp)

        if dist_to_light < 150:
            state = self.get_light_state()
            return closest_light_stop_wp, state, car_position

        return -1, TrafficLight.UNKNOWN, car_position


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')

