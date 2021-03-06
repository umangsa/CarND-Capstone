#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

import math
import tf
import copy
from collections import deque

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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
# TODO: Link this to waypoint_loader value.
MAX_DECEL = 3.0

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/obstacle_waypoint', Lane, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.base_waypoints = None
        self.orig_wp_vel = []
        self.pose = None
        self.current_waypoint = None
        self.traffic_waypoint = None

        #rospy.spin()
        self.loop()

    def loop(self):
        rate = rospy.Rate(20) # 50Hz
        while not rospy.is_shutdown():
            self.generate_waypoints()
            rate.sleep()

    def generate_waypoints(self):
        # find the nearest waypoint ahead
        if self.base_waypoints and self.pose:
            closest_wp = self.get_closest_waypoint(self.pose, self.base_waypoints)

            # find the next waypoint
            orig_closest_wp = closest_wp
            for limit_i in range(10):
                map_x = self.base_waypoints[closest_wp].pose.pose.position.x
                map_y = self.base_waypoints[closest_wp].pose.pose.position.y
                heading = math.atan2( (map_y - self.pose.position.y), (map_x - self.pose.position.x) )
                quaternion = (
                    self.pose.orientation.x,
                    self.pose.orientation.y,
                    self.pose.orientation.z,
                    self.pose.orientation.w)
                euler = tf.transformations.euler_from_quaternion(quaternion)
                theta = euler[2] # Yaw angle = Heading angle
                angle = abs(theta - heading)

                if abs(angle) > math.pi/2:
                    closest_wp = (closest_wp + 1) % len(self.base_waypoints)
                else:
                    break

            self.current_waypoint = closest_wp
            #rospy.loginfo('Orig closest Waypoint {}/{}, updated {} @ angle {}deg'.format(orig_closest_wp,
            #    len(self.base_waypoints), closest_wp, angle*180/math.pi))

            # condition to handle if the traffic waypoint for red was in the past. Should not happen
            # if (self.traffic_waypoint != -1) and (self.current_waypoint > self.traffic_waypoint):
            #     self.reset_waypoints_velocity()

            # now get the list of waypoints that we want to publish
            final_waypoints = []
            index = closest_wp
            for orig_i in range(closest_wp, closest_wp + LOOKAHEAD_WPS):
                i = orig_i % len(self.base_waypoints)
                wp = Waypoint()
                # wp.pose.pose.position = copy.deepcopy(self.base_waypoints[i].pose.pose.position)
                # wp.pose.pose.orientation = copy.deepcopy(self.base_waypoints[i].pose.pose.orientation)
                # wp.twist.twist = copy.deepcopy(self.base_waypoints[i].twist.twist)

                wp.pose.pose.position.x = self.base_waypoints[i].pose.pose.position.x
                wp.pose.pose.position.y = self.base_waypoints[i].pose.pose.position.y
                wp.pose.pose.position.z = self.base_waypoints[i].pose.pose.position.z

                wp.pose.pose.orientation.x = self.base_waypoints[i].pose.pose.orientation.x
                wp.pose.pose.orientation.y = self.base_waypoints[i].pose.pose.orientation.y
                wp.pose.pose.orientation.z = self.base_waypoints[i].pose.pose.orientation.z
                wp.pose.pose.orientation.w = self.base_waypoints[i].pose.pose.orientation.w

                wp.twist.twist.linear.x = self.base_waypoints[i].twist.twist.linear.x
                wp.twist.twist.linear.y = self.base_waypoints[i].twist.twist.linear.y
                wp.twist.twist.linear.z = self.base_waypoints[i].twist.twist.linear.z

                wp.twist.twist.angular.x = self.base_waypoints[i].twist.twist.angular.x
                wp.twist.twist.angular.y = self.base_waypoints[i].twist.twist.angular.y
                wp.twist.twist.angular.z = self.base_waypoints[i].twist.twist.angular.z
                
                final_waypoints.append(wp)

            # publish final waypoints
            lane = Lane()
            lane.header.stamp = rospy.Time.now()
            # lane.header.frame_id = msg.header.frame_id
            # lane.header.seq = msg.header.seq
            lane.waypoints = final_waypoints
            self.final_waypoints_pub.publish(lane)

            self.pose = None

    def get_closest_waypoint(self, pose, waypoints):
        wp_distance = 100000.0
        closest_wp = 0

        a = pose.position

        for i in range(len(waypoints)):
            b = waypoints[i].pose.pose.position
            dist = math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) **2)
            if dist < wp_distance: # and (i < len(waypoints) - 5):
                wp_distance = dist
                closest_wp = i

        return closest_wp

    def reset_waypoints_velocity(self):
        for wp_i in range(0,len(self.base_waypoints)):
            self.set_waypoint_velocity(wp_i, self.orig_wp_vel[wp_i])

    def brakeBeforeTrafficLight(self):
        # Clear any existing maneuvers.
        # TODO: Check for race condition here.
        #self.reset_waypoints_velocity()

        #rospy.loginfo('Dist to traffic light: {}'.format(self.distance(self.base_waypoints, self.current_waypoint, self.traffic_waypoint)))

        # Decelerate algorithm similar to waypoint_loader.decelerate().
        self.set_waypoint_velocity(self.traffic_waypoint, 0)
        # Distance before stopping point to aim to stop.
        brakeMargin = 10
        dist = 0
        for wp in range(self.traffic_waypoint-1, self.current_waypoint, -1):
            dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
            # Approximation: Assume straight path to traffic light.
            #dist = dl(wp.pose.pose.position, self.base_waypoints[self.traffic_waypoint].pose.pose.position)
            dist += dl(self.base_waypoints[wp].pose.pose.position, self.base_waypoints[wp+1].pose.pose.position)
            if dist < brakeMargin:
                vel = 0
            else:
                vel = math.sqrt(2 * MAX_DECEL * (dist - brakeMargin))
                if vel < 1:
                    vel = 0
            self.set_waypoint_velocity(wp, min(vel, self.get_waypoint_velocity(wp)))

    def pose_cb(self, msg):
        self.pose = msg.pose
        #self.generate_waypoints()

    def waypoints_cb(self, waypoints):
        rospy.loginfo('Received waypoints - number of waypoints {}'.format(len(waypoints.waypoints)))
        self.base_waypoints = copy.deepcopy(waypoints.waypoints)
        self.orig_wp_vel = []
        for wp_i in range(0,len(self.base_waypoints)):
            self.orig_wp_vel.append(self.get_waypoint_velocity(wp_i))

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        if msg.data == -1 and self.traffic_waypoint is not None:
            self.reset_waypoints_velocity()
            self.traffic_waypoint = None
            #rospy.loginfo('D2: traffic_cb clear traffic light. wp 318 vel: {}'.format(self.get_waypoint_velocity(318)))
        else:
            self.traffic_waypoint = msg.data
            self.brakeBeforeTrafficLight()
            #rospy.loginfo('D3: traffic_cb rcv traffic light: {}, wp 318 vel {}'.format(msg.data, self.get_waypoint_velocity(318)))

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return self.base_waypoints[waypoint].twist.twist.linear.x

    def set_waypoint_velocity(self, waypoint, velocity):
        self.base_waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2):
            dist += dl(waypoints[i].pose.pose.position, waypoints[i+1].pose.pose.position)
        return dist

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
