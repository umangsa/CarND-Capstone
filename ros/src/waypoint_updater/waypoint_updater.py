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
        self.base_raw_waypoints = None
        self.pose = None
        self.current_waypoint = None
        self.traffic_waypoint = None

        # rospy.spin()
        self.loop()

    def loop(self):
        rate = rospy.Rate(50) # 50Hz
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
            rospy.loginfo('Orig closest Waypoint {}/{}, updated {} @ angle {}deg'.format(orig_closest_wp,
                len(self.base_waypoints), closest_wp, angle*180/math.pi))

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
        self.base_waypoints = self.base_raw_waypoints

    def brakeBeforeTrafficLight(self):
        # Distance from traffic light to start braking.
        brakeStartDist = 20
        # Distance from traffic light (or stop line) to come to a stop.
        brakeStopDist = 0

        # Stepping back from traffic waypoint, find waypoints where braking
        # should start and finish.
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)   # Copied from distance()
        dist = 0
        brakeStartWp = None
        brakeStopWp = None
        assert(brakeStartDist > brakeStopDist)
        # TODO: Make sure that waypoint indices always increase monotonically.
        # For example, if car is driving in a continuous loop then at some point
        # the waypoints might wrap around.
        #rospy.loginfo('D5: brakeBeforeTrafficLight wps {}, {}'.format(self.traffic_waypoint, self.current_waypoint))
        for wp in range(self.traffic_waypoint, self.current_waypoint, -1):
            dist += dl(self.base_waypoints[wp].pose.pose.position, self.base_waypoints[wp+1].pose.pose.position)
            if (brakeStopWp is None) and (dist >= brakeStopDist):
                brakeStopWp = wp
            brakeStartWp = wp
            if dist >= brakeStartDist:
                break

        #rospy.loginfo('D4: brakeBeforeTrafficLight start {} stop {}'.format(brakeStartWp, brakeStopWp))

        # Sanity checks. Example: if traffic waypoint is at very short range, perhaps
        # even the current waypoint, then above search is nonsensical. In those cases,
        # do not attempt to brake.
        if (brakeStopWp is not None) and (brakeStartWp is not None) and (brakeStartWp < brakeStopWp):
            self.brakeBetweenWaypoints(brakeStartWp, brakeStopWp)

    def brakeBetweenWaypoints(self, brakeStartWp, brakeStopWp):
        init_velocity = self.get_waypoint_velocity(self.base_raw_waypoints[brakeStartWp])

        # TODO: Determine units here. m/s? mph?
        brakeDist = self.distance(self.base_raw_waypoints, brakeStartWp, brakeStopWp)
        decel = init_velocity / brakeDist
        rospy.loginfo('Planning brake across wp {} to {}, dist {}, deceleration {}'.format(brakeStartWp, brakeStopWp, brakeDist, decel))

        # Clear any existing maneuvers.
        # TODO: Check for race condition here.
        self.reset_waypoints_velocity()
        for wp in range(brakeStartWp, brakeStopWp):
            # Linear interpolation
            # TODO: Improve this math s.t. stopping point is right at light.
            new_vel = init_velocity * (wp-brakeStartWp) / (brakeStopWp-brakeStartWp)
            if new_vel <= 1:
                new_vel = 0
            self.set_waypoint_velocity(self.base_waypoints, wp, new_vel)

    def pose_cb(self, msg):
        self.pose = msg.pose

    def waypoints_cb(self, waypoints):
        rospy.loginfo('Received waypoints - number of waypoints {}'.format(len(waypoints.waypoints)))
        self.base_raw_waypoints = waypoints.waypoints
        self.base_waypoints = self.base_raw_waypoints

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        if msg.data == -1:
            #rospy.loginfo('D2: traffic_cb clear traffic light')
            self.reset_waypoints_velocity()
            self.traffic_waypoint = None
        else:
            #rospy.loginfo('D3: traffic_cb rcv traffic light: {}'.format(msg.data))
            self.traffic_waypoint = msg.data
            self.brakeBeforeTrafficLight()

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

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
