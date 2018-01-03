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
        self.pose = None
        self.current_waypoint = -1
        self.traffic_waypoint = -1
        self.original_velocity = deque()

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
            self.current_waypoint = closest_wp

            # find the next waypoint
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

            if(angle > math.pi / 4.0):
                closest_wp += 1

            # condition to handle if the traffic waypoint for red was in the past. Should not happen
            # if (self.traffic_waypoint != -1) and (self.current_waypoint > self.traffic_waypoint):
            #     self.reset_waypoints_velocity()

            # now get the list of waypoints that we want to publish
            final_waypoints = []
            index = closest_wp
            for i in range(closest_wp, closest_wp + LOOKAHEAD_WPS):
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
            if dist < wp_distance:
                wp_distance = dist
                closest_wp = i

        return closest_wp

    def reset_waypoints_velocity(self):
        for i in range(len(self.original_velocity)):
            wp = self.original_velocity.popleft()
            self.set_waypoint_velocity(self.base_waypoints, wp[0], wp[1])

    def slow_down_car(self):
        current_velocity = self.get_waypoint_velocity(self.base_waypoints[self.current_waypoint])
        delta_vel = current_velocity / float(self.traffic_waypoint - self.current_waypoint)
        new_vel = current_velocity - delta_vel

        for i in range(self.current_waypoint, self.traffic_waypoint):
            self.original_velocity.append((i, self.get_waypoint_velocity(self.base_waypoints[i])))
            self.set_waypoint_velocity(self.base_waypoints, i, new_vel)
            new_vel -= delta_vel
            if new_vel <= 1.:
                new_vel = 0


    def pose_cb(self, msg):
        self.pose = msg.pose
        pass


    def waypoints_cb(self, waypoints):
        rospy.loginfo('Received waypoints - number of waypoints {}'.format(len(waypoints.waypoints)))
        self.base_waypoints = waypoints.waypoints
        pass

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        if msg.data == -1:
            self.reset_waypoints_velocity()
            self.traffic_waypoint = -1
        else:
            self.traffic_waypoint = msg.data
            self.slow_down_car()

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
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
