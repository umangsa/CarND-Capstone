#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

import math
import tf

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

        rospy.spin()

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

    def pose_cb(self, msg):
        # find the nearest waypoint ahead
        if self.base_waypoints:
            closest_wp = self.get_closest_waypoint(msg.pose, self.base_waypoints)

            # find the next waypoint
            map_x = self.base_waypoints[closest_wp].pose.pose.position.x
            map_y = self.base_waypoints[closest_wp].pose.pose.position.y
            heading = math.atan2( (map_y - msg.pose.position.y), (map_x - msg.pose.position.x) )
            quaternion = (
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            theta = euler[2]
            angle = abs(theta - heading)

            if(angle > math.pi / 4.0):
                closest_wp += 1

            # now get the list of waypoints that we want to publish
            final_waypoints = []
            index = closest_wp
            for i in range(closest_wp, closest_wp + LOOKAHEAD_WPS):
                wp = Waypoint()
                wp.pose.pose.position = self.base_waypoints[i].pose.pose.position
                wp.pose.pose.orientation = self.base_waypoints[i].pose.pose.orientation
                wp.twist.twist = self.base_waypoints[i].twist.twist
                final_waypoints.append(wp)

            # publish final waypoints
            lane = Lane()
            lane.header.stamp = rospy.Time.now()
            lane.header.frame_id = msg.header.frame_id
            lane.header.seq = msg.header.seq
            lane.waypoints = final_waypoints
            self.final_waypoints_pub.publish(lane)

        pass

    def waypoints_cb(self, waypoints):
        rospy.loginfo('Received waypoints - number of waypoints {}'.format(len(waypoints.waypoints)))
        self.base_waypoints = waypoints.waypoints
        pass

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
