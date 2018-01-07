#!/usr/bin/env python

import os
import csv
import math

from geometry_msgs.msg import Quaternion

from styx_msgs.msg import Lane, Waypoint

import tf
import rospy

CSV_HEADER = ['x', 'y', 'z', 'yaw']
MAX_DECEL = 1.0


class WaypointLoader(object):

    def __init__(self):
        rospy.init_node('waypoint_loader', log_level=rospy.DEBUG)

        self.pub = rospy.Publisher('/base_waypoints', Lane, queue_size=1, latch=True)

        self.velocity = self.kmph2mps(rospy.get_param('~velocity'))
        self.new_waypoint_loader(rospy.get_param('~path'))
        rospy.spin()

    def new_waypoint_loader(self, path):
        if os.path.isfile(path):
            waypoints = self.load_waypoints(path)
            self.publish(waypoints)
            rospy.loginfo('Waypoint Loded')
        else:
            rospy.logerr('%s is not a file', path)

    def quaternion_from_yaw(self, yaw):
        return tf.transformations.quaternion_from_euler(0., 0., yaw)

    def kmph2mps(self, velocity_kmph):
        return (velocity_kmph * 1000.) / (60. * 60.)

    def load_waypoints(self, fname):
        waypoints = []
        with open(fname) as wfile:
            reader = csv.DictReader(wfile, CSV_HEADER)
            for wp in reader:
                p = Waypoint()
                p.pose.pose.position.x = float(wp['x'])
                p.pose.pose.position.y = float(wp['y'])
                p.pose.pose.position.z = float(wp['z'])
                q = self.quaternion_from_yaw(float(wp['yaw']))
                p.pose.pose.orientation = Quaternion(*q)
                p.twist.twist.linear.x = float(self.velocity)

                waypoints.append(p)

            # Clear out redundant waypoints that wrap around beginning of waypoint list.
            assert(len(waypoints) >= 12)
            matchWp = None
            for wp in range(len(waypoints)-1, len(waypoints)-10, -1):
                # Get angle between previous wp and this wp.
                map_prev_x = waypoints[wp-1].pose.pose.position.x
                map_prev_y = waypoints[wp-1].pose.pose.position.y
                map_x = waypoints[wp].pose.pose.position.x
                map_y = waypoints[wp].pose.pose.position.y
                theta_toThisWp_deg = math.atan2( (map_y - map_prev_y), (map_x - map_prev_x) ) * 180/math.pi

                # Get angle between this wp and wp 0.
                map_prev_x = waypoints[wp].pose.pose.position.x
                map_prev_y = waypoints[wp].pose.pose.position.y
                map_x = waypoints[0].pose.pose.position.x
                map_y = waypoints[0].pose.pose.position.y
                theta_to0_deg = math.atan2( (map_y - map_prev_y), (map_x - map_prev_x) ) * 180/math.pi

                # If angle between previous wp and this wp is in same general direction
                # as angle between this wp and wp 0, stop.
                if abs(theta_to0_deg - theta_toThisWp_deg) < 90:
                    matchWp = wp
                    break
            # If found a wp towards the end of the list that was in the same heading
            # as towards wp 0, clear out later waypoints.
            if matchWp is None:
                return self.decelerate(waypoints)
            else:
                # No need to decelerate--waypoints loop around.
                return waypoints[0:matchWp]


    def distance(self, p1, p2):
        x, y, z = p1.x - p2.x, p1.y - p2.y, p1.z - p2.z
        return math.sqrt(x*x + y*y + z*z)

    def decelerate(self, waypoints):
        last = waypoints[-1]
        last.twist.twist.linear.x = 0.
        for wp in waypoints[:-1][::-1]:
            dist = self.distance(wp.pose.pose.position, last.pose.pose.position)
            vel = math.sqrt(2 * MAX_DECEL * dist)
            if vel < 1.:
                vel = 0.
            wp.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
        return waypoints

    def publish(self, waypoints):
        lane = Lane()
        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time(0)
        lane.waypoints = waypoints
        self.pub.publish(lane)


if __name__ == '__main__':
    try:
        WaypointLoader()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint node.')
