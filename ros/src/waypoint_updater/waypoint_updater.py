#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

import math
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

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        self.final_waypoints_pub = rospy.Publisher('final_waypoints',
                                                   Lane,
                                                   queue_size=1)

        # TODO: Add other member variables you need below
        self.waypoints_ = None
        self.current_pose_ = None
        self.recieve_basepoint_ = False
        self.traffic_indx = -1
        self.loop()

    def loop(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.waypoints_ and self.current_pose_:
                self.send_now()
            rate.sleep()

    def send_now(self):
        if self.recieve_basepoint_:
            min_dist = 999999
            min_index = 0
            for i in range(len(self.waypoints_.waypoints)):
                dist = self.calc_distance(
                    self.waypoints_.waypoints[i].pose.pose.position,
                    self.current_pose_.pose.position)
                if (dist < min_dist):
                    min_dist = dist
                    min_index = i
            if (self.waypoints_.waypoints[min_index].pose.pose.position.x >
                    self.current_pose_.pose.position.x):
                start = min_index
                end = min_index + LOOKAHEAD_WPS

            else:

                start = min_index + 1
                end = min_index + LOOKAHEAD_WPS + 1
            msg = self.waypoints_.waypoints[start:end]
            new_waypoints = Lane()
            new_waypoints.waypoints = msg
            rospy.loginfo("current pose= %s\n",
                          self.current_pose_.pose.position)
            rospy.loginfo("nearest waitpoint= %s", msg[0].pose.pose.position)
            rospy.loginfo(
                "distance= %s\n",
                self.calc_distance(msg[0].pose.pose.position,
                                   self.current_pose_.pose.position))
            rospy.loginfo("self.traffic_indx= %s\n", self.traffic_indx)
            rospy.loginfo("end= %s\n", end)
            if (self.traffic_indx != -1) and (self.traffic_indx < end):
                new_waypoints.waypoints = self.stop(new_waypoints.waypoints,
                                                    start)
            self.final_waypoints_pub.publish(new_waypoints)

    def stop(self, waypoints, cloest_indx):
        JERK = .5
        points = []
        for i, waypoint in enumerate(waypoints):
            temp = Waypoint()
            temp.pose = waypoint.pose

            stop_indx = max(self.traffic_indx - cloest_indx - 4, 0)
            dist = self.distance(waypoints, i, stop_indx)
            vel = math.sqrt(2 * JERK * dist)
            if vel < 1:
                vel = 0
            temp.twist.twist.linear.x = min(vel, waypoint.twist.twist.linear.x)
            points.append(temp)
        return points

    def calc_distance(self, a, b):
        return math.sqrt((a.x - b.x)**2 + (a.y - b.y)**2 + (a.z - b.z)**2)

    def pose_cb(self, msg):
        # TODO: Implement
        self.current_pose_ = msg

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.waypoints_ = waypoints
        self.recieve_basepoint_ = True

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.traffic_indx = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0

        def dl(a, b):
            return math.sqrt((a.x - b.x)**2 + (a.y - b.y)**2 + (a.z - b.z)**2)

        for i in range(wp1, wp2 + 1):
            dist += dl(waypoints[wp1].pose.pose.position,
                       waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')