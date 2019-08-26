#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg     import Lane, Waypoint
from scipy.spatial     import KDTree

import math


'''
Goal
1) Take (Subscribe to) the pose-of-the-car & the base_waypoints
   Intermediate steps.
   i)  In pose_cb read the pose
   ii) In waypoints_cb set up the base_waypoints as query-able KDTree
2) Publish a subset of the base_waypoints i.e from ahead of the car to the #LOOKAHEAD_WPS as a 'lane' object
   Intermediate steps.
   iii)  closest_waypoint_idx = query the base_waypoints-KDTree using car-pose
   iv)   Publish the closes_waaypoints using closest_waypoint_idx as a lane object

'''

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

        rospy.Subscriber('/current_pose',   PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane,        self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        #Declare Publishers
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        # Pose will be updated in pose_cb
        self.pose          = None
        # All the waypoints will be updated in waypoints_cb
        self.base_waypoints = None
        self.waypoints_2d   = None
        self.waypoint_tree  = None
        self.loop()

        #rospy.spin()


    # Pose-callback for the current_pose subscriber
    def pose_cb(self, msg):
        # TODO: Implement
        # Just subscribe to pose. i think this occurs every 50Hz
        self.pose = msg


    # Waypoints-callback for the base_waypoints subscriber
    def waypoints_cb(self, base_waypts):
        # TODO: Implement
        # Stores the waypoints in the object.
        # So will you store the waypoints everytime the waypoints come in.
        # Ans: This is a latched subscriber. So they dont get stored everytime . This is good because the base waypoint are not changing everytime
        self.base_waypoints = base_waypts

        # Take a chunk of base_waypoints take a chunk of them. Take the first 200 of them that are in front of the car as reference
        # Figure out which waypoint is closest to the car using the KDTree. KDTree will give you the closest waypoint 'n' efficiently(its efficient. Instead of O(n) is it O(log n)
        # Lets say you have a list of 1000 waypoints and the waypoint that you want is at the very end of the list then the a direct search would take 1000 cycles.
        # But the the KDTree is of the order(log n) = log 1000= 3 cycles (but aaron brown said 10 cycles ???, maybe he was wrong)

        #use if not self.waypoints_2d because we want to initialise self.waypoints_2d much before the subsrciber is initialised (to prevent race condition)
        if not self.waypoints_2d:
            #Convert the waypoints to 2d waypoints to make it KDTree friendly
            self.waypoints_2d  = [[way_pt.pose.pose.position.x, way_pt.pose.pose.position.y] for way_pt in base_waypts.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)



    def loop(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.pose and self.base_waypoints:
                # Get closest waypoint
                closest_waypoint_idx = self.get_closest_waypoint_idx()
                self.publish_waypoints(closest_waypoint_idx)
            rate.sleep()

    def get_closest_waypoint_idx(self):
        # Courtesy pose_cb
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y

        # Courtesy waypoints_cb
        # The waypoints_cb has already established the base_waypoints as a KDTree data structure in the self.waypoint_tree variable.
        # All we are doing is querying the waypoint_tree for the closest-base-waypoint-index based on the pose of the car (x,y)
        closest_waypt_idx = self.waypoint_tree.query([x,y] ,1)[1]

        # Check if closest is ahead or behind the vehicle
        closest_coord = self.waypoints_2d[closest_waypt_idx]
        prev_coord    = self.waypoints_2d[closest_waypt_idx -1]

        # Equation for hyperplane through closest_coords
        cl_vect   = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect  = np.array([x,y])
        val       = np.dot(cl_vect - prev_vect, pos_vect-cl_vect)
        if val > 0: # if waypoint is behind us
        	closest_waypt_idx = (closest_waypt_idx +1) % len(self.waypoints_2d)

        return closest_waypt_idx



    def publish_waypoints(self, closest_idx):
        lane           = Lane()
        # The lane headaer is the same as the base_waypoints.header
        lane.header    = self.base_waypoints.header

        # So all the waypoints will not be published: Publish only a slice of the waypoints (i.e. starting from the closest waypoint to the number of lookahead-waypoints ahead)
        # Publish this as a lane object
        lane.waypoints = self.base_waypoints.waypoints[closest_idx:closest_idx + LOOKAHEAD_WPS]

        # The actual pubisher
        self.final_waypoints_pub.publish(lane)






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
