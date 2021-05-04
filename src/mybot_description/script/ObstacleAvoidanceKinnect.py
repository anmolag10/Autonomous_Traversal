#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan, Imu, NavSatFix
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
import math
import time
from pyproj import Geod


class Avoidance():
    def __init__(self):
        self.yaw = 0
        self.gps_angle = 0
        self.lat1 = 0
        self.lon1 = 0
        self.dist = 0
        self.lat2 = 49.9000015169
        self.state_description = ''
        self. start = True
        self.lon2 = 8.89987335398
        self.state = 0
        self.regions = {}
        self.pub = None

    def imu_callback(self, pose):
        quaternion = (pose.orientation.x, pose.orientation.y,
                      pose.orientation.z, pose.orientation.z)
        euler = euler_from_quaternion(quaternion)
        self.yaw = math.degrees(euler[2])+180
        self.yaw = abs(yaw-360)
        self.yaw = yaw % 360

    def gps_callback(self, data):
        self.lat1 = data.latitiude
        self.lon2 = data.longitude

    def align(self):
        speed_msg = Twist()
        self.dist
        geodesic = Geod(ellps='WGS84')
        bearing, reverse_bearing, dist = geodesic.inv(
            self.lon1, self.lat1, self.lon2, self.lat2)
        bearing = bearing + 180
        angle_diff = self.yaw - bearing
        while((angle_diff > 1 or angle_diff < -1)):
            if angle_diff > 1:
                if angle_diff < 180:
                    speed_msg.angular.z = 0.7
                elif angle_diff > 180:
                    speed_msg.angular.z = -0.7
                pub.publish(speed_msg)

            elif angle_diff < -1:
                angle_diff = abs(angle_diff)
                if angle_diff < 180:
                    speed_msg.angular.z = -0.7
                elif angle_diff > 180:
                    speed_msg.angular.z = 0.7
                pub.publish(speed_msg)

    def take_actions(self):
        speed_msg = Twist()
        linear_x = 0
        angular_z = 0

        if self.regions['front'] > 1 and self.regions['fleft'] > 1 and self.regions['fright'] > 1:
            self.state_description = '  CASE 1 - CLEAR'
            linear_x = 0
            angular_z = 0
            self.state = 0

        elif self.regions['front'] < 1 and self.regions['fleft'] > 1 and self.regions['fright'] > 1:
            self.state_description = 'CASE 2 - FRONT OBSTRUCTED'
            self.state = 1
            linear_x = 0
            angular_z = -0.3

        elif self.regions['front'] > 1 and self.regions['fleft'] > 1 and self.regions['fright'] < 1:
            self.state_description = 'CASE 3 - FRIGHT OBSTRUCTED'
            self.state = 1
            linear_x = 0.0
            angular_z = -0.3

        elif self.regions['front'] > 1 and self.regions['fleft'] < 1 and self.regions['fright'] > 1:
            self.state_description = 'CASE 4 -FLEFT OBSTRUCTED'
            self.state = 1
            linear_x = 0.0
            angular_z = 0.3

        elif self.regions['front'] < 1 and self.regions['fleft'] > 1 and self.regions['fright'] < 1:
            self.state_description = 'CASE 5 - FRONT AND FRIGHT OBSTRUCTED'
            self.state = 1
            linear_x = 0.0
            angular_z = -0.3

        elif self.regions['front'] < 1 and self.regions['fleft'] < 1 and self.regions['fright'] > 1:
            self.state_description = 'CASE 6 - FRONT AND FLEFT OBSTRUCTED'
            self.state = 1
            linear_x = 0.0
            angular_z = 0.3

        else:
            self.state_description = 'LOCAL MINIMA HAS OCCURED'
            rospy.loginfo(self.regions)

        rospy.loginfo(self.state_description)
        speed_msg.linear.x = linear_x
        speed_msg.angular.z = angular_z
        pub.publish(speed_msg)

    def kinect_callback(self, msg):
        newranges = []
        for i in range(0, 720):
            newranges.append(msg.ranges[i])
            if math.isnan(msg.ranges[i]) is True:
                newranges[i] = 10

        regions = {'right': min(min(newranges[576:713], 10)),
                   'fright': min(min(newranges[4376:575], 10)),
                   'front':  min(min(newranges[288:431]), 10),
                   'fleft':  min(min(newranges[144:287]), 10),
                   'left':   min(min(newranges[0:143]), 10)}

        if self.start is True:
            self.align()
            self.start = False

        self.take_actions(self.regions)

    def speed_cmd(self, l_x, l_y, l_z, a_x, a_y, a_z):
        speed_msg = Twist()

        speed_msg.linear.x = l_x
        speed_msg.linear.y = l_y
        speed_msg.linear.z = l_x
        speed_msg.angular.x = a_x
        speed_msg.angular.y = a_y
        speed_msg.angular.z = a_z
        self.pub.publish(speed_msg)

    def main_function(self):
        rospy.init_node('Obstacle Avoidance')
        rospy.Subscriber('imu', Imu, imu_callback)
        rospy.Subscriber('fix', NavSatFix, gps_callback)
        rospy.Subscriber('/scan', LaserScan, kinect_callback)

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        while self.state == 0:
            self.align
            while True:
                _, _, self.dist = geodesic.inv(
                    self.lon1, self.lat1, self.lon2, self.lat2)
                if self.dist > 1.5:
                    self.speed_cmd(-0.8, 0, 0, 0, 0, 0)
                else:
                    self.speed_cmd(0, 0, 0, 0, 0, 0)
        while self.state == 1:
            self.take_actions(self.regions)
        rospy.spin()


if __name__ == '__main__':
    Traversal = Avoidance()
    Traversal.main_function()
