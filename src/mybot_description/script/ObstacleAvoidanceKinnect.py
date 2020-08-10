#! /usr/bin/env python

import rospy

from sensor_msgs.msg import LaserScan, Imu, NavSatFix
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
import math
import time
from pyproj import Geod
yaw=0
gps_angle=0
lat1=0
lon1=0
dist=0
lat2= 49.9000015169
state_description=''
start=True
lon2= 8.89987335398
state=0
regions={}



pub = None

def callback(pose):
    global yaw
    quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)

    euler = euler_from_quaternion(quaternion)
    yaw= math.degrees(euler[2])+180
    yaw = abs(yaw-360)
    yaw = yaw%360

def callback2(data):
    global lat1 
    lat1= data.latitude
    global lon1 
    lon1= data.longitude
def angle_dif():
   global dist
   geodesic =Geod(ellps='WGS84')
   bearing, reverse_bearing, dist = geodesic.inv(lon1,lat1,lon2,lat2)
   bearing = bearing +180
   angle_diff = yaw- bearing
   return angle_diff
   




def take_action(regions):
    msg = Twist()
    linear_x = 0
    angular_z = 0
    global state

    global state_description
    anglediff=angle_dif()
    if regions['front'] > 1 and regions['fleft'] > 1 and regions['fright'] > 1:
     state_description = 'case 1 - nothing'
     linear_x = 0.0
     angular_z = 0
     state=0
   
    elif regions['front'] < 1 and regions['fleft'] > 1 and regions['fright'] > 1:
        state_description = 'case 2 - front'
        linear_x = 0.0
        angular_z = -0.3
        
        

    elif regions['front'] > 1 and regions['fleft'] > 1 and regions['fright'] < 1:
        state_description = 'case 3 - fright'
        linear_x = 0.0
        angular_z = -0.3
        
    elif regions['front'] > 1 and regions['fleft'] < 1 and regions['fright'] > 1:
        state_description = 'case 4 - fleft'
        linear_x = 0.0
        angular_z = 0.3
        
    elif regions['front'] < 1 and regions['fleft'] > 1 and regions['fright'] < 1:
        state_description = 'case 5 - front and fright'
        linear_x = 0.0
        angular_z = -0.3
        
    elif regions['front'] < 1 and regions['fleft'] < 1 and regions['fright'] > 1:
        state_description = 'case 6 - front and fleft'
        linear_x = 0.0
        angular_z = 0.3
        
    elif regions['front'] < 1 and regions['fleft'] < 1 and regions['fright'] < 1:
        state_description = 'case 7 - front and fleft and fright'
        linear_x = 0.0
        angular_z = -0.3
      
    elif regions['front'] > 1 and regions['fleft'] < 1 and regions['fright'] < 1:
        state_description = 'case 8 - fleft and fright'
        linear_x = 0.0
        angular_z = -0.3
        
    else:
        state_description = 'unknown case'
        rospy.loginfo(regions)

    rospy.loginfo(state_description)
    msg.linear.x = linear_x
    msg.angular.z = angular_z
    pub.publish(msg)
def clbk_laser(msg):
      global start
      global state_description
      print(start)
      twist=Twist()
      
      if str(min(msg.ranges[0:144]))=='nan':
        r1=10
      else:
        r1=(min(msg.ranges[0:144]))
      if str(min(msg.ranges[144:287]))=='nan':
        r2=10
      else:
        r2=(min(msg.ranges[144:287]))
      if str(min(msg.ranges[288:431]))=='nan':
        r3=10
      else:
        r3=(min(msg.ranges[288:431]))
      if str(min(msg.ranges[432:575]))=='nan':
        r4=10
      else:
        r4=(min(msg.ranges[432:575]))
      if str(min(msg.ranges[576:713]))=='nan':
        r5=10
      else:
        r5=(min(msg.ranges[576:713]))   
      global regions     
      regions = {
        'right':  min(r5, 10),
        'fright': min(r4, 10),
        'front':  min(r3, 10),
        'fleft':  min(r2, 10),
        'left':   min(r1, 10), }
      
      angle_diff=angle_dif()
      print(angle_diff)
      print('start val',start)
      
      while (angle_diff>2 or angle_diff<-2) and  (start is True):
       
       print('entering the loop')
       angle_diff=angle_dif()
       if angle_diff>2:
            print(2)
            print(angle_diff)
            if angle_diff<180:
               twist.angular.z=0.7
            elif angle_diff>180:
                twist.angular.z=-0.7
            pub.publish(twist)

       elif angle_diff<-2:
            print(3)
            print(angle_diff)
            angle_diff = abs(angle_diff)
            if angle_diff<180:
                twist.angular.z = -0.7
            elif angle_diff>180:
                twist.angular.z = 0.7
            pub.publish(twist)
       if angle_diff<2 and angle_diff>-2:

           twist.angular.z=0
           print(angle_diff)
           pub.publish(twist)
           start=False
           break
      
def main():
    global pub
 
    rospy.init_node('reading_laser','bot_yaw','bot_gps')
    
    rospy.Subscriber("imu",Imu,callback)
    rospy.Subscriber("fix",NavSatFix,callback2)
    rate=rospy.Rate(10)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    twist=Twist()
    while state ==0:
     while 1:
	geodesic =Geod(ellps='WGS84')
	bearing, reverse_bearing, dist = geodesic.inv(lon1,lat1,lon2,lat2)
        bearing = bearing +180
	angle_diff = yaw- bearing
        print("Yaw: ", yaw, "Bearing: ", bearing)
        print("Angle: ", angle_diff)       
        
        if angle_diff>1:
            if angle_diff<180:
                twist.angular.z=0.7
            elif angle_diff>180:
                twist.angular.z=-0.7
            pub.publish(twist)

        elif angle_diff<-1:
            angle_diff = abs(angle_diff)
            if angle_diff<180:
                twist.angular.z = -0.7
            elif angle_diff>180:
                twist.angular.z = 0.7
            pub.publish(twist) 
        if angle_diff<1 and angle_diff>-1:
            break
     while 1:
        bearing, reverse_bearing, dist = geodesic.inv(lon1,lat1,lon2,lat2)
        if  dist>3:
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x=0
            twist.angular.y=0
            twist.angular.z=0
            twist.linear.x= -0.8
            print("Distance: ", dist)
            pub.publish(twist)
            flag=0
        
        elif dist<3:
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x=0
            twist.angular.y=0
            twist.angular.z=0
            twist.linear.x = 0
            pub.publish(twist)     
   
    while state==1:
       take_action(regions)



 
    rospy.spin()
 
if __name__ == '__main__':
    main()
