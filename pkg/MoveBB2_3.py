#! /usr/bin/env python
#-*- coding: utf-8 -*-

# MoveBB2_3.py는 드론의 기본적인 비행과 관련한 모듈이다.
# 이 코드는 이용진("Odometry 토픽을 참조한 이동", 2021)에서 가져온 것이다.
# 출처: https://github.com/greattoe/ros_tutorial_kr/blob/master/ros1_tutorial/rospy/bebop2/bb2_4_move_by_odom.md

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from math import radians, degrees, pi, sqrt
from bb2_pkg.msg import Pos_XYZ_th

LIN_SPD = 0.200
LIN_MINSPD = 0.100
ANG_SPD = 0.50

class MoveBB2:
    def __init__(self):
        rospy.Subscriber('/bb2_pose_odom', Pos_XYZ_th, self.get_pos_xyzth_cb)
        self.pub0 = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size = 1)
        self.pub1 = rospy.Publisher('/bebop/takeoff', Empty, queue_size = 1)
        self.pub2 = rospy.Publisher('/bebop/land',    Empty, queue_size = 1)
        self.pub3 = rospy.Publisher('/bebop/reset',   Empty, queue_size = 1)
        self.pub4 = rospy.Publisher('/bebop/camera_control', Twist, queue_size = 1)

        self.empty_msg = Empty()
        self.xyzth_now = self.xyzth_org = Pos_XYZ_th()
        
        rospy.sleep(3.0)

    def get_pos_xyzth_cb(self, msg):
        self.xyzth_now = msg

    def print_xyzth(self, msg):
        print("x = {}, y = {}, z = {}, th = {}".format(msg.x, msg.y, msg.z, degrees(msg.th)))

    def update_org(self):
        self.xyzth_org = self.xyzth_now

    def elapsed_dist(self):
        return sqrt(pow((self.xyzth_now.x - self.xyzth_org.x), 2) + pow((self.xyzth_now.y - self.xyzth_org.y), 2))

    def elapsed_angle(self):
        return abs(self.xyzth_now.th - self.xyzth_org.th)

    def elapsed_height(self):
        return abs(self.xyzth_now.z - self.xyzth_org.z)
    
    def cam_control(self, angle):
        tw = Twist()
        tw.angular.y = angle
        self.pub4.publish(tw)

    def move_x(self, distance, tolerance):
        tw = Twist()
        tw.linear.x = 0.0
        tw.linear.y = 0.0
        tw.linear.z = 0.0
        tw.angular.z = 0.0

        if distance >= 0:   # distance(+): forward
            tw.linear.x =  LIN_SPD
        else:               # distance(-): backward
            tw.linear.x = -LIN_SPD
            
        self.update_org()   # update starting point
        
        while self.elapsed_dist() < abs(distance) - abs(distance) * tolerance:
            self.pub0.publish(tw)
        
        tw.linear.x = 0;    self.pub0.publish(tw) # stop move
        rospy.sleep(2.0)

    def move_y(self, distance, tolerance):
        tw = Twist()
        tw.linear.x = 0.0
        tw.linear.y = 0.0
        tw.linear.z = 0.0
        tw.angular.z = 0.0

        if distance >= 0:   # distance(+): move left
            tw.linear.y =  LIN_SPD
        else:               # distance(-): move right
            tw.linear.y = -LIN_SPD            
        self.update_org()   # update starting point

        while self.elapsed_dist() < abs(distance) - abs(distance) * tolerance:
            self.pub0.publish(tw)
        
        tw.linear.y = 0;    self.pub0.publish(tw) # stop move
        rospy.sleep(2.0)

    def move_xy(self, distance, tolerance, xspeed, yspeed):
        tw = Twist()
        distance = float(distance)
        xspeed = float(xspeed)
        yspeed = float(yspeed)

        #print("이동합니다.")
        if xspeed > 0:
            xspeed = ((distance/(distance+1))*xspeed) + LIN_MINSPD
        elif xspeed < 0:
            xspeed = ((distance/(distance+1))*xspeed) - LIN_MINSPD
        else:
            xspeed = 0
        #print("x속도는 {}입니다.".format(xspeed))
        if yspeed > 0:
            yspeed = ((distance/(distance+1))*yspeed) + LIN_MINSPD
        elif yspeed < 0:
            yspeed = ((distance/(distance+1))*yspeed) - LIN_MINSPD
        else:
            yspeed = 0
        #print("y속도는 {}입니다.".format(yspeed))
        tw.linear.x = xspeed
        tw.linear.y = yspeed

        self.update_org()
        while self.elapsed_dist() < abs(distance) - abs(distance)*tolerance:
          self.pub0.publish(tw)
        #print("이동을 완료했습니다.")

    def move_z(self, height, tolerance):
        tw = Twist()
        tw.linear.x = 0.0
        tw.linear.y = 0.0
        tw.linear.z = 0.0
        tw.angular.z = 0.0

        if height >= 0:	# height(+): ascend
            tw.linear.z =  LIN_SPD
        else:			# height(-): descend
            tw.linear.z = -LIN_SPD
        
        self.update_org()
        
        while self.elapsed_height() < abs(height) - abs(height) * tolerance:
            self.pub0.publish(tw)

        tw.linear.z =  0;  self.pub0.publish(tw) # stop move
        rospy.sleep(2.0)
        
    def rotate(self, deg, tolerance):
        tw = Twist()
        angle = radians(deg)
        tw.linear.x = 0.0
        tw.linear.y = 0.0
        tw.linear.z = 0.0
        tw.angular.z = 0.0

        if angle >= 0:	# angle(+): rotate left(ccw)
            tw.angular.z =  ANG_SPD
        else:			# angle(-): rotate right(cw)
            tw.angular.z = -ANG_SPD
        
        self.update_org()
        
        while self.elapsed_angle() < abs(angle) - abs(angle) * tolerance:
            self.pub0.publish(tw)
            
        tw.angular.z =  0;  self.pub0.publish(tw) # stop move
        rospy.sleep(2.5)
    
    def takeoff(self):
        self.pub1.publish(self.empty_msg)
        #print("이륙합니다.")
   
    def landing(self):
        self.pub2.publish(self.empty_msg)
        #print("착륙합니다")

    def emergency(self):
        tw = Twist()
        self.pub3.publish(self.empty_msg)
        print("emergency")

