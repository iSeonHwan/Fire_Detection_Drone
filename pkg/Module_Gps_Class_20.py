#!/usr/bin/env python
#-*- coding: utf-8 -*-

#Module_Gps_Class_20.py는 GPS 정보를 바탕으로 비행하는 코드이다.
#이 코드는 이용진("8. GPS 좌표를 이용한 드론 이동", 2021)의 코드를 활용한 것이다.
#출처: https://github.com/greattoe/ros_tutorial_kr/blob/master/ros1_tutorial/rospy/bebop2/bb2_6_move_by_gps.md

#표준 라이브러리를 임포트한다.
import rospy, sys

from scipy import sqrt, cos, sin, arctan2, pi
from math import degrees, radians, sqrt
from haversine import haversine

#필요한 메시지를 임포트한다.
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from bebop_msgs.msg import Ardrone3PilotingStateAttitudeChanged
from bebop_msgs.msg import Ardrone3PilotingStatePositionChanged
from bebop_msgs.msg import Ardrone3PilotingStateAltitudeChanged
from bebop_msgs.msg import Ardrone3GPSStateNumberOfSatelliteChanged
from bebop_msgs.msg import CommonCommonStateBatteryStateChanged as Battery

#사용자가 만든 클래스를 임포트한다.
from bb2_pkg.MoveBB2_3 import MoveBB2
#from bb2_pkg.Module_Local_patrol_20 import Local_Patrol

'''
    GPS for center of map  ( 36.51994848698016, 127.17306581466163)
    Parot-Sphinx start GPS ( 48.878900,           2.367780        )
    diffrence              (-12.358951513,     +124.805285815     ) 
'''
#스핑크스의 좌표 특성에 따른 보정값을 아래와 같이 정의한다. 실제 드론을 날릴 때에는 0으로 해야 한다.
OFFSET_LAT = -12.358951513
OFFSET_LON = 124.805285815
#OFFSET_LAT = 0.0
#OFFSET_LON = 0.0
LIN_SPD    = 0.30
ANG_SPD    = 0.30 * pi
#실제 비행에서는 고도를 64로 할 것.
FLIGHT_ALT = 4
DEG_PER_M  = 0.00000899320363721
'''
                               p2 (lat2,lon2)
                       | | |   /         
                       | | |  / 
                       | | |0/  
                       | | |/              
                       | |0/    
                       | |/      
                       |0/<--- bearing         
                       |/________     
                       p1 (lat1,lon1)
                       
  when center is (a,b), equation of circle : pow((x-a),2) + pow((y-b),2) = pow(r,2)
'''
class MoveByGPS:
    
    def __init__(self):
        
        rospy.Subscriber('/bebop/states/ardrone3/PilotingState/AttitudeChanged',
                         Ardrone3PilotingStateAttitudeChanged,
                         self.cb_get_atti)
        rospy.Subscriber('/bebop/states/ardrone3/PilotingState/PositionChanged',
                         Ardrone3PilotingStatePositionChanged,
                         self.cb_get_gps)
        rospy.Subscriber('/bebop/states/ardrone3/PilotingState/AltitudeChanged',
                         Ardrone3PilotingStateAltitudeChanged,
                         self.cb_get_alti) #고도를 0부터 시작함.
        rospy.Subscriber('/bebop/states/ardrone3/GPSState/NumberOfSatelliteChanged',
                         Ardrone3GPSStateNumberOfSatelliteChanged,
                         self.cb_get_num_sat)
        rospy.Subscriber("bebop/states/common/CommonState/BatteryStateChanged", Battery, self.cb_bt_check)

        self.atti_now = 0.0
        self.atti_tmp = 0.0        
        self.use_tmp = False        
        self.lati_now = 1000.0
        self.long_now = 1000.0
        self.alti_gps = 0.0
        self.alti_bar = 0.0
        self.bearing_now = 0.0
        self.bearing_ref = 0.0
        #배터리를 상태의 값을 담는 변수
        self.battery_percent = 100.0

        self.margin_angle  = radians(5.0)
        self.margin_radius = DEG_PER_M * 1.5
        self.margin_alt    = 0.25
        
        self.tw = Twist()
        #비밥을 움직이는 코드의 객체를 만든다.
        self.mb = MoveBB2()
        
        #지역 순찰의 코드의 객체를 만든다.
        #self.lf = Local_Patrol()

        rospy.sleep(3.0)

    def cb_bt_check(self, data):
        self.battery_percent = int(data.percent)

    def cb_get_num_sat(self, msg):
        pass

    def cb_get_alti(self, msg):
        self.alti_bar = msg.altitude


    def cb_get_gps(self, msg):
        self.lati_now = msg.latitude  + OFFSET_LAT
        self.long_now = msg.longitude + OFFSET_LON
        self.alti_gps = msg.altitude


    def cb_get_atti(self, msg):
    
        self.atti_now = msg.yaw
        
        if   msg.yaw < 0:
            self.atti_tmp = msg.yaw + pi 
        elif msg.yaw > 0:
            self.atti_tmp = msg.yaw - pi
        else:
            self.atti_tmp = 0.0
            
    
    def get_atti(self):
        if self.use_tmp == True:
            return self.atti_tmp
        else:
            return self.atti_now
            
    
    def get_bearing(self, lat1, lon1, lat2, lon2):

        Lat1,  Lon1 = radians(lat1), radians(lon1) 
        Lat2,  Lon2 = radians(lat2), radians(lon2) 

        y = sin(Lon2-Lon1) * cos(Lat2) 
        x = cos(Lat1) * sin(Lat2) - sin(Lat1) * cos(Lat2) * cos(Lon2-Lon1) 

        return arctan2(y, x)
    
        
    def get_gps_now(self):
        return self.lati_now, self.long_now
            
    
    def rotate(self, lat2, lon2, speed):

        pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size = 1)
        tw  = Twist()
        
        lat1, lon1 = self.get_gps_now()
        
        target  = self.get_bearing(lat1, lon1, lat2, lon2)
        
        current = self.atti_now;        angle = abs(target-current)
        
        if angle > pi:  #   if angle > radians(180):
            self.use_tmp = True            
            if   target > 0.0:
                target = target - pi
            elif target < 0.0:
                target = target + pi
            else:   pass
            current = self.get_atti();  angle = abs(target - current)
        else:           #   if angle > radians(180):
            self.use_tmp = False        
        
            
        if   target > current:    # cw, -angular.z
            
            tw.angular.z = -speed
            
            if   angle > radians(50):
                target = target - radians(5)
            elif angle > radians(20): 
                target = target - radians(10)
            else:
                tw.angular.z = -0.1125
                
            while target > current:
                if abs(tw.angular.z) > 0.125:
                    tw.angular.z = -speed * abs(target - current) / angle
                else:
                    tw.angular.z = -0.125
                current = self.get_atti();  pub.publish(tw)
                
        elif target < current:    # ccw,  angular.z            
            
            tw.angular.z =  speed
            
            if   angle > radians(50):
                target = target + radians(5)
            elif angle > radians(20): 
                target = target + radians(10)
            else:
                tw.angular.z =  0.1125
                
            while target < current:
                if abs(tw.angular.z) > 0.125:
                    tw.angular.z =  speed * abs(target - current) / angle
                else:
                    tw.angular.z =  0.125
                current = self.get_atti();  pub.publish(tw)
                
        else:   pass

    def check_route(self, lat2, lon2):
        
        lat_now, lon_now = self.get_gps_now()
        
        bearing = self.get_bearing(lat_now, lon_now, lat2, lon2)
        
        if bearing > self.bearing_ref - self.margin_angle and \
           bearing < self.bearing_ref + self.margin_angle:
            return True
        else:
            return False
        

    def check_alt(self):
        if self.alti_bar > FLIGHT_ALT - self.margin_alt and \
           self.alti_bar < FLIGHT_ALT + self.margin_alt:
            return True
        else:
            return False


    def check_arrived(self, lat2, lon2):
        lat_now, lon_now = self.get_gps_now()
        radius = sqrt(pow((lat_now-lat2), 2) + pow((lon_now - lon2), 2))
        
        if radius < self.margin_radius:
            return True
        else:
            return False

    #드론의 배터리를 확인하는 함수
    def check_battery(self):
        rospy.loginfo("배터리 잔량: %d%%", self.battery_percent)

    def alti_up(self):
        while self.alti_bar < FLIGHT_ALT:
            self.tw.linear.z = LIN_SPD
            self.mb.pub0.publish(self.tw)

    def move_to_target(self, lat1, lon1, lat2, lon2):
        while self.check_arrived(lat2, lon2) is False:
            
            if self.check_alt() is False:
                if self.alti_bar > FLIGHT_ALT:
                    self.tw.linear.z = -0.1
                else:
                    self.tw.linear.z =  0.1
            else:
                self.tw.linear.z = 0.0
            
            if self.check_route(lat2, lon2) is True:
                self.tw.linear.x = LIN_SPD
                self.mb.pub0.publish(self.tw)

            else:
                lat1, lon1 = self.get_gps_now()
                self.bearing_ref = self.get_bearing(lat1, lon1, lat2, lon2)
                self.rotate(lat2, lon2, radians(45))

        self.tw.linear.x = 0
        self.mb.pub0.publish(self.tw)
        rospy.sleep(2.0)

        #비밥이 정북을 바라보도록 roatate의 인자값을 조정한다.
        rot_lati = self.lati_now + 10
        rot_long = self.long_now
        self.rotate(rot_lati, rot_long, radians(0))
        print("목표의 gps 위치({}, {}) {}에 도착했습니다." .format(self.lati_now, self.long_now, self.alti_gps))

    def fly_to_target(self, p2_lati_deg, p2_long_deg):
        if not rospy.is_shutdown(): 

            #이륙한다.
            self.mb.takeoff()
            #정면을 바라본다.NameError()
            self.mb.cam_control(0)

            #고도를 높인다.
            self.alti_up()

            p1_lati_deg = self.lati_now
            p1_long_deg = self.long_now

            print("{}, {}에서 {}, {}으로 이동합니다".format(p1_lati_deg, p1_long_deg, p2_lati_deg, p2_long_deg))

            self.bearing_ref = self.get_bearing(p1_lati_deg, p1_long_deg, p2_lati_deg, p2_long_deg)
            self.rotate(p2_lati_deg, p2_long_deg, radians(45))        
            self.move_to_target(p1_lati_deg, p1_long_deg, p2_lati_deg, p2_long_deg)

            #도착한 후 카메라가 45도를 바라보도록 한다.
            self.mb.cam_control(-45)

        else:
            exit()

    #회전하지 않고 일정한 방향을 바라본 채 목표 지점으로 이동하는 코드를 아래와 같이 구현한다.
    def fly_without_rotate(self, tar_lati, tar_long):
        self.mb.takeoff()
        self.alti_up()
        #현재의 좌표, 목표 좌표의 정보를 이용하여 사선으로 이동한다.
        #현재의 좌표를 구한다.
        #ori_lati = self.lati_now
        #ori_long = self.long_now
        #print("현재의 좌표는 {}, {}입니다.".format(ori_lati, ori_long))

        #지구 위에서의 gps 이동은 완전히 직선으로 이루어질 수 없으므로 10번 반복한다.
        for i in range(5):
            #현재의 좌표를 구한다.
            ori_lati = self.lati_now
            ori_long = self.long_now
            #print("현재의 좌표는 {}, {}입니다.".format(ori_lati, ori_long))

            #비밥이 정북을 바라보도록 roatate의 인자값을 조정한다.
            self.rotate(ori_lati, ori_long, radians(0))
            print("{}, {}에서 {}, {}으로 순찰합니다.".format(ori_lati, ori_long, tar_lati, tar_long))

            #목표 좌표와 현재 좌표의 위도 거리, 경도 거리, 총 거리를 구한다.
            start_point = (ori_lati, ori_long)
            target_point = (tar_lati, tar_long)

            #위도 간의 차이는 la_gap에 저장한다. 경도는 시작점을 기준으로 한다.
            la_gap = haversine((ori_lati, ori_long), (tar_lati, ori_long), unit = 'm')

            #경도 간의 차이는 lo_gap에 저장한다. 위도는 시작점을 기준으로 한다.
            lo_gap = haversine((ori_lati, ori_long), (ori_lati, tar_long), unit = 'm')

            #총 거리는 dis에 저장한다.
            dis = haversine(start_point, target_point, unit = 'm')
            #위도의 거리(la_gap), 경도의 거리(lo_gap), 총 거리(dis)를 구한다.

            #print("위도 간 차이는 {}이고, 경도간 차이는 {}이며, 거리는 {}입니다.".format(la_gap, lo_gap, dis))
            #위도 거리, 경도거리, 총거리의 비율을 고려하여 x속도, y속도를 비례적으로 적용한다.
            xdeno = la_gap + lo_gap
            xdeno = abs(xdeno)

            #목표 지점의 위도가 더 높으면 음수를 주고, 아니면 양수를 준다.
            if tar_lati >= ori_lati:
                xnume = la_gap
            elif tar_lati < ori_lati:
                xnume = -la_gap
            else:
                pass
            xspeed = xnume/xdeno * LIN_SPD 

            ydeno = la_gap + lo_gap
            ydeno = abs(ydeno)

            #위도와 달리, 목표 지점의 경도가 더 높으면 양수를 주고, 아니면 음수를 준다. 왜냐하면, 비밥은 -y값이 오른쪽으로 향하기 때문이다.
            if tar_long >= ori_long:
                ynume = -lo_gap
            elif tar_long < ori_long:
                ynume = lo_gap
            else:
                pass

            yspeed = ynume/ydeno * LIN_SPD

            self.mb.move_xy(dis, 0.5, xspeed, yspeed)

        #print("이동을 완료했습니다.")
        #현재의 좌표를 다시 구한다.
        #ori_lati = self.lati_now
        #ori_long = self.long_now
        #print("현재의 좌표는 {}, {}입니다.".format(ori_lati, ori_long))
