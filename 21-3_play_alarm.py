#!/usr/bin/env python
#-*- coding: utf-8 -*-

#play_alarm.py는 시작 지점의 위도와 경도를 파라미터에 저장하고, 화재가 발생했을 경우 특정 사용자(들)에게 화재 발생 여부 및 그 지역을 알리는 프로그램이다.

from twilio.rest import Client
import rospy
from playsound import playsound
from bebop_msgs.msg import Ardrone3PilotingStatePositionChanged

#cb_get_gps()는 드론의 토픽으로부터 위도와 경도의 정보을 구독할 때, 실행할 콜백 함수이다.
def cb_get_gps(msg):
    global lati_now
    global long_now
    lati_now = msg.latitude
    long_now = msg.longitude

if __name__ == '__main__':
    rospy.init_node('play_alarm')
    #드론으로부터 드론의 gps값을 구독하고 그 값을 콜백 함수로 처리한다.
    rospy.Subscriber('/bebop/states/ardrone3/PilotingState/PositionChanged', Ardrone3PilotingStatePositionChanged, cb_get_gps)
    #화재 발생 여부 및 그 지역 정보를 문자로 받기 위해 필요한 정보를 저장하고 처리한다.
    account_sid = 'AC3a674bf50d4d0511d8600550e6e50739'
    auth_token = '715e9cbd3d7bbd699606e9915362381c'
    client = Client(account_sid, auth_token)

    #일단, 구동하자마자 시작점의 위도, 경도를 파라미터에 저장한다.
    while not rospy.is_shutdown():
        try:
            #콜백 함수로부터 받은 위도와 경도의 정보가 유효하다면 이를 파라미터에 저장한다. 유효하다는 의미는 위도와 경도의 범위가 대한민국의 지역 안에 해당될 수 있음을 나타낸다.
            if ( lati_now >=34.0 and lati_now <=38.0 and long_now >= 126.0 and long_now <=130.0 ):
                start_lati = lati_now 
                start_long = long_now 
                rospy.set_param("/ori_lati", start_lati)
                rospy.set_param("/ori_long", start_long)
                break
        except:
            continue

    try:
        while not rospy.is_shutdown():
            #파라미터로부터 화재 경보를 울려야 하는지 확인하고 만일, 화재를 경보를 울려야 한다면 조건문 아래의 코드를 실행한다.
            if rospy.get_param("/play_alarml/fire_detection_state") is True:
                #화재 경보음을 울린다.
                playsound('/home/iseonhwan/Documnet/ROS project/store/alarm.mp3')
                #화재 경보 메시지를 보낸다.
                alarm_s1 = str(lati_now) + ', '
                alarm_s2 = str(long_now)
                alarm_s3 = "에서 화재가 발생하였습니다"
                alarm_message = alarm_s1+alarm_s2+alarm_s1+alarm_s3
                print(alarm_message)
                #사용자의 번호로 화재 정보의 메시지를 보낸다.
                message = client.api.account.messages.create(to="+821077572419", from_="+17378885431", body= alarm_message )
                rospy.sleep(3)
    except rospy.ROSInterruptException:
        exit()