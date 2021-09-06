#!/usr/bin/env python
#-*- coding: utf-8 -*-

#detect_fire.py는 불을 발견하고 이에 따라 비행을 제어하는 코드이다.

import rospy, cv2, datetime, time, subprocess, rosnode
import serial
from std_msgs.msg import String
from bb2_pkg.MoveBB2_3 import MoveBB2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist


class DetectFire:

  def __init__(self):
    #드론으로부터 이미지를 구독한다. 구독한 이미지를 콜백 함수로 보낸다.
    self.sub = rospy.Subscriber("/bebop/image_raw", Image, self.callback)
    self.bridge = CvBridge()
    self.cv_msg = cv2.imread("cimg.png")

  def callback(self,data):
    try:
      #드론으로부터 이미지 데이터를 opencv가 처리할 수 있도록 변환한다.
      self.cv_msg = self.bridge.imgmsg_to_cv2(data, "bgr8")

    except CvBridgeError as e:
      print(e)

  def save_picture(self, picture):
    try:
      img = picture
      #저장한 이미지를 시간을 접미사로 하여 명명하고 저장한다.
      now = datetime.datetime.now() #datetime.strftime(format)은 명시적인 포맷 문자열에 의해 제어되는 날짜와 시간을 나타내는 문자열을 반환한다.
      date = now.strftime('%Y%m%d')
      hour = now.utcnow().strftime('%H%M%S%f') #화재/불의 인식은 아주 빠르게 이어지기 때문에 초 이하 단위까지도 접사로 하여서 동일한 이름이 겹치지 않도록 한다.
      filename = '/home/iseonhwan/Documnet/ROS project/picture/fire_{}_{}.png'.format(date, hour)
      cv2.imwrite(filename, img)
    except CvBridgeError as e:
      print(e)

if __name__ == '__main__':

    rospy.init_node('fire_detector', anonymous=False)
    fly = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size = 1)
    sp = serial.Serial('/dev/ttyUSB0', 9600)
    df = DetectFire()
    tw = Twist()
    rospy.sleep(1.0)

    mb = MoveBB2()
    #드론 비행의 스피드를 저장하는 변수
    dspeed = 0.1

    #rosnode kill 코드 블록이 조건에 따라 실행되도록 하는 인덱스 변수를 i로 설정한다. 0이면 해당 코드 블록을 실행하고, 1이면 실행하지 않는다.
    i = 0

    #이미지를 저장하는 코드의 실행 횟수를 저장하는 인덱스 변수를 j로 설정한다. 이미지를 저장하는 함수가 실행될 때마다 1씩 증가한다.
    j = 0

    #과거의 일정 시점에 불을 인지했는지 판단하는 내부 변수를 per_fire로 설정한다. 0이면, 과거의 일정 시점에 불/화재가 없었음을 의미하고 1이면, 과거의 일정 시점에 불/화재가 있었음을 의미한다.
    per_fire = 0

    #불을 인지하지 못한 시간을 저장하는 변수를 ztime으로 설정한다.
    ztime = 0.0

    #불을 인지했을 때의 시간을 저장하는 변수를 ftime으로 설정한다.
    ftime = 0.0

    try:
        #훈련 데이터로 불을 구별해낸다. CascadeClassifier()는 훈련 데이터(*.xml)로 특정 이미지의 특징을 추출해내는 함수이다. 'fire_detectionction.xml'에는 저장된 불의 특징이 저장되어 있다.
        fire_cascade = cv2.CascadeClassifier('/home/iseonhwan/Documnet/ROS project/store/fire_detection.xml') 

        while not rospy.is_shutdown():
          #파라미터로부터 화재를 인식할지의 여부를 확인한다. 파라미터 '/fire_detectorl/param_of_detector'가 'True'일 경우에 조건문 아래의 화재 인식 및 조정 비행을 실행한다.
           cod1 = rospy.get_param("/fire_detectorl/param_of_detector")
           if cod1 == True:
            
            #드론으로부터 이미지를 불러온다.
            frame = df.cv_msg
            #비밥에서 불러온 이미지에서 불/화재 객체를 찾는다.
            fire  = fire_cascade.detectMultiScale(frame, 1.2, 5) #detectMultiScale()은 입력된 이미지에서 사이즈가 다양한 객체를 찾는 함수이다. 사각형의 리스트로서 찾은 객체가 반환된다. 여기서 사각형의 리스트라 함은 왼쪽 꼭지점의 x좌표, y좌표, 폭(width), 넓이(height)에 관한 값을 저장한 리스트를 말한다. 여기서는 불의 이미지를 발견하고 이를 사각형의 리스트로서 반환된다.
            #detectMultiScale()의 매개변수는 필수적인 것 1개, 수의적인 것 5개를 취한다. 먼저 필수적으로 객체를 찾아야 하는 공간이라 할 수 있는 이미지가 있어야 한다. 두 번째부터는 수의적인 매개변수이다. 두 번째 매개변수는 각 이미지 범위(scale)를 얼마나 줄일 것인지 명시하는 값이고 세 번째 매개변수는 찾아낸 객체들이 얼마나 이웃하고 있어야 하는지를 나타내는 값이며, 네 번째 매개변수는 cvHaarDetectObjects()과 같은 의미의 매개변수이고, 다섯 번째 매개변수는 인식되는 개체가 최소 얼마나 되어야 하는지를 명시하는 값이며, 여섯 번째 매개변수는 인식되는 개체가 최대 얼마나 되어야 하는지를 명시하는 값이다.

            #fire가 0이라면, 즉, 불이 전혀 발견되지 않은 경우라면, 조건문의 코드를 실행한다.
            if len(fire) == 0:
              #불이 발견되지 않은 경우에 "/play_alarm/fire_detection_state"를 0으로 설정한다. /play_alarm/fire_detection_state는 화재가 발생했음을 소리나 SMS 등으로 알리는 코드를 실행하기 위한 파라미터(parameter)이다.
              rospy.set_param("/play_alarml/fire_detection_state", False)
              #불을 잠깐 인지했으나, 화면의 중앙에 맞추는 가운데 그 객체가 사라진 경우, 더 구체적으로 만일, 최초로 불을 발견한 시점과 불을 인지하지 못한 시점이 어느 한계를 넘어간다면 다시 순찰 비행(Flytarget)을 실행한다.
              #그 이전에 불을 발견했으나(즉, per_fire가 1이나) 현시점에서 불이 발견되지 않았다면 아래의 코드를 실행한다.
              if per_fire == 1:
                #불이 발견되지 않은 현재 시간을 저장한다.
                ztime = time.time()
                #과거에 불을 발견했던 시간과 현재 불을 발견하지 못한 시간의 차이를 구한다.
                gap_time = ztime - ftime
                #과거의 불을 발견했던 시간과 현재 불을 발견하지 못한 시간의 차이가 10초 이상이면(즉 그 10초 동안 불을 발견하지 못했다면) 아래의 코드를 실행한다.
                if gap_time >= 10.0:
                  print("화재로 의심되는 개체가 잠까 발견되었으나 확실하지 않습니다.")
                  #과거에 불을 감지했었음을 나타내는 인덱스 변수를 초기화한다.
                  per_fire = 0
                  #이후에 다시 실행하는 detect_fire.py에서 다시 FlyTarget.py를 시작할 수 있도록 i값을 초기화한다.
                  i = 0
                  #사진 촬영 횟수를 초기화한다.
                  j = 0
                  #다시 순찰 비행을 시작하는 파마미터를 설정한다.
                  rospy.set_param("/fly_to_targetl/param_of_flying", 2)
                  #비행 노드를 실행하기 위하여 Manager 노드의 파라미터 값을 변경시킨다.
                  rospy.set_param("/Fire_drone_managerl/order", 1)
            else:
              #불 또는 화재가 발견된 경우라면 아래의 코드를 실행한다.
              for (x,y,w,h) in fire:
                  #리스트 fire의 값만큼 아래의 코드를 실행한다.
                  #리스트 fire의 값을 활용하여 이미지에 사각형을 그린다.
                  cv2.rectangle(frame,(x-20,y-20),(x+w+20,y+h+20),(255,0,0),2)

                  print("화재가 의심되는 장면을 포착하였습니다.")
                  #일단 불을 발견했다면, 불을 인지했는지 판단하는 내부 변수(per_fire)를 변화시킨다.
                  per_fire = 1

                  #불을 발견했다면, 불을 인지한 시간을 저장한다.
                  ftime = time.time()

                  #불을 발견했다면 패트롤 모드(patrol mode)를 종료하고 adjust mode를 실행한다. 단, 불을 발견한 반복문 아래에서 한 번만 실행한다. 이를 위하여 인덱스 변수 i를 활용한다.
                  if i == 0:
                    rosnode.kill_nodes(['/fly_to_targetl'])
                    i = 1

                  #사각형의 가운데 좌표를 구한다.
                  fx = x + (w * 0.5)
                  fy = y + (h * 0.5)

                  #인식된 불이 화면의 중앙에 있을 경우에는 아래의 코드를 실행한다.
                  if ((fx >= 190)and(fx <= 450)) and ((fy >= 150)and(fy <= 330)):
                    #드론의 비행을 멈춘다.
                    tw.linear.x = 0
                    tw.linear.y = 0
                    fly.publish(tw)
                    print("화재 장면을 화면의 중앙에 맞췄습니다({}, {})".format(fx, fy))
                    #화재 경보를 알리는 코드를 활성화한다.
                    rospy.set_param("/play_alarml/fire_detection_state", True)
                    #동일한 화재에 대한 사진 촬영의 횟수가 5미만이면 사진을 찍는다.
                    if j < 5:
                      #인식한 화재/불의 이미지를 저장한다.
                      df.save_picture(frame)
                      print("화재 장면을 {}회 찍었습니다.".format(j+1))
                      #사진 촬영의 횟수를 j에 반영한다.
                      j = j + 1
                    else:
                      #사진을 일정 횟수까지 저장했다면, 드론이 일정 거리 앞으로 이동하여 소화탄을 투척한다. 여기서 일정 거리란, 드론이 화재를 인식한 각도, 그리고 드론의 높이를 고려한 거리이다. 드론이 3m 높이에서, 높이를 기준으로 45도 꺾어서 화재를 탐지하는 것을 전제로 함으로 앞으로 3m 이동하여 소화탄을 투척한다.
                      mb.move_x(3, 0.01)
                      sp.write('1')
                      print("소화탄을 투척합니다.")
                      #소화탄을 투척했다면, 화재 경보를 알리는 코드를 비활성화한다.
                      rospy.set_param("/play_alarml/fire_detection_state", False)
                      #처음 지점으로 되돌아 오는 비행 모드의 파라미터를 설정한다.
                      rospy.set_param("/fly_to_targetl/param_of_flying", 3)
                      #매니저 노드가 비행 코드를 실행하도록 파라미터를 설정한다
                      rospy.set_param("/Fire_drone_managerl/order", 1)
                      rospy.sleep(2)
                      #열려진 창을 닫는다.
                      cv2.destroyWindow('frame')
                      exit()

                  #화재가 화면의 아래에서 발견된 경우 뒤로 이동시킨다.
                  elif ((fx >= 190) and (fx <= 450)) and ((fy >= 331) and(fy <= 480)):
                      tw.linear.x = -dspeed
                      tw.linear.y = 0
                      fly.publish(tw)

                  #화재가 화면의 위에서 발견된 경우 드론을 앞으로 이동시킨다.  
                  elif ((fx >= 190) and (fx <= 450)) and ((fy >= 0 ) and(fy <= 149)):
                      tw.linear.x = dspeed
                      tw.linear.y = 0
                      fly.publish(tw)

                  #화재가 화면의 왼쪽에서 발견된 경우 드론을 왼쪽으로 이동시킨다.
                  elif ((fx >= 0) and (fx <= 189)) and ((fy >= 151 ) and(fy <= 329)):
                      tw.linear.x = 0
                      tw.linear.y = dspeed
                      fly.publish(tw)

                  #화재가 화면의 왼쪽 아래에서 발견된 경우 드론을 왼쪽 아래로 이동시킨다.
                  elif ((fx >= 0) and (fx <=189)) and ((fy >= 331) and (fy <= 480)):
                      tw.linear.x = -dspeed
                      tw.linear.y = dspeed
                      fly.publish(tw)

                  #화재가 화면의 왼쪽 위에서 발견된 경우 드론을 왼쪽 위로 이동시킨다.
                  elif ((fx >= 0) and (fx <=189)) and ((fy >= 0) and (fy <= 149)):
                      tw.linear.x = dspeed
                      tw.linear.y = dspeed
                      fly.publish(tw)

                  #화재가 오른쪽 아래에서 발견된 경우 드론을 오른쪽 아래로 이동시킨다.
                  elif ((fx >= 450) and (fx <=640)) and ((fy >= 331) and (fy <= 640)):
                      tw.linear.x = -dspeed
                      tw.linear.y = -dspeed
                      fly.publish(tw)

                  #화재가 오른쪽 위에서 발견된 경우, 드론을 오른쪽 위로 이동시킨다.
                  elif ((fx >= 450) and (fx <=640)) and ((fy >= 0) and (fy <= 149)):
                      tw.linear.x = dspeed
                      tw.linear.y = -dspeed
                      fly.publish(tw)

                  #화재가 오른쪽에 있을 경우, 드론을 오른쪽으로 이동시킨다.
                  elif ((fx >= 451) and (fx <= 640)) and ((fy >= 150 ) and(fy <= 330)):
                      tw.linear.x = 0
                      tw.linear.y = -dspeed
                      fly.publish(tw)                    
                  else:
                    pass

            cv2.imshow('frame', frame)
            #waitKey()는 키 입력을 기다리는 대기 함수이다. 매개변수는 대기 시간인데, 단위는 ms(millisecond, 1ms = 0.001)이다. 0xFF는 255를 의미한다. cv2.waitKey(1)의 반환값은 -1이고, -1과 255를 &연산하면 255가 된다. 그리고 ord()는 문자의 유니코드 값을 돌려주는 함수이다. ord('q')의 반환값은 113이다. 즉, 이 조건문은 결코 참이 될 수 없으므로 어떤 키 입력을 받더라도 위의 조건문은 멈추지 않는다.
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        rospy.spin()

    except KeyboardInterrupt:
        print("Shutting down")
