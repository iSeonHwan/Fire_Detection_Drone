# Fire_Detection_Drone(화재 알림 소방 드론)

<div align = "right">

이선환, 이주현, 조민석, 한정훈

</div>
 
## 1. 배경

전세계적으로 화재의 피해가 급증하고 있다. 2021년 그리스, 터키, 이탈리아, 알제리 등에서는 산불이 발생하여 매우 큰 재산과 인명의 피해를 입혔고, 그리스 에비아 섬은 8월 3일날 발생한 화재로 서울시 면적의 80%에 달하는 면적이 불탔다. 터키 안탈리아주에서 시작된 불은 해당 지역에 평년보다 8배 넓은 지역을 불태우기도 했다(노예진, “’특파원보고 세계는지금’기후변화로 인한 섭씨 50도의 폭염...최악의 산불”; “화마가 덮친 남유럽, 통제불능 기후변화”). 전문가들은 그 주요한 원인 중 하나로 기후변화를 꼽고 있다. 즉, 일시적인 것이 아니라 지속적으로 우리가 마주하고 대처해야 할 재난 중 하나가 바로 화재라는 것이다.

e-나라지표(https://www.index.go.kr)에 따르면, 2011년부터 2020년까지 우리나라는 비록 최근에 연도별 화재 발생 건수가 줄어들고 있으나, 재산 피해의 규모는 급증하고 있다.

<div align = "center">
 
![화재 현황표_1](https://user-images.githubusercontent.com/84608929/131605822-d7d0a013-095d-4b76-bf28-15cb429672d9.png)

![화재_현황 그래프_1](https://user-images.githubusercontent.com/84608929/131606254-6f496f38-4664-4ccc-b1df-b8b12475dc0d.png)

</div>

반면, 우리나라에서는 최근 소방공무원의 고령화가 급격히 진행되고 있다. 조성완의 연구에 따르면, 2015년 전체 소방공무원의 94,617명 중 20대와 30대가 11,971명으로 전체 4%에 불과하고 50대 이상의 소방공무원이 40,609명으로 전체 43%에 달했다. 소방공무원은 사회 재난, 자연 재난의 현장에서 국민의 생명과 재산을 보호하는 고당도의 업무를 맡고 있기에(조성완, “소방공무원의 고령화에 따른 자기효능감, 팀워크가 현장대응역량에 미치는 영향과 제도 개선방안에 대한 연구”), 멀지 않은 미래에 그 규모와 빈도가 커질 화재 재난에 효과적으로 대응할 역량이 점점 낮아지고 있는 실정에 있다.

소방 분야에서 드론은 이러한 문제점을 해결할 하나의 주요한 대안이 될 수 있다. 화재 현장에서 현장 지휘관은 현장 대원이나 지휘 참모로부터 정보를 얻고 이로써 화재 진압을 위한 최선의 판단을 해야 한다. 그런데 현장 지휘관이 얻는 정보는 모두 음성 정보로 전달되기 때문에 지휘관 1명이 모든 정보를 통제하고 관리하기에는 어려운 한계가 있다(신열우&박진호, “경험적 접근법을 활용한 재난현장에서의 소방드론 임무수행 효율성 분석”.). 소방 드론은 화재의 범위, 연소 경로, 구대 대상자의 위치 확인 등 지상에서 놓칠 수 있는 입체적인 정보를 제공할 수 있기에(배용민, “[119기고] 소방 드론의 활용”), 드론의 투입은 화재 진압시 소방 인력을 효율적으로 활용하여 화재 진압 및 구조를 수행하도록 할 수 있다.

재난 현장에서 유용하게 사용될 수 있는데도 불구하고 현재 소방 드론의 활용 빈도는 높지 않다(배용민, “[119기고] 소방 드론의 활용”). 가장 큰 이유는 소방 드론을 운용하는 인원이 제한적이기 때문이다. 만일 사람의 전문적인 컨트롤이 필요없는 자동화 시스템을 드론이 갖춘다면, 소방대원들이 더 쉽게 드론을 화재 진압 현장에 활용할 수 있을 것이다.

전세계적으로 기후 이상 현상으로 화재 발생의 빈도 및 규모가 커지는 가운데, 소방 공무원의 노령화가 진행되는 상황 속에서 우리 프로젝트 드론팀은 소방 인력을 최소화하고 화재 진압이 효과적으로 이루어질 수 있도록 하는 화재 탐지 드론 시스템을 구축하고자 한다.

## 2. 개념


- 드론: 조종사가 탑승하지 아니한 상태로 항행할 수 있는 비행체(드론 활용의 촉진 및 기반조성에 관한 법률(약칭: 드론법)'(국토교통부(첨단항공과), 법률 제16420호, 2019. 4. 30, 제정)

- 드론시스템: 드론의 비행이 유기적·체계적으로 이루어지기 위한 드론, 통신체계, 지상통제국(이·착륙장 및 조종인력을 포함한다), 항행관리 및 지원체계가 결합된 것('드론 활용의 촉진 및 기반조성에 관한 법률(약칭: 드론법)'(국토교통부(첨단항공과), 법률 제16420호, 2019. 4. 30, 제정)

- 화재: 사람의 의도에 반하거나 고의에 의해 발생하는 연소 현상으로서 소화 설비 등을 사용하여 소화할 필요가 있거나 또는 사람의 의도에 반해 발생하거나 확대된 화학적인 폭발 현상(소방청,  2020년도 화재통계연감, 2021., 3쪽.)

- 조사: 화재 원인을 규명하고 화재로 인한 피해를 산정하기 위하여 자료의 수집, 관계자 등에 대한 질문, 현장 확인, 감식, 감정 및 실험 등을 하는 일련의 활동(소방청, 2020년도 화재통계연감, 2021., 3쪽.)

- 감식: 화재 원인의 판정을 위하여 전문적인 지식, 기술 및 경험을 활용하여 주로 시각에 의한 종합적인 판단으로 구체적인 사실 관계를 명확하게 규명하는 것(소방청, 2020년도 화재통계연감, 2021., 3쪽.)

- 감정: 화재와 관련되는 물건의 형상, 구조, 재질, 성분, 성질 등 이와 관련된 모든 현상에 대하여 과학적 방법에 의한 필요한 실험을 행하고 그 결과를 근거로 화재 원인을 밝히는 자료를 얻는 것(소방청, 2020년도 화재통계연감, 2021., 3쪽.)

- 발화 지점: 열원과 가연물이 상호작용하여 화재가 시작된 지점(소방청, 2020년도 화재통계연감, 2021., 3쪽.)

- 화재 현장: 화재가 발생하여 소방대 및 관계자 등에 의해 소화 활동이 행하여지고 있는 장소(소방청, 2020년도 화재통계연감, 2021., 4쪽.)

- 상황실: 소방관서 또는 소방기관에서 화재·구조·구급 등 각종 소방 상황을 접수·전파 처리 등의 업무를 행하는 곳(소방청, 2020년도 화재통계연감, 2021., 4쪽.)

- 접수: 119상황실에서 화재 등의 신고를 받은 최초의 시각(소방청, 2020년도 화재통계연감, 2021., 4쪽.)

- 출동: 화재를 접수하고 119상황실로부터 출동지령을 받아 소방대가 소방서 차고에서 출발하는 것(소방청, 2020년도 화재통계연감, 2021., 4쪽.)

- 도착: 출동지령을 받고 출동한 선착대가 현장에 도착하는 것(소방청, 2020년도 화재통계연감, 2021., 4쪽.)

- 잔불 감시: 화재를 진화한 화재가 재발하지 않도록 감시조를 편성하여 불씨가 완전히 소멸될 때까지 확인하는 것(소방청, 2020년도 화재통계연감, 2021., 4쪽.)

# 3.개념적 알고리즘

<div align = "center">

![21_알고리즘](https://user-images.githubusercontent.com/84608929/131607483-e052e912-9fb5-4cfe-98dd-0169c9bf885d.png)

</div> 

# 4. 프로그램 내부 구조도

<div align = "center">
 
 ![image](https://user-images.githubusercontent.com/84608929/131785487-136bcdf6-f899-40d2-9194-414646c12ce8.png)
 
 </div>


# 5.기능


1. 화재 감지 드론은 사용자가 정해준 일정한 영역을 체계적으로 비행한다.

2. 화재 감지 드론은 사용자가 정해준 일정한 영역에 화재가 발생했는지 여부를 판단한다.
 
3. 화재 감지 드론은 사용자가 정해준 일정한 영역에 발생한 화재의 위치와 규모 등을 사진 및 영상으로 촬영하여 사용자에게 즉시 알린다.


# 6.코드

## 가) 21_FireDetector.launch
launch 폴더의 21_FireDetector.launch를 참조할 것.

## 나) 21-0_Manager.py
Scripts 폴더의 21-0_Manager.py를 참조할 것.

## 다) 21-1_FlyTarget.py
Scripts 폴더의 21-1FlyTarget.py를 참조할 것.

## 라) 21-2_detect_fire.py
Scripts 폴더의 21-2_detect_fire.py를 참조할 것.

## 마) 21-3_play_alarm.py
Scripts 폴더의 21-3_play_alarm.py를 참조할 것.

## 바) 21-4_battery.py
Scripts 폴더의 21-4_battery.py를 참조할 것.

## 사) Module_Gps_Class_20.py
Pkg 폴더의 Module_Gps_Class_20.py를 참조할 것.

## 아) MoveBB2_3.py
Pkg 폴더의 MoveBB2_3.py를 참조할 것.


# 7. 화재 인식을 위한 xml 파일 만들기

화재 인식을 위한 학습 xml파일을 만들기 위하여 본 팀은 Cascade Trainger GUI(Version 3.3.1)을 썼다. 그 과정은 대략 다음과 같다.

1. 학습시키고자 하는 화재 영상을 찍는다.
2. 화재 영상에서 사진을 추출한다.
3. 화재의 사진과 그와 대비되는 사진을 분류한다.
4. 캐스케이드 프로그램에 사진이 포함된 폴더를 가리키고, 사진의 크기 · 사진의 수 · 개채의 특징 등 여러 매개변수들을 변동시켜가며 학습시킨다.
5. 캐스케이드 프로그램의 학습 결과로 나온 xml 파일을 실제 영상에 적용시켜 본다.

그 결과는 '학습 xml 정리표.xlsx'를 참조할 것.

# 8. 실험 결과

2021년 충남인력개발원의 ROS Fire_Detection_Drone 팀은  8월 16일부터 9월 2일까지 수십 차례에 걸쳐 코드를 테스트하였다. 그 중 유의미한 결과나 나온 일부 내용을 아래와 같이 정리한다.

## 가) 2021년 8월 30일


2021년 8월 30일 우리팀은 충남인력개발원 본관 앞 운동장에서 화재 감지 드론의 코드를 검증하였다.

<div align = "center">

![image](https://user-images.githubusercontent.com/84608929/131615502-f062ae06-bb7d-45d9-851c-bd464abe9ab1.png)

</div> 
 
위 그림(3)은 rqt 화면, 실행 터미널을 각각 보여준다. rqt 화면에서는 드론의 실시간 화면이 나타나고 터미널에서는 실행 명령 및 그 결과 들을 볼 수 있다. 드론이 목표 지점에 도달했다면 새로운 화면 창이 뜨고, 화재 감지 코드가 활성화될 것이다.

<div align = "center">

![image](https://user-images.githubusercontent.com/84608929/131615543-1d6e22a7-0de4-42c5-aa89-45afb4304693.png)

</div>
 
우리 팀이 짠 코드는 일정 GPS 지점까지 도달할 때까지 화재 감지를 작동하지 않는다. 그 이유는 드론의 목적이 일정 지점의 일정 범위에서 화재가 발생했는지의 여부를 수색하는 것이기 때문이다. 그러므로 위의 그림(4)에서 보는 것과 같이 목표 지점에 도달하기 rqt에서 보이는 영상은 화재 탐지의 대상이 되지 않는다. 사용자가 입력한 GPS 지점에 도달하면 화재 감식 코드가 활성화되고 새로운 프레임 창이 뜬다.

<div align = "center">

![image](https://user-images.githubusercontent.com/84608929/131615566-ee97c1b3-f2b9-4af1-bbec-64fef50088ba.png)

</div>
 
드론이 사용자가 입력한 GPS 지점에 도착하면 위의 그림(5)에서 보는 것과 같이 새로운 프레임이 생기고 화재 감식 코드를 활성화한다. 만일 화재가 감식되면 화재로 판단된 객체에 파란색 박스가 둘러쳐지고 그것을 화면의 중앙에 맞추는 비행 코드가 작동한다.

<div align = "center">

![image](https://user-images.githubusercontent.com/84608929/131615595-18e737f1-99cb-47e1-8757-ba5db1dbd66d.png)

</div>
 
그림(6)에서는 드론 코드가 화재를 인식하고 그것에 파란색 박스를 친 것을 볼 수 있고, 터미널 창에서는 “화재로 의심되는 장면을 포착하였습니다.”라는 메시지가 뜬다. 즉, 입력된 영상에서 화재를 감지한 것이다. 그런데 화재가 아닌 의자도 화재로 오인식함도 엿보인다. 이처럼 이번 프로젝트에서는 화재 인식의 정확도가 매우 중요하다.

만일, 실제 화재가 아닌 객체를 화재로 인식할 가능성이 있다. 이를 방지하기 위하여 코드에서는 화재의 객체가 화면의 중앙에 올 때까지 화재 인식을 10초 이내로 끊김없이 이어지도록 해야 한다는 조건을 만들었다.

<div align = "center">

![image](https://user-images.githubusercontent.com/84608929/131615621-7bc68a57-e9d4-40e4-a127-d42849bb8028.png)

</div>
 
화재로 감지된 객체가 화면의 중앙 범위에 맞추어졌을 때, 코드에서는 해당 영상의 이미지를 3회 연속해서 저장한다. 그런 다음 사용자에게 어느 (GPS상의) 위치에서 화재가 발생했는지를 알리고 시작 지점으로 되돌아 간다. 사용자에게는 터미널 창에서 표준 출력함과 동시에 문자 메시지로 알리는 기능이 코드에 추가되어 있다.

화재 감식의 임무를 끝냈다면, 드론이 다시 시작지점으로 되돌아 가도록 한다. 따라서 화재 감식을 할 때 보여지는 프레임도 사라지게 된다.

<div align = "center">

![image](https://user-images.githubusercontent.com/84608929/131615655-c42243af-078c-4b7d-9404-c4ffc6c61d31.png)

</div>

# 8.참고자료
강욱. “공공임무용 드론의 사회적 비용·편익 분석에 대한 연구”. 한국테러학회보, 제11권 제4호, 2018년.

국토교통부(첨단항공과). 드론 활용의 촉진 및 기반조성에 관한 법률 ( 약칭: 드론법 ), Pub. L. No. 제16420호 (2019).

김동현·김성렬. “Open_CV를 활용한 위험 상황 인식에 관안 연구”. Journal of the KIECS, 2021년 2월 16일.

김별·황광일. “딥러닝 기반의 연기 확산거리 예측을 위한 알고리즘 개발 기초연구”. Journal of the Korean, 2021년.

노예진. “’특파원보고 세계는지금’기후변화로 인한 섭씨 50도의 폭엽...최악의 산불”, 2021년 8월 14일.

문준호·최혁두·박남훈·김종희·박용운·김은태. “데이터베이스 기반 GPS 위치 보정 시스템”. Jouranl of Korea Robotics Society, 2012년.

배용민. “[119기고] 소방 드론의 활용”. 소방방재신문, 2021년 4월 7일. https://www.fpn119.co.kr/155045.

서울소방재난본부. “재난현장에서 드론의 운용 활성화를 위한 방안 연구”. 제29회 119소방 정책 컨퍼런스, 2017년 9월.

소방안전플러스 편집실. “소방안전의 파수꾼 드론[drone]”. 소방안전플러스, 2021년 4월 7일. http://webzine.kfsa.or.kr/user/0008/nd46782.do#.

소방청, 편집자. 2020년도 화재통계연감, 2021.

신열우·박진호. “경험적 접근법을 활용한 재난현장에서의 소방드론 임무수행 효율성 분석”. 한국화재소방학회,  한국화재소방학회논문지 Vol.34 No.5, 2020년.

조성완. “소방공무원의 고령화에 따른 자기효능감, 팀워크가 현장대응역량에 미치는 영향과 제도 개선방안에 대한 연구”. 서울시립대학교, 2016.

하강훈, 김재호&최재욱. “소방분야의 드론 활용방안 연구 경향 분석”. 한국산학기술학회논문지 제22권 제4호, 2021년. https://www.koreascience.or.kr/article/JAKO202113855736456.pdf.

“화마가 덮친 남유럽, 통제불능 기후변화”. 세계는지금. KBS, 2018년 8월 14일.

이용진. “1. bebop_autonomy”. Lee Yongjin (blog), 2021년 7월 9일. https://github.com/greattoe/ros_tutorial_kr/blob/master/ros1_tutorial/rospy/bebop2/bb2_1_bebop_autonomy.md.

———. “2. Parrot-Sphinx”. Lee Yongjin (blog), 2021년 7월 9일. https://github.com/greattoe/ros_tutorial_kr/blob/master/ros1_tutorial/rospy/bebop2/bb2_2_parrot_sphinx.md.

———. “3. teleop_key 노드 작성”. Lee Yongjin (blog), 2021년 7월 9일. https://github.com/greattoe/ros_tutorial_kr/blob/master/ros1_tutorial/rospy/bebop2/bb2_3_teleop_key.md.

———. “4. Odometry 토픽을 참조한 이동”. Lee Yongjin (blog), 2021년 7월 9일. https://github.com/greattoe/ros_tutorial_kr/blob/master/ros1_tutorial/rospy/bebop2/bb2_4_move_by_odom.md.

———. “5. 웹페이지에 실시간 드론 GPS 위치 표시”. Lee Yongjin (blog), 2021년 7월 9일. https://github.com/greattoe/ros_tutorial_kr/blob/master/ros1_tutorial/rospy/bebop2/bb2_5_mark_bebop2_on_web.md.

———. “6. GPS 위치정보에 의한 이동”. Lee Yongjin (blog), 2021년 7월 9일. https://github.com/greattoe/ros_tutorial_kr/blob/master/ros1_tutorial/rospy/bebop2/bb2_6_move_by_gps.md.


# 참고

문서 형식으로 보실 분은 동일 폴더의 pdf 파일을 다운받아 보시면 됩니다.
