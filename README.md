# 2023_Capstone


2023 1. 10 ~ 모터 선정, 모터 드라이버선정  
모터 MD로봇의 BL9 + ENCODER 부착형 모터 32만원 
모터 드라이버 MD500S 16.6만원


##############################################
###### 정기 발표는 매주 화요일 입니다.#########
##############################################


2023 2.1 ~ 2.9 하드웨어 부품 픽스, 설계도 작성
2023 2.10 ~ 2.16 그린 설계도 기반 simscape simulation, 부품마다 업체 찾아서 주문 할 준비 ( 비싼거 말고)
2023 2.18 ~ 2.28 하드웨어 디버깅, bldc 모터 stm으로 앵간치 돌릴줄 알아야함 
(2월 부터 3월 중순 까지는 하드웨어 수정이랑, 시뮬레이션 병행 해야함) 

###### 2023. 2. 10 feedback  ############################################################
- 베어링,샤프트,암 부분 체결하는곳 inventor 에서 다시 설계해야함
- 하우징 바깥 부분을 body1 내부에 넣고, 중간에 파이프 하나 더 박아서 하우징 두번째 부분을 따로 고정 시켜야함 
- 하우징 문제를 해결하려면 본체 가로 폭을 넓혀야 할거라고 생각됨
- 바퀴 회전축으로 부터의 아랫부분 무게와 위에 달려있는 배터리나 돌림힘의 비교가 필요함
#######################################################################################

###### 2023. 2. 14 feedback  ############################################################
- bearing ===> shaft 연결부분 1.5mm ~~~ 2.0mm로 고정 부분 넓히기
- 텐셔너 고정부분 --> Square 하지말고 원형으로 
- 바퀴부분 연결된 Shaft 에 엔코더 부착은 없는걸로 --> 엔코더 하나로 1:50, 1:3 계산 이용해서 보드로 따로 값 전달
- 파이프 --> 프로파일로 바꾸면서 용접문제, 대각선 고정 문제 해결 
- Arm 연결부분 사각형으로 만든건 ok --> 나사구멍 홀텝가공 M4
- 계단 충돌문제는 기어박스를 거꾸로 부착하면 --> ARM 길이 수정안해도됨 --> 거꾸로 붙여서 전부 해결 
- 하우징 고정은 
                커플링 - 하우징 - 암 
                                      이런 순서로 고정되어 있다.
   이런 순서에서 하우징을 고정 시키려면 
   
          커플링 - 링 - 하우징 - 링 - 암 
                                      이렇게 고정을 하면됨 
                                      
 - 조립 순서에 대해서도 생각해봐야함    
 - 조립은 우리가 다해야하는데, 프로파일로 인해서 용접 문제는 해결 되었지만 
        ARM, ARM connector, shatf, 모터 고정 부분
                                                 이부분은 직접 설계도를 줘서 만들어야험 
#######################################################################################

- simulink 상에서 계단이나 장애물을 배치 할 수 있는지
- 하드웨어 설계와 동반되는 simscape 공부
- 하드웨어 설계도 가지고 업체 찾아서 부품 주문해야함
- MCU ( STM32F103RB ) 에 모터드라이버 값을 직접 전달해줘서 MCU 에서 직접 제어 할 수 있어야함
- PID 로 제어를 할것이냐 LQR 을 구현할 것이냐. LQR 로 할거면 PINKLAB 찾아서 라그랑지안 공식 확인 // 2023.2.10 -->교수님 의견은 아마 제어기없이 근야 튜닝을 해야할거라고 하심
- bldc 모터 matlab 상에서 구현 --> 로봇학 실험 4에서 하듯이 전달함수 구해서 simscape 연결 
- 카메라 --> Rasberry PI --> STM32 --> DRIVER --> MOTOR 
( CAMERA --> MOTOR) 여기 까지 오기의 통신을 할 수 있느냐. 이 부분도 상당히 난이도가 높을 것이라고 생각함
- 계단 인식 밑,  평면상에서의 움직임제어 LEFT, RIGHT

- 배터리 윗쪽 가이드 설치, Battery Management system 확인
- PCB로 해야하는지? , 그냥 기판으로 가야하는지  --> Noise가 얼마나 심한지 확인해봐야함?

#########2023.02.25 업데이트###################
- 시뮬레이션 프로그램 (matlab simscape multibody 기반 PID제어기 버전) 업로드 완료 확인바람
-

#################################################
#################  SENSOR, IC ###################
#################################################

IC --> LM7805, 5v Regulator
Sensor
--> MPU6050 or MPU9250 
--> encoder ( 암에 부착될 엔코더)



final_step. 계단 오르기 ...
( 2023. 2. 09에 작성한 글입니다. 향후 계획은 단계마다 바뀔 수 있습니다.)
