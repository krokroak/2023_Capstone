# 3.1 ~ 3.6 



simscape multi-body 를 통해  1 : 2 기어비를 구현하는데 성공
이 두개의 기어의 중심이 한 평면 상에 존재 해야  
Matlab simulink 의 Gear Constraint 박스에 묶일 수 있다.

![gear_make (2)](https://user-images.githubusercontent.com/66929200/223035130-0924b8fb-b003-4957-9d5c-606f16d8ffff.gif)


이 부분에서 이어서 

샤프트, 암, 암 두개를 이어주는 부분을 inventor 부품을 가져와서 file solid 블록에 적용하고

body 부분을 base_frame으로 시작해서 flower 를 바퀴까지 연결했더니 바디 중심으로 두개의 암이

자유 진자 운동을 하는것으로 보였다. 

![prototype_2 (3)](https://user-images.githubusercontent.com/66929200/223037730-e9d1c27e-c612-4c62-8bc7-99ccb83251e8.gif)

이런 모습을 보이는 시뮬레이션은 완전히 잘못 구현한 시뮬레이션이다.


빠르면 이번주 안으로 늦어도 다음주 안으로 base follower를 수정해서.

Wheel 이 base frame 으로 설정하고

arm 과 body 두 부분이 ( 보기에 따라 다르겠지만 )  double pendulum의 자유 운동처럼 보이게 다시 만들어야 한다.

####################################################################################################
####################################################################################################