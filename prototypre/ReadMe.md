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

# 3.6 


이전 실패작과는 다르게
Simulink Simscape Multibody 에서 생각을 달리 해봤다
두개의 바퀴를 같은 공간 다른 좌표에 두고 
위에 문제점인 base_frame을 body 부분이 아니라 바퀴로 두었다 
바퀴의 회전축과 샤프트 회전축을 동일하게 맞췄다.
타이밍 풀리 위, 아래의 회전축을 동일하게 맞췄다. --> 위쪽 샤프트가 +Z 방향으로 회전하게 되면, 아래쪽 샤프트가 -Z 방향으로 회전하게 된다.

![image](https://user-images.githubusercontent.com/66929200/223115952-01cdce5c-f00e-469e-b5bb-fa2c4853c7ff.png)
![image](https://user-images.githubusercontent.com/66929200/223116980-0a4d8bad-310f-4629-a01d-993db49a58b9.png)


# 3.7


일단 바퀴에 theta 를 넣어서 제대로 움직이는 지 확인을 해보려 했으나

바퀴에 40도 각도를 주면 
샤프트에 큰풀리 작은 풀리가 다 묶여있어서 풀리만 도는게 아니라 다같이 확돌아버렸다. 
나도 돌겠다.

사진은 Block 을 수정하다가 world frame 을 잘못건드려서 난리가 난 모습이다.
![image](https://user-images.githubusercontent.com/66929200/223439875-01b6a4d6-5f99-4690-840b-b857e9c22d0a.png)
