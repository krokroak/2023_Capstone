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

사진은 Block 을 수정하다가 world frame 을



 잘못건드려서 난리가 난 모습이다.
![image](https://user-images.githubusercontent.com/66929200/223439875-01b6a4d6-5f99-4690-840b-b857e9c22d0a.png)



# 3.14 

base - follower 를 다시 수정하고 나서 만들었는데 이번에는 바퀴가 안돌아갔다.

 이유는 revolute joint 가 하나에 비해 rigid transform 이 너무 여러개인것
 너무 frame 의 수가 부족해서 생겨난 문제인것 같다.
 찾아보니까 file solid, 또는 solid 를 만드는 block 에서 frame 을 자체적으로 추가, 수정 할 수 있다는 것을 알았다.
 다음주 발표 전까지는 어느정도 완성이 되어야 할..
https://user-images.githubusercontent.com/66929200/224895526-4a51fb76-36a8-4ddc-9764-93095118ec78.mp4



# 3.15 
위의 문제에서 단순히 frame 문제 인줄 알았지만, 단순한 polygon 덩어리의 결합으로는 뭔가를 할 수 없을것 같다.
( 실력이 안되서 못하는듯)

지금 까지는 문제로
Pulley 쪽 두부분 조립이 원할하지 않다는점 --> 이것도 gear constraint 처럼 두개의 회전축이 같은 평면에 있어야 하는건가? 
그래서 Pulley constraint 를 사용하지 않고..

요약하자면 지금까지의 시뮬레이션은 우리가 만드려는 작품의 메커니즘 Pulley, Gear 이 두부분을 Multibody 로만 구현하기가 힘든 부분이 있다.
Simscape 의 Multibody에만 몰두에서 시뮬레이션을 할 것이 아니라
Simscape-Mechanism, Multibody 두부분 ( 아마 회로 까지 들어가면 Electric 까지 써야 할 것 같다.)


# 3.16 
여태 안되던 문제를 찾았다...
우리가 만들었던 샤프트를 file solid 를 통해서 만들면 .. 풀리와 완벽하게 조립이 안되는게 문제점이였다.
( 샤프트 빼고 하니까 잘돌아감 )


https://user-images.githubusercontent.com/66929200/225520315-0dc8c864-d914-4b68-a484-a289294cb415.mp4

# 3.17 ~ 3. 22 
위에 문제점들을 해결했다ㅣ 
샤프트를 그냥 원래있던 cylinder solid 로 바꿔서 부품 체결을 했다 

![image](https://user-images.githubusercontent.com/66929200/227857085-5bd82473-e728-41a2-9805-3a3917bb0b8e.png)
(simulink block diagram)
![image](https://user-images.githubusercontent.com/66929200/227860615-858ddba9-4f1e-4d2e-ac69-eaf5c6540090.png)
실제로 조립이 잘된 문제 였다..

그리고 조립을 하다가 막힌 부분이 있었는데 두개의 Arm이 샤프트에 매달려 자유운동을 해야하는데
(bearing 이 필요한 부분으로 여러가지 문제 점이 있었다)
pulley 가 돌아가면 shaft 가 돌아가는건 구현을 했지만 shaft 가 돌아감에 따라 Arm이 같이 돌아가는 문제가 생겼다.
그래서 아래 샤프트에 revolute joint 로 묶고 시작 각도를 180도로 맞춰놨다.
![image](https://user-images.githubusercontent.com/66929200/227861099-9ff9f959-e57d-4de1-838c-c57693394249.png)

그리고 spatial contact force 를 이용해서 평지에서 자세제어와 계단을 올라갈 때의 자세제어 시뮬레이션을 해볼 예정이다.

하드웨어가 완벽하게 다 주문이 되면 무게측정을 해서 file solid 의 parameter 를 다시 조정하고
풀리에서 발생하는 마찰력, 여러가지 디테일을 계속 추가해서 시뮬레이션을 진행해봐야 할 것 같다.


아래의 동영상은 제어가 추가되지않은 ( 모터에만 ) 입력을 넣은 모습이다.

https://user-images.githubusercontent.com/66929200/227863122-e6cca382-48b9-4f92-bf0d-876312bbe70a.mp4


# 3.27(KJH)
-제어기를 단 시뮬레이션 모델에 계단을 추가하여 업로드 했다.
-실험결과 로봇을 추진시키는 데까지는 성공했으나 계단을 오르게 하는과정에서 균형을 잡지 못하였다.
-추후에 시뮬레이션을 개선시켜 계단을 오르는 동작을 수행할 수 있도록 할 예정이다.

# 4.03(KJH)
-평면에서의 균형을 더 잘 잡는 게인 값을 찾아 적용함
-계단 시뮬레이션을 여러차례 시도하였으나 계단을 올라가는데는 실패함 
-아마 계단의 모서리 부분에서 마찰이 적용되지 않아 올라가지 못하고 계속 미끄러지는 걸로 추정됨
-추후 여러 방법을 통해 개선 필요


