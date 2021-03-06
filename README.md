# Basic-car
Nxt Lego Mindstorm을 이용해 차체를 제작하고 이 차체를 소프트웨어적으로 동작시키는 펌웨어를 제작하였다.
이 프로그램을 실제 차량에 접목시키기 위해 실제 차와 유사하게 섀시 및 바디를 레고 블록과 Nxt Lego Mindstorm을 이용하여 제작하였으며, Nxt Lego Mindstorm에 올라갈 펌웨어로는 
real-time 운영체제인 nxtOSEK과 사용자 어플리케이션을 실행파일로 묶어 사용한다. 펌웨어는 운전자가 직접 차량을 운전하는 상황을 가정해 휴대폰 앱에서 
블루투스 기능을 활용하여 직접 자동차의 동작을 제어할 수 있도록 만들었다.
# Author
임채진                     
정지원
# 요구사항
### 1. 전진 및 후진
위 화살표를 누르면 전진, 아래 화살표를 누르면 후진이 되도록 한다.
### 2. 좌회전 및 우회전
왼쪽 화살표를 누르고 있을 시, 앞 바퀴 축이 왼쪽으로 회전하고, 오른쪽 화살표를 누르고 있을 시, 오른쪽으로 회전한다.
### 3. 고속 및 저속
고속 버튼을 누르면 차량의 속도가 빠르게 설정되고, 저속 버튼을 누르면 속도가 느리게 설정된다.
### 4. 브레이크 모드
즉시 버튼을 누르면 모터가 곧바로 멈추도록 설정되고, 천천히 버튼을 누르면 모터가 천천히 멈추도록 설정된다.
### 5. 브레이크
A 버튼을 누르고 있으면, 차량의 모터가 멈춘다.
### 6. 정적
B버튼을 누르면 차량에서 경적이 울린다.
# 구현
### 1. Task(nxtcar)
nxtcar Task alarm 기능을 통해 5ms마다 실행되게 된다. 해당 Task는 지속적으로 실행되며 배열의 값을 확인하고 if문을 통해 배열에 값이 들어온 것을 확인하면 해당 기능을 직접 처리하거나 SetEvent()를 통해 다른 Task로 해당 event를 넘겨주는 기능을 한다.
### 2. 전진 및 후진
3번 배열에 1 또는 2가 들어갈 경우에 nxtcar Task에서 해당 배열의 값이 1일 경우 forward, 2일 경우 backword event를 setEvent()를 통해 MotorTask로 넘겨준다.           
MototTask Task는 해당하는 이벤트들이 들어올 경우, nxt_motor_set_speed를 통해 알맞은 방향으로 모터를 동작시킨다.        
brake가 기능하고 있을 때 모터가 움직이는 것을 방지하기 위해 리소스 res1을 사용하도록 하여 브레이크가 res1을 가지고 있을 때에는 동작하지 않는다.       
### 3. 좌회전 및 우회전
nxtcar Task에서nxt_motor_set_speed() API를 이용하여 4번 배열에 1의 값이 들어오면 A모터에 -60의 속도를 2의 값이 들어오면 60의 속도를 할당하였다.
### 4. 고속 및 저속
전역변수로 (int)speed를 만들어 0으로 초기화를 해주었다.
5번 배열에 1 또는 2의 값이 들어가면 nxtcar Task에서 해당 배열의 값이 1일 경우 fast, 2일 경우 slow의 event를 setEvent()를 통하여 velocity Task로 넘겨준다.
velocity Task는 해당 event를 받게 되면 fast 의 경우 speed를 100으로 설정하고, slow의 경우 80으로 설정하게 된다.
### 5. 브레이크 모드
mode2라는 전역변수를 만들어 6번 배열에 1이 입력될 경우 1의 값, 2가 입력될 경우 0이 들어가도록 하였다. mode2의 0과 1은 enum을 이용하여 Float와 Brake로 지정하였다. mode2에 저장된 값은 모터의 nxt_motor_set_speed의 세 번째 파라미터로서 사용된다.
### 6. 브레이크
7번 배열에 1의 값이 들어올 경우 nxtcar Task에서 pause event를 StopTask에 넘겨준다.
StopTask에서 해당 이벤트를 받을 경우 GetResource를 이용해 res1 리소스를 얻음으로써 MotorTask Task가 동작하지 못하도록 하고, nxt_motor_set_speed()를 이용해 모터 B, C를 멈추도록 한다. 이 때 세 번째 파라미터는 브레이크 모드에서 설정한 mode2를 사용한다.
그 후에는 모터 동작을 방지하기 위해 리소스를 바로 release하지 않고 release event를 기다리게 된다.
A버튼에서 손을 떼면 release event가 발생하게 되고, res1 리소스를 놓아주어 MotorTask Task에서 모터가 동작할 수 있게 된다.
### 7. 정적
7번 배열에 2의 값이 들어올 경우 ecrobot_sound_tone() API를 이용, 특정 음이 1초간 출력되도록 하였다.
# 휴대폰 Application(Android Only)
링크 : https://drive.google.com/file/d/1KLePUlIggyI366MFMeKe1apX_o-uGGMi/view?usp=sharing                  
블루투스 버퍼에서 사용하는 배열 요소는 3,4,5,6,7 입니다.    
                
3 - 전진, 후진(1, 2)               
          
4 - 좌, 우(3, 4)              
          
5 - 고속, 저속(1, 2)             
             
6 - 즉시, 천천히 (브레이크 모드) (1, 2)            
            
7 - A(브레이크), B(정적) (1, 2)           
        
입니다.            
               
각각 배열의 상태는 1과 2로 구분합니다. 다만 4번 배열만 3(좌), 4(우) 로 구분합니다.                 
                      
4번과 7번배열은 버튼 누르고 있을 시 해당하는 값을 전송하고
버튼에 손을 때면 0으로 자동 초기화 됩니다.
# 사용 방법
1. 컴파일한 실행 파일(rxe 파일)을 Nxt Lego Mindstorm에 임포트한 후 Nxt Lego Mindstorm에서 펌웨어를 실행한다.            
2. 휴대폰 app을 실행시킨 후 휴대폰을 이용해 자동차의 동작을 컨트롤한다.    
# 자동차 모습
Nxt Lego Mindstorm과 레고를 이용해 제작한 자동차 모형을 링크를 이용해 첨부합니다.              
자동차 이미지 링크 : https://drive.google.com/file/d/19ElpvvJCCBZH7ghfeE1yhKE7aLIH9HAy/view?usp=sharing
