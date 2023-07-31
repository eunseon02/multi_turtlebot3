# Object Detection using 2D LiDAR





System Architecture
![image](https://github.com/eunseon02/multi_turtlebot3/assets/108911413/2fef21a5-3c62-479c-bbd7-565bf1194350)


1. Clustering(클러스터링)
LiDAR 센서로부터 데이터는 극좌표계로 획득되어 이를 직각 좌표계로 변환한다. 물체를 인식하기 위해서는 2D LiDAR 측정치를 클러스터링하는 작업이 필요하다. 센서의 정보는 2D LiDAR를 원점으로 하는 데카르트 좌표에서 위치를 포함하는 정보를 가진다. 특정 두 점 사이의 유클라디안 거리가 임계 거리 이하이면 같은 클러스터로 묶는 방식을 이용한다.

 하지만 고정된 임계 거리를 이용하면 센서로부터의 먼 거리에 있는 물체일수록 LiDAR point cloud 간격의 증가로 클러스터링 능력이 현저하게 떨어지게 되는 LiDAR Sparsity 문제가 발생한다. 이 문제를 해결하기 위해서 임계 거리()를 LiDAR frequency마다 업데이트한다. 따라서 임계 거리()값은 LiDAR로부터의 거리에 따라 변하는 특징을 갖게 된다.
 
 따라서 클러스터링을 구현하는 방법으로 Adaptive Breakpoint Detector Algorithm(ABDA)을 이용한다.

 ![image](https://github.com/eunseon02/multi_turtlebot3/assets/108911413/e9ff6622-c257-4b6e-8a02-80a742187d8a)


2. Circle Extraction

4. Data Association(구현 중)
프레임 간의 일치하는 물체들을 매칭시키기는 과정이다. 즉, 현재 탐색된 특징점이 현재 지도에 등록된 특징점 중 어느 특징점에 해당하
는지 판단하는 과정이다. data association 과정이 필요하다.

5. Circle tracker(구현 중)
로봇에 장착된 센서만으로 주변 동적/정적 객체의 위치를 추정하는 과정이 필요하다. 하지만 이러한 위치 추정은 센서 관측정보가 불확실하다는 점과 실제 우리 환경에서 동작하기 위하여 실시간성을 확보해야 한다는 점 등 많은 문제점을 가지고 있다. Kalman Filter는 연산 과정이 빠르기 때문에 실시간 문제 및 임베디드 시스템의 여러 분야에서 자주 사용되고 있어 LiDAR를 통해 인식한 객체의 위치 추정하기 위해 Kalman Filter를 선정하였다. 추정은 입력된 자료가 불완전하거나 불확실하더라도 사용할 수 있는 계산된 결과의 근삿값을 말한다. 선형 시스템에서 추정 시 Kalman Filter를 이용하고 비선형 시스템 추정 시 Unscented Kalman Filter를 이용한다. 다만 Kalman Filter만으로는 객체의 위치와 속도를 정확하게 알아내기는 어렵다. 대신 실제 속도와 위치가 어느 범위 안에 속한다는 것 정도는 예측할 수 있다.

- Unscented Kalman Filter
- 
  $$x_CTM =  \begin{bmatrix} x& y& v_x&v_y& \omega  \end{bmatrix} ^T$$





실행방법
-Turtlebot3 관련 패키지 다운로드
workspace directory에서 
roslaunch multiple_turtlebots_sim multi_robot.launch
roslaunch multiple_turtlebots_sim lidar.launch

##구현 영상



https://github.com/eunseon02/multi_turtlebot3/assets/108911413/cc826f3b-c4d4-47c7-a01b-20741281c243


