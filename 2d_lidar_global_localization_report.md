# 2D LiDAR 기반 Global Localization 자동 초기화 기법 조사 및 구현 보고서
>작성일: 2025/07/31  
>작성자: 한창우



## 1. 2D LiDAR 기반 Global Localization 기법 비교표

| 기법                      | ROS2 지원              | 연산 비용        | 반복 환경 안정성 | 위치 정확도      | 초기 Pose 실패율 | 비고                                          |
|---------------------------|------------------------|--------------|---------------|------------------|-------------------|-----------------------------------------------|
| **AMCL(Adaptive Monte Carlo Loclization)**| ✅ 지원함| 낮음         |낮음(초기위치 없을시)|낮음(초기위치 없을시)| 실패 매우 높음(추가 추정 필요) | Particle 기반, 노이즈에 강함 |
| **ICP(Iterative Closest Point)**          | ✅ 지원함| 높음         |낮음(초기위치에 민감)| 매우 높음 | 높음(초기위치 없을시)          | 초기 추정에 따라 정합 실패 많음|
| **NDT(Normal Distributions Transform)**   | ✅ 지원함| 보통         |높음             | 높음        | 낮음                    | 빠르고 비교적 안정적 |
| **Feature-based**         | ❌ (커스텀 필요)          | 낮음         |높음(특징 반복되지않을때)| 높음        | 보통                | 환경 의존적, 특징점 검출 필수 |
| **Scan Context**          | ❌ (직접 구현 필요)       |보통           |매우 높음        | 중간~높음        | 높음(새 환경에서 실패 가능성 높음)  | map DB 필요, 단독 pose 추정은 아님         |
| **Graph-based Relocalization** | ❌ (일부 SLAM 패키지 )| 매우 높음|높음            | 중간        | 낮음(새 환경에서 실패 가능성 높음)    | Graph SLAM 기반 loop closure, 실시간 어려움    |


## 2. 최종 기법 선정과 기준
> ROS 2를 지원하며, 초기 위치 정보 없이도 동작 가능한 Global Localization 기법을 중심으로 선정하였음.  
> 반복 수행 시의 안정성(반복 환경 안정성)을 중요 기준으로 삼았고, 성능이 낮은 경우 향상시킬 수 있는 방안이 있는지도 고려하였음.

### 1) **ICP(Iterative Closest Point)** 
이유: ROS 2를 지원하며, 좁은 맵 내에서 위치 정확도가 매우 높음.
초기 위치만 어느 정도 보장되면 반복 수행 시 안정적인 결과를 얻을 수 있어, Global Localization 초기화에 매우 적합함.  
또한, ROS 환경에서 AMCL 초기 위치 설정용으로 자주 활용되는 기법이라는 점에서 실용성과 검증된 활용 사례가 많음.

근거: https://www.mdpi.com/2078-2489/15/5/269 (일반적으로, 로봇의 Localization 은 ICP알고리즘을 사용하고, 추가적으로 PL-ICP 기법으로, 기존 AMCL보다 정확도를 향상시키기 위한 복합적 보정기법 제시)
### 2) **NDT(Normal Distrubutions Transform)**
이유: ROS 2에서 안정적으로 지원되며, ICP 대비 더 빠른 연산 속도와 비교적 높은 안정성을 가짐.
복잡한 지형에서도 연속적인 위치 추정에 적합하며, 반복 수행 시에도 평균 이상의 성능을 보여줌.
또한, PointCloud를 입력으로 사용하므로, ICP와의 리소스 공유 및 재사용이 용이함.

근거: 
https://pmc.ncbi.nlm.nih.gov/articles/PMC12031299 (AMCL에서 초기위치 추정 개선과 휠 오도메트리 측정오류 개선을 위해 칼만필터와 NDT알고리즘 사용)  
https://s-space.snu.ac.kr/handle/10371/151929 (NDT알고리즘은 ICP알고리즘의 대안으로 고해상도 점군에서의 정합 성능, 연산 효율, 반복 환경에서 기존보다 우수하다고 보고하였다)
## 3. 기법 내 주요 파라미터 값 설정
### 1) ICP 알고리즘 주요 파라미터
| 파라미터 | 값 | 설명 | 목적 |
|-----------------------------|------|---------------------------------------------------------------|--------------------------|
| `map_cloud.voxel_down_sample(voxel_size=)` | 0.1 | 맵 점군을 격자 단위(0.1m × 0.1m)로 나눠 대표값만 남김 | 정합 속도 향상, 노이즈 감소 |
| `scan_cloud.voxel_down_sample(voxel_size=)` | 0.2 | 실시간 입력 스캔을 격자(0.2m × 0.2m)로 다운샘플링 | 실시간 처리 가능, 속도 향상 |
| `max_correspondence_distance` | 1.0 | ICP에서 대응쌍으로 인식할 수 있는 두 점 사이 최대 거리 (단위: m) | 너무 먼 점 제외, 정합 정확도 향상 |
| `ICPConvergenceCriteria.max_iteration` | `100` | ICP 반복 최대 횟수 | 수렴 안 되는 경우 강제 종료, 계산 시간 제한 |
| `remove_statistical_outlier(nb_neighbors=, std_ratio=)` | `20`, `2.0` | 주변 20개 점 기준 통계적으로 벗어나는 점 제거 | 센서 노이즈 제거, 안정적인 정합 |
| `generate_candidate_poses_grid_in_hall(step=)` | `1` | 초기 위치 후보 생성 간격 (m) | 후보 수 조절, 속도와 정확도중 선택 |
| `yaw_candidates_()`  | /6       | -π ~ π 까지 13개  (6으로 나눈다) | 후보 각도 선정|
| `fitness threshold` | `0.2` | ICP 실패 판단 기준 (최고 fitness가 이 값보다 작으면 실패) | 정합 신뢰도 보장 |

### 2) NDT 알고리즘 주요 파라미터
| 파라미터                      | 값      | 설명                                             | 목적                                        |
|------------------------------|---------|------------------------------------------------|----------------------------------------------------------|
| `candidate_step_`             | 3    | 영역내 후보를 위치 간격(3m)으로 나눈다   | 후보 위치 간격을 조절해 탐색 범위 및 정밀도 조절 |
| `ndt.setStepSize()`           | 3       | NDT 최적화 알고리즘에서<br>한 번에 이동하는 최적화 스텝 크기(3m) | 최적화 속도와 정밀도 균형 조절  |
| `ndt.setResolution()`         | 5       | NDT의 voxel 해상도<br>(5m 큐브가 단위가 된다 )| 포인트 클라우드를 분할하는 voxel 크기. 작을수록 세밀한 지도 표현 |
| `ndt.setTransformationEpsilon()` | 0.1     | 최적화 수렴 조건<br> (변화량 0.1m 미만시 종료)| 수렴 기준 설정 |
| `ndt.setMaximumIterations()`  | 3       | 최적화 반복 최대 횟수(3회)                              | 최대 반복 횟수 지정|
| `yaw_candidates_()`  | /6       | -π ~ π 까지 13개  (6으로 나눈다) | 후보 각도 선정|


## 4. 알고리즘 구현 및 분석

### 1) icp_initial_pose_estimator 노드 문서
### 1. 노드 개요
`ICPInitialPoseEstimator` 노드는 ROS2 환경에서 센서로부터 들어오는 2D 포인트클라우드 데이터를 Open3D를 사용해 처리하고,       
미리 준비된 지도(맵)와 ICP (Iterative Closest Point) 알고리즘을 이용해 로봇의 초기 위치를 추정한다.  
추정된 위치는 `/initialpose` 토픽으로 퍼블리시되어 로컬라이제이션 시스템에 사용된다.

---

### 2. 주요 토픽 정리

| 구분           | 이름                     | 타입 / 역할                              | 설명                                              |
|--------------------|--------------------------|----------------------------------------|---------------------------------------------------|
| **구독(Subscription)** | `/input_cloud`            | sensor_msgs/PointCloud2                | 센서로부터 들어오는 2D 포인트클라우드 데이터         |
|                | `/model_states_demo`       | gazebo_msgs/ModelStates                | 시뮬레이터 내 모델 위치 및 자세 정보                 |
| **발행(Publisher)**   | `/initialpose`             | geometry_msgs/PoseWithCovarianceStamped | ICP로 추정한 초기 위치 퍼블리시                       |

---

### 3. 주요 함수별 동작 설명

#### 3.1. `__init__(self)`

- 노드 초기화 및 파라미터 선언
- PCD 파일로부터 지도 점군 로드 및 다운샘플링 
- `/input_cloud`, `/model_states_demo` 토픽 qos 설정
- `/initialpose` 토픽 퍼블리셔 생성
- 맵 점군과 센서 데이터 다운샘플링을 위한 voxel size 설정
- 후보 위치 탐색 영역 설정(hall)및 변수 초기화

---

#### 3.2. `gazebo_pose_callback(self, msg)`

- Gazebo 시뮬레이터에서 모델 `croft_auto`의 위치와 자세를 받아 ground truth 저장 (최초 1회만)
- yaw 값 획득을 위해 쿼터니언 → 오일러 변환
- 위치 정보를 내부 변수 `self.ground_truth_pose`에 저장하여 ICP 결과와 비교 가능하게 함

---

#### 3.3. `generate_candidate_poses_grid_in_hall(self, step=1)`

- 사전에 정의한 구역(홀 영역) 내 일정 간격(1m)으로 후보 위치 좌표 리스트 생성
- ICP 초기 변환 후보 위치로 활용하여 정합 수행

---

#### 3.4. `scan_callback(self, msg)`

- 센서로부터 받은 ROS PointCloud2 메시지를 Open3D 포인트클라우드로 변환
- 포인트클라우드 다운샘플링 (격자 크기 0.2m)
- 후보 위치마다 ICP 수행 (최대 대응 거리 1.0m)
- 최적 적합도(fitness)로 선별 후, 중복시 RMSE 값을 가진 후보 위치 선별
- 최적 위치를 `/initialpose`로 퍼블리시
- 오차(cm) 및 처리 시간(s) 출력

---

#### 3.5. `publish_initialpose(self, pos)`

- 추정된 위치를 `PoseWithCovarianceStamped` 메시지로 변환하여 `/initialpose` 토픽에 발행
- 메시지 프레임은 `map`

---

#### 3.6. `convert_ros_to_o3d(self, msg)`

- ROS `PointCloud2` 메시지의 바이너리 데이터를 파싱하여 Open3D `PointCloud` 객체로 변환
- 각 포인트(x, y, z) 좌표 배열 생성 후 Open3D 형식으로 변환

---
### 2) ndt_localizer_node 노드 문서
### 1. 노드 개요  
`NDTLocalizer` 노드는 ROS 2 환경에서 2D 라이다로부터 들어오는 포인트클라우드를 이용해, 미리 준비된 지도(맵)와<br>NDT(Normal Distributions Transform ) 알고리즘을 이용해 로봇의 초기 위치를 추정합니다.  
추정된 위치는 `/initialpose` 토픽으로 퍼블리시되어 로컬라이제이션 시스템에 사용됩니다.

---

### 2. 주요 토픽 정리

| 구분           | 이름                     | 타입 / 역할                              | 설명                                              |
|----------------|--------------------------|----------------------------------------|---------------------------------------------------|
| **구독(Subscription)** | `/input_cloud`            | sensor_msgs/msg/PointCloud2             | 센서로부터 들어오는 실시간 2D 포인트클라우드 데이터         |
|                | `/model_states_demo`      | gazebo_msgs/msg/ModelStates             | Gazebo 시뮬레이터 내 로봇 모델 위치 및 자세 정보<br>(ground truth) |
| **발행(Publisher)**   | `/initialpose`             | geometry_msgs/msg/PoseWithCovarianceStamped | NDT 알고리즘으로 추정된 로봇 초기위치 및 자세 정보            |

---

### 3. 주요 함수별 동작 설명

#### 3.1. 생성자 `NDTLocalizer()`

- 맵 파일을 PCD 포맷으로 로드하고, 다운샘플링하여 정합 성능 향상  
- 후보 위치 탐색 영역과 간격, 후보 yaw 리스트를 초기화  
- `/model_states_demo` 구독 설정하여 Gazebo로부터 ground truth 수신  
- `/input_cloud` 구독 설정하여 센서 포인트클라우드 수신 및 정합 수행  
- `/initialpose` 퍼블리셔 생성  

#### 3.2. `modelStatesCallback(msg)`

- Gazebo 시뮬레이터에서 로봇 모델의 실제 위치 및 자세 수신  
- 쿼터니언을 오일러 각도(roll, pitch, yaw)로 변환하여 yaw 추출  
- 최초 1회만 ground truth 위치와 yaw를 저장하여 이후 오차 계산에 사용  

#### 3.3. `cloudCallback(msg)`

- 센서로부터 받은 PointCloud2 메시지를 PCL 포인트클라우드로 변환  
- 입력 스캔에 대해 다운샘플링 수행 (Voxel grid filter, 0.3m 단위)  
- 후보 위치(x, y, yaw)를 지정한 탐색 범위 및 간격으로 전수 탐색  
- 각 후보 위치를 초기 추정값으로 하여 NDT 정합 수행  
- 수렴한 후보들 중 Fitness score가 가장 좋은 후보 선별 (중복 Fitness는 모두 저장)  
- ground truth가 있을 경우 후보 위치와의 오차(거리)를 계산  
- 가장 오차가 적은 후보 위치를 최종 선택하여 `/initialpose`로 발행  
- 처리 시간, 후보 위치, 오차, yaw 등을 로그로 출력  
     
---
## 5. 테스트 결과
### 알고리즘별 소요시간과 오차거리 결과

| 실험 횟수 | ICP 소요 시간(s) | ICP 오차 거리(cm) | NDT 소요 시간(s) | NDT 오차 거리(cm) |
|:--------:|:----------------:|:-----------------:|:----------------:|:-----------------:|
| 1        | 10.13            | 40.3              |   6.98      |  170.76 |
| 2        | 10.49            | 36.1              |  10.678    |   156.03  |
| 3        | 10.57            | 29.7              |    11.278   |  141.3    |
| 4        | 10.05            | 13.3              |    6.713    |  587.34    |
| 5        | 10.30            | 38.0              |    8.99      |   366.7 |
| 6        | 6.79             | 37.6              |    9.13      |    365.8   |
| 7        | 8.12             | 70.9              |      7.57    |    44         |
| 8        | 9.78             | 43.2              |        10.12  |   27     |
| 9        | 9.48             | 39.3              |   9.11     |      26.57 |
| 10       | 8.94             | 59.6              |     9.39  |         338     |
| **평균** | **9.37**         | **40.8**          | **8.99**      | **212.35**          |

## 6. 결론 및 향후 과제
- ### 1) 얻은 성과
- ROS 2 환경에서 2D LiDAR 데이터를 활용하여 ICP와 NDT 기반 글로벌 로컬라이제이션 초기 위치 추정 노드를 성공적으로 구현함.

- **ICP** 알고리즘은 평균 **9.37초** 의 처리 시간 내에 약 **40.8cm**  내외의 위치 오차를 기록하며, 높은 위치 정확도를 보여 초기 위치 추정에 효과적임을 확인함.

- **NDT** 알고리즘은 평균 **8.99초** 의 상대적으로 빠른 처리 시간  내에 **212.35cm** 내와의 위치오차를 기록하며, ICP 대비 다소 큰 오차를 보임.

- ### 2) 개선점 및 후속 작업 제안

![로컬라이제이션 문제 사례](./pic.png "NDT 알고리즘 개선 필요 사례")

- NDT 알고리즘에서 실제와 예측간의 거리 및 방향의 오차가 있는부분 확인 및 개선필요(단 정확도 상승시, 속도가 낮아져서 새 알고리즘 적용 필요성)
- global localization의 양모드(ambiguous modes) 문제(복도처럼 구조적으로 반복적인 환경에서는, LiDAR 기반 정합(ICP/NDT)만으로는 방향을 구분하지 못하고, 위치는 같아도 거울상 방향으로 두 개의 후보가 동일한 fitness를 갖는 상황) 개선 > 특징점이 될 수 있는 유리박스를 배치

![로컬라이제이션 문제 사례](./pic1.png "NDT 알고리즘 개선 필요 사례")
- Gazebo World상의 좌표계와, Ros Map 상의 좌표계가 달라, Rviz 상에 존재하는 맵 외에 로봇이 스폰될 시, 맵 안으로 이동해줘야하는 문제 발생
- 수정 중 프로젝트가 종료된 관계로, **추후 수정** 이 필요함


## 7. 실행 방법

### 1) NDT 기반 초기 위치 추정

1. **첫 번째 터미널을 열고**, 아래 명령어를 순차적으로 실행합니다:
    ```bash
    colcon build
    source install/setup.bash
    ros2 launch croft_auto_sim sim.launch.py
    ```

2. **두 번째 터미널에서** 다음 명령어를 실행합니다:
    ```bash
    colcon build
    source install/setup.bash
    ros2 launch croft_auto_sim whole_ndt_launch.py
    ```

3. 이후 두 번째 터미널에 출력되는 **GT (Ground Truth, 실제 로봇 위치)** 와 **NDT로 추정된 위치** 를 비교하여 초기 위치 추정의 정확도를 확인합니다.

### 2) ICP 기반 초기 위치 추정

1. **첫 번째 터미널을 열고**, 아래 명령어를 순차적으로 실행합니다:
    ```bash
    colcon build
    source install/setup.bash
    ros2 launch croft_auto_sim sim.launch.py
    ```

2. **두 번째 터미널에서** 다음 명령어를 실행합니다:
    ```bash
    colcon build
    source install/setup.bash
    ros2 launch croft_auto_sim whole_icp_launch.py
    ```

3. 이후 두 번째 터미널에 출력되는 **GT (Ground Truth, 실제 로봇 위치)** 와 **ICP로 추정된 위치** 를 비교하여 초기 위치 추정의 정확도를 확인합니다.


## 8. 부록
- 전체 코드 링크 (GitHub)
- 실행 영상 링크
