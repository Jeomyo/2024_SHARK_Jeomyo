# 🚗 2024 성남 SHARK - YOLOv3 기반 차간 간격 유지 시스템 (with MORAI)

## 📌 시스템 구조 (System Architecture)

아래는 센서부터 제어 명령까지 전체 처리 흐름을 보여주는 플로우차트입니다.
<p align="center">
  <img src="https://github.com/user-attachments/assets/14f3428d-d4b7-4c85-82bd-5704563205b7" alt="이미지 설명" width="600">
</p>


## 📌 패키지 상세 구성 및 주요 기능

# 1. GPS IMU 데이터 Publish

## 1) gpsimu_parser.py - GPS & IMU 데이터 파싱 및 Odometry 생성
|역할|GPS 및 IMU 데이터를 수신하고, UTM 좌표 변환 후 /odom으로 퍼블리시|
|------|------|
|구독 토픽| /gps (GPSMessage), /imu (Imu)|
|퍼블리시 토픽| /odom (Odometry)|
|주요 기능 요약| GPS 좌표를 UTM 좌표로 변환하여 위치 정보 제공, IMU에서 차량의 자세(Orientation) 정보 제공|

<details>
<summary> <b> 📌 gpsimu_parser 코드 분석 펼쳐보기 </b> </summary>
  
### 1. 노드 초기화 및 토픽 설정
  
```python
rospy.init_node('GPS_IMU_parser', anonymous=True)
self.gps_sub = rospy.Subscriber("/gps", GPSMessage, self.navsat_callback)
self.imu_sub = rospy.Subscriber("/imu", Imu, self.imu_callback)
self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=1)
```
* /gps에서 GPS 데이터를 수신
* /imu에서 IMU 데이터를 수신 (자세 정보)
* /odom으로 최종 변환된 위치 및 자세 정보 퍼블리시
* 10Hz 주기로 동작

### 2. GPS 수신 및 저장 (navsat_callback)
```python
def navsat_callback(self, gps_msg):
    self.lat = gps_msg.latitude
    self.lon = gps_msg.longitude
    self.e_o = gps_msg.eastOffset
    self.n_o = gps_msg.northOffset
    self.is_gps = True
```
* GPSMessage 메시지를 받아서 위도(lat), 경도(lon), 동쪽 오프셋(eastOffset), 북쪽 오프셋(northOffset)을 저장
* 데이터가 들어왔음을 self.is_gps로 표시해 이후 처리 시 플래그로 활용

### 3. GPS 위도/경도를 UTM 좌표로 변환 (convertLL2UTM)
```python
xy_zone = self.proj_UTM(self.lon, self.lat)

if self.lon == 0 and self.lat == 0:
    self.x = 0.0
    self.y = 0.0
else:
    self.x = xy_zone[0] - self.e_o
    self.y = xy_zone[1] - self.n_o

self.odom_msg.header.stamp = rospy.get_rostime()
self.odom_msg.pose.pose.position.x = self.x
self.odom_msg.pose.pose.position.y = self.y
self.odom_msg.pose.pose.position.z = 0.
```
* UTM 좌표계 변환 (pyproj 라이브러리 활용)
* 변환된 UTM 좌표에서 GPS 오프셋(eastOffset, northOffset)을 보정
* 변환 결과를 /odom 메시지의 position에 저장

### 4. IMU 데이터 수신 및 orientation 저장 (imu_callback)
```python
if data.orientation.w == 0:
    self.odom_msg.pose.pose.orientation.x = 0.0
    self.odom_msg.pose.pose.orientation.y = 0.0
    self.odom_msg.pose.pose.orientation.z = 0.0
    self.odom_msg.pose.pose.orientation.w = 1.0
else:
    self.odom_msg.pose.pose.orientation.x = data.orientation.x
    self.odom_msg.pose.pose.orientation.y = data.orientation.y
    self.odom_msg.pose.pose.orientation.z = data.orientation.z
    self.odom_msg.pose.pose.orientation.w = data.orientation.w

self.is_imu = True
```
* /imu에서 수신한 orientation를 odom 메시지에 저장
* IMU 데이터가 들어온 상태를 self.is_imu 플래그로 표시

### 5. 메인 루프에서 GPS와 IMU 데이터 결합 후 Odometry 퍼블리시
```python
 if self.is_imu == True and self.is_gps == True:
    self.convertLL2UTM()
    self.odom_pub.publish(self.odom_msg)
```

* GPS와 IMU 데이터가 모두 수신되면, 좌표 변환 및 자세 결합 후 /odom 퍼블리시
* GPS, IMU 데이터 누락 시 경고 메시지 출력

### 📝 요약
|기능|설명|
|---|---|
|GPS 수신|/gps에서 위도, 경도, 오프셋 수신|
|IMU 수신|/imu에서 자세 데이터 수신|
|UTM 변환|위경도 → UTM 변환 및 오프셋 보정|
|Odometry 생성|위치+자세 결합 후 /odom 퍼블리시|
|주기적 동작|10Hz 주기로 데이터 수신 여부 체크 및 퍼블리시|
</details>




# 2. 글로벌 & 로컬 경로 계획

### 경로 계획이란?
* 목표 지점까지 최적의 경로로 도달하게 하는 기술
* 경로 계획은 크게 전역 경로 계획과 지역 경로 계획으로 나눌 수 있음.

| 구분 | 설명 |
|---|---|
| 전역 경로 계획 | 지도에 기반하여 경로를 자율적으로 생성 |
| 지역 경로 계획 | 전역 경로에서 얻은 구간을 주행하기 위하여 차량이 실제로 주행해야 할 경로 |


## 1) global_path_pub.py - 전역 경로 퍼블리시
|역할|저장된 경로 파일(mando_path.txt)을 읽어 /global_path로 퍼블리시|
|------|------|
|구독 토픽| X|
|퍼블리시 토픽| /global_path (Path)|
|주요 기능 요약|전역 경로 제공|
<details> <summary><b> 📌 global_path 코드 분석 펼쳐보기 </b></summary>

### 1. 전역 경로 파일 불러오기 
  
```python
rospack = rospkg.RosPack()
pkg_path = rospack.get_path('beginner_tutorials')
full_path = pkg_path + '/path/mando_path.txt'

with open(full_path, 'r') as f:
    lines = f.readlines()
    for line in lines:
        tmp = line.split()
        read_pose = PoseStamped()
        read_pose.pose.position.x = float(tmp[0])
        read_pose.pose.position.y = float(tmp[1])
        read_pose.pose.orientation.w = 1
        self.global_path_msg.poses.append(read_pose)
```
* rospkg를 통해 패키지 경로를 가져옴
* 지정된 파일(mando_path.txt)에서 Waypoint를 읽어와 PoseStamped로 변환
* 변환된 포즈들을 Path 메시지에 추가

### 2. 전역 경로 퍼블리시
```python
rate = rospy.Rate(20)  # 20Hz
while not rospy.is_shutdown():
    self.global_path_pub.publish(self.global_path_msg)
    rate.sleep()
```
* 20Hz 주기로 /global_path에 전역 경로를 퍼블리시
* 시뮬레이션 또는 실제 차량에서 전역 경로 참조 가능
</details>

## 2) local_path_pub.py - 로컬 경로 생성 및 퍼블리시 노드
|항목|설명|
|------|------|
|구독 토픽|/odom (nav_msgs/Odometry), /global_path (nav_msgs/Path)|
|퍼블리시 토픽|/local_path (nav_msgs/Path)|
|주요 기능 요약|차량 현재 위치 기반 최근접 Waypoint 탐색 및 로컬 경로 생성|

<details> <summary> <b> 📌 local_path 코드 분석 펼쳐보기 </b></summary>

### 1. 글로벌 경로 수신 및 저장
```python
def global_Path_callback(self, msg):
    self.global_path_msg = msg
```
* /global_path 토픽에서 경로 데이터를 수신해 저장.
* 추후 차량 현재 위치 기준으로 가까운 포인트를 찾기 위해 사용.

### 2. 차량 현재 위치 수신
```python
def odom_callback(self, msg):
    self.x = msg.pose.pose.position.x
    self.y = msg.pose.pose.position.y
    self.is_status = True
```
* /odom에서 차량의 현재 위치를 수신해 저장.
* 위치 정보가 수신되면, 로컬 경로 생성이 가능하도록 is_status 플래그를 True로 설정.

### 3. 최근접 웨이포인트 탐색
```python
min_dis = float('inf')
current_waypoint = -1
for i, waypoint in enumerate(self.global_path_msg.poses):
    distance = sqrt(pow(self.x - waypoint.pose.position.x, 2) + pow(self.y - waypoint.pose.position.y, 2))
    if distance < min_dis:
        min_dis = distance
        current_waypoint = i
```
* 현재 위치에서 글로벌 경로상의 모든 포인트와의 거리를 계산.
* 그 중 가장 가까운 포인트(current_waypoint)를 탐색.

### 4. 로컬 경로 생성
```python
if current_waypoint + self.local_path_size < len(self.global_path_msg.poses):
    for num in range(current_waypoint, current_waypoint + self.local_path_size):
        tmp_pose = PoseStamped()
        tmp_pose.pose.position = self.global_path_msg.poses[num].pose.position
        tmp_pose.pose.orientation.w = 1
        local_path_msg.poses.append(tmp_pose)
```
* 최근접 웨이포인트부터 50개 포인트를 잘라서 로컬 경로 구성.
* 남은 경로가 50개 미만이면 끝까지 모두 포함.

### 5. 로컬 경로 퍼블리시
```python
self.local_path_pub.publish(local_path_msg)
```
* 생성된 로컬 경로를 /local_path로 퍼블리시.
* Pure Pursuit 등 경로 추종 알고리즘에서 이 경로를 따라 주행하도록 사용.


📝 요약
* 글로벌 경로(Global Path)에서 차량 현재 위치를 기준으로 가장 가까운 지점을 찾는다.
* 최근접 위치부터 50개 포인트를 잘라 **로컬 경로(Local Path)**로 생성한다.
* 생성된 로컬 경로는 /local_path 토픽으로 주기적으로 퍼블리시되어, pure_pursuit, lattice_planner의 입력으로 사용된다.
</details>


# 3. 차량 인식 

## 1) yolov3.py - 카메라 이미지 기반 실시간 차량 탐지

|항목|설명|
|------|------|
|구독 토픽|`/image_jpeg/compressed` (CompressedImage)|
|퍼블리시 토픽|`/car_bounding_boxes` (Float32MultiArray)|
|주요 기능 요약|카메라 이미지에서 차량 객체를 실시간으로 탐지하고, 바운딩 박스를 퍼블리시하여 LiDAR와의 캘리브레이션 기반 거리 추정을 위한 전처리 데이터를 제공함.|

<details> <summary> <b>📌 yolov3.py 코드 분석 펼쳐보기 </b> </summary>
  
### 1. YOLOv3 네트워크 및 클래스 초기화
```python
self.net = cv2.dnn.readNet("yolov3.weights", "yolov3.cfg")
self.classes = open("coco.names").read().strip().split("\n")
```
 
- YOLOv3의 가중치와 구성 파일 로드

- coco.names에서 객체 클래스 목록 불러옴 (우리는 그중 "car"에 주목)

### 2. 카메라 이미지 수신 (callback)
```python
def callback(self, msg):
    np_arr = np.frombuffer(msg.data, np.uint8)
    self.latest_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
```
- /image_jpeg/compressed로 들어온 이미지를 OpenCV로 디코딩

- 최신 이미지 저장

### 3. YOLOv3 객체 탐지 수행
```python
blob = cv2.dnn.blobFromImage(img, 0.00392, (320, 320), (0, 0, 0), True, crop=False)
self.net.setInput(blob)
outputs = self.net.forward(self.output_layers)
```
- 이미지를 blob 형식으로 변환해 YOLO에 입력

- 네트워크 순전파(forward) 수행하여 결과 출력

### 4. 객체 탐지 및 바운딩 박스 필터링
```python
if confidence > 0.3 and label == "car":
    bbox_data = Float32MultiArray(data=[x, y, w, h])
    self.bbox_pub.publish(bbox_data)
```
- 신뢰도가 0.3 이상이고 클래스가 "car"일 때만 바운딩 박스를 퍼블리시

- 바운딩 박스 좌표 [x, y, w, h] 형식으로 /car_bounding_boxes 퍼블리시

### 5. 차량이 감지되지 않았을 경우
```python
if not car_detected:
    empty_bbox_data = Float32MultiArray(data=[0, 0, 0, 0])
    self.bbox_pub.publish(empty_bbox_data)
```
- 차량이 탐지되지 않으면 기본값 [0, 0, 0, 0] 전송 → 이후 거리 계산에서 무시되도록 처리
