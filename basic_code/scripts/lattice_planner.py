#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from math import cos, sin, sqrt, atan2
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry  # Odometry 추가
from morai_msgs.msg import EgoVehicleStatus
from std_msgs.msg import Float32MultiArray, Bool
import numpy as np

class latticePlanner:
    def __init__(self):
        rospy.init_node('lattice_planner', anonymous=True)

        # object_pointcloud_data 데이터를 받을 토픽을 명시
        rospy.Subscriber("/object_pointcloud_data", PointCloud, self.object_callback)

        # Local Path와 Vehicle Status 데이터를 수신할 Subscriber 설정
        rospy.Subscriber("/local_path", Path, self.path_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)  # Odom에서 위치 정보 수신
        rospy.Subscriber("/Competition_topic", EgoVehicleStatus, self.status_callback)  # 속도만 받음
        rospy.Subscriber("/right_path", Path, self.right_path_callback)  # 우측 경로 받아옴
        rospy.Subscriber("/cluster_distances", Float32MultiArray, self.distance_callback)


        self.lattice_path_pub = rospy.Publisher("/lattice_path", Path, queue_size=1)
        self.speed_pub = rospy.Publisher("/speed_control", Bool, queue_size=1)

        self.is_path = False
        self.is_status = False
        self.is_odom = False  # odom flag 추가
        self.is_obj = False
        self.is_right_path = False
        self.local_path = None
        self.object_data = None
        self.lane_pos = None
        self.right_path = None  # 우측 경로 Path 데이터
        self.is_distance = False  # 거리 데이터 수신 여부 플래그
        self.distances = None  # 거리 데이터를 저장할 변수
        

        rate = rospy.Rate(50)  # 30hz
        while not rospy.is_shutdown():
            if self.is_path and self.is_status and self.is_odom and self.is_right_path:
                # 객체가 없어도 우측 경로 충돌 여부 확인
                is_crash, right_crash = self.check_collision(self.local_path, self.object_data if self.is_obj else None)
                if is_crash or right_crash:
                    lattice_path = self.latticePlanner(self.local_path)
                    lattice_path_index = self.collision_check(self.object_data, lattice_path, right_crash)
                    print(lattice_path_index)
                    self.lattice_path_pub.publish(lattice_path[lattice_path_index])
                    self.publish_speed_bool(True)
                else:
                    self.lattice_path_pub.publish(self.local_path)
                    self.publish_speed_bool(False)
            rate.sleep()

    def publish_speed_bool(self, bool_value):
        speed_bool_msg = Bool()
        speed_bool_msg.data = bool_value
        self.speed_pub.publish(speed_bool_msg)  # speed_bool 퍼블리시

    def object_callback(self, msg):
        self.is_obj = True
        self.object_data = msg  # PointCloud 데이터를 저장

    def check_collision(self, ref_path, object_data):
        is_crash = False
        right_crash = False  # 우측 경계 충돌 여부를 따로 관리
        # 차량 위치 정보 가져오기
        vehicle_x = self.vehicle_pos_x
        vehicle_y = self.vehicle_pos_y

        # 객체와의 충돌 검사
        for point in object_data.points:
            for path in ref_path.poses:
                dis = sqrt((path.pose.position.x - point.x)**2 + (path.pose.position.y - point.y)**2)
                if dis < 2.7:  # 충돌 판단 거리 설정
                    is_crash = True
                    break
            if is_crash:  # 충돌이 발생하면 더 이상 확인할 필요가 없으므로 break
                break

        # 거리 값 리스트에서 충돌 여부 판단 (distance_callback을 통해 수신된 거리 값 사용)
        if self.is_distance and self.distances is not None:
            for i, diss in enumerate(self.distances):
                if diss < 4.5:  # 충돌 판단 거리 설정
                    is_crash = True
                    print(f"충돌 위험! 객체와의 거리: {diss}")
                    break

        filtered_right_path, lane_position = self.filter_path_by_distance(self.right_path, 30)

        # 우측 경계와의 충돌 검사
        for path in filtered_right_path.poses:
            dis_right = sqrt((vehicle_x - path.pose.position.x)**2 +
                            (vehicle_y - path.pose.position.y)**2)
            if dis_right < 1:  # 충돌 판단 거리
                right_crash = True
                print("right crash!")
                break

        return is_crash, right_crash
    
    def rectangle(self, x, y, rect):
        def cross_product(x1, y1, x2, y2):
            return x1 * y2 - y1 * x2

        point = np.array([x, y])

        for i in range(4):
            p1 = np.array(rect[i])
            p2 = np.array(rect[(i+1) % 4])

            edge = p2 - p1
            vec_to_point = point - p1

            if cross_product(edge[0], edge[1], vec_to_point[0], vec_to_point[1]) < 0:
                return False
        return True
    
    def filter_path_by_distance(self, path, max_distance):
        """ 주어진 path에서 차량과 일정 거리 이내의 경로만 반환 """
        filtered_path = Path()
        filtered_path.header = path.header

        min_distance = float('inf')
        current_waypoint = -1

        for i, pose in enumerate(path.poses):
            dis = sqrt((pose.pose.position.x - self.vehicle_pos_x)**2 +
                    (pose.pose.position.y - self.vehicle_pos_y)**2)
            if dis < min_distance:
                min_distance = dis
            
            if dis <= max_distance:
                filtered_path.poses.append(pose)

            if len(filtered_path.poses) > 0 and dis > max_distance:
                break  # 이미 거리를 넘는 경우 이후는 필요 없음

        # 현재 waypoint와 최소 거리를 기준으로 임계값 설정
        threshold = 4.5  # 이 값은 테스트 후 조정 필요
        if min_distance < threshold:
            lane_position = 1  # 1차선
        else:
            lane_position = 0  # 다른 차선

        return filtered_path, lane_position

    def collision_check(self, object_data, out_path, right_crash):
        selected_lane = -1
        lane_weight = [12, 11, 10, 9, 8, 7, 6, 0, 1, 2, 3, 4, 5, 6, 1000]
        min_r = self.calculate_curvature(self.local_path, 10)

        # 사각형의 좌표를 정의 (주어진 네 좌표)
        rect = [(690,-137), (693,-129), (530,-28), (524,-35)]
        
        # 객체와의 충돌 여부 체크 및 가중치 부여
        for point in object_data.points:
            for path_num in range(len(out_path)):
                for path_pos in out_path[path_num].poses:
                    dis = sqrt(pow(point.x - path_pos.pose.position.x, 2) + pow(point.y - path_pos.pose.position.y, 2))
                    if dis < 1.45:
                        lane_weight[path_num] += 100

        # 필터링된 우측 경로를 사용하여 충돌 검사 및 가중치 계산
        filtered_right_path, lane_position = self.filter_path_by_distance(self.right_path, 30)

        # 우측 경로가 짧으면 종료
        if len(filtered_right_path.poses) < 2:
            print("Right path too short for comparison.")
            return selected_lane
        
        # 우측 경로에서 시작점과 중간점 추출
        right_start = filtered_right_path.poses[0].pose.position
        mid_index = len(filtered_right_path.poses) // 2
        right_mid = filtered_right_path.poses[mid_index].pose.position

        # lane_position이 1일 때만 우측 경로를 계산
        if lane_position == 1:

            # 경로 이탈 체크
            for path_num in range(len(out_path)):
                min_right_dist = float('inf')  # min_right_dist를 무한대로 초기화하여 최소 거리를 찾을 수 있도록 설정
                path_positions = np.array([[pose.pose.position.x, pose.pose.position.y] for pose in out_path[path_num].poses])
                right_positions = np.array([[rp.pose.position.x, rp.pose.position.y] for rp in filtered_right_path.poses])

                # 거리 계산 (벡터 연산)
                for path_pos in path_positions:
                    distances = np.sqrt(np.sum((right_positions - path_pos) ** 2, axis=1))
                    min_right_dist = np.min(distances)
                    
                    if min_right_dist < 0.7:
                        lane_weight[path_num] += 100


            # 각 회피 경로의 포즈마다 외적을 계산하고 가중치 부여
            if min_r is not None and min_r < 100:
                # min_r < 100일 때 path_num이 0~7인 경우 외적 계산
                for path_num in range(len(out_path)):
                    for path_pos in out_path[path_num].poses:
                        path_point = path_pos.pose.position

                        # 벡터 계산: 시작점 -> 중간점 (우측 경로), 시작점 -> 현재 경로 위치 (회피 경로)
                        right_vec = np.array([right_mid.x - right_start.x, right_mid.y - right_start.y])
                        path_vec = np.array([path_point.x - right_start.x, path_point.y - right_start.y])

                        # 외적 값 계산
                        cross_product = np.cross(np.array([right_vec[0], right_vec[1], 0]), np.array([path_vec[0], path_vec[1], 0]))[2]

                        if cross_product < 0:  # 외적 값이 음수면 경로가 우측에 있으므로 가중치 추가
                            lane_weight[path_num] += 100
            else:
                # min_r >= 100일 때 path_num이 0~4인 경우 외적 계산
                for path_num in range(5):  # 0~4
                    for path_pos in out_path[path_num].poses:
                        path_point = path_pos.pose.position

                        # 벡터 계산: 시작점 -> 중간점 (우측 경로), 시작점 -> 현재 경로 위치 (회피 경로)
                        right_vec = np.array([right_mid.x - right_start.x, right_mid.y - right_start.y])
                        path_vec = np.array([path_point.x - right_start.x, path_point.y - right_start.y])

                        # 외적 값 계산
                        cross_product = np.cross(np.array([right_vec[0], right_vec[1], 0]), np.array([path_vec[0], path_vec[1], 0]))[2]

                        if cross_product < 0:  # 외적 값이 음수면 경로가 우측에 있으므로 가중치 추가
                            lane_weight[path_num] += 100
        else:
            # min_r >= 100일 때 path_num이 0~4인 경우 외적 계산
                for path_num in range(2):  
                    for path_pos in out_path[path_num].poses:
                        path_point = path_pos.pose.position

                        # 벡터 계산: 시작점 -> 중간점 (우측 경로), 시작점 -> 현재 경로 위치 (회피 경로)
                        right_vec = np.array([right_mid.x - right_start.x, right_mid.y - right_start.y])
                        path_vec = np.array([path_point.x - right_start.x, path_point.y - right_start.y])

                        # 외적 값 계산
                        cross_product = np.cross(np.array([right_vec[0], right_vec[1], 0]), np.array([path_vec[0], path_vec[1], 0]))[2]

                        if cross_product < 0:  # 외적 값이 음수면 경로가 우측에 있으므로 가중치 추가
                            lane_weight[path_num] += 100

                for path_num in [13 ,14]:
                    # 회피 경로 포인트가 사각형 내부에 있을 때 가중치 추가
                    for path_pos in out_path[path_num].poses:
                        path_point = path_pos.pose.position
                        if self.rectangle(path_point.x, path_point.y, rect):
                            print("sex1")
                            lane_weight[path_num] += 100  # 사각형 내부에 있을 때 추가 가중치
                



        if right_crash:
            for i in range(6):
                lane_weight[i] += 10000

        for i in range(len(out_path)):
            print(f"{i} : {lane_weight[i]}")

        selected_lane = lane_weight.index(min(lane_weight))
        return selected_lane
    
    def distance_callback(self, msg):
        self.is_distance = True
        self.distances = msg.data  # 거리 데이터를 Float32MultiArray로 저장


    def calculate_curvature(self, local_path, point_num):
        r_values = []  # r 값을 저장할 리스트
        
        if len(local_path.poses) > point_num * 2:
            for i in range(point_num, len(local_path.poses) - point_num):
                x_list = []
                y_list = []
                for box in range(-point_num, point_num):
                    x = local_path.poses[i + box].pose.position.x
                    y = local_path.poses[i + box].pose.position.y
                    x_list.append([-2 * x, -2 * y, 1])
                    y_list.append((-x * x) - (y * y))
                
                # 최소 자승법을 이용한 곡률 반지름 계산
                A = np.array(x_list)
                B = np.array(y_list)
                a, b, c = np.dot(np.linalg.pinv(A), B)
                r = (a ** 2 + b ** 2 - c) ** 0.5
                r_values.append(r)  # r 값을 리스트에 추가
        
        # r 값 중 최솟값을 구함
        if r_values:
            min_r = r_values[0]
            return min_r
        
        return None

    def path_callback(self, msg):
        self.is_path = True
        self.local_path = msg

    def status_callback(self, msg):  # 속도만 처리
        self.is_status = True
        self.vehicle_velocity = msg.velocity.x * 3.6  # 속도를 km/h로 변환

    def odom_callback(self, msg):  # 차량 위치만 처리
        self.is_odom = True
        self.vehicle_pos_x = msg.pose.pose.position.x
        self.vehicle_pos_y = msg.pose.pose.position.y

    def right_path_callback(self, msg):
        self.right_path = msg
        self.is_right_path = True

    def latticePlanner(self, ref_path):
        out_path = []
        vehicle_pose_x = self.vehicle_pos_x  # Odometry에서 받은 차량 위치
        vehicle_pose_y = self.vehicle_pos_y  # Odometry에서 받은 차량 위치
        vehicle_velocity = self.vehicle_velocity  # EgoVehicleStatus에서 받은 차량 속도

        look_distance = int(vehicle_velocity * 0.2 * 2)
        
        if look_distance < 20:  # 최소 20m 설정
            look_distance = 20

        if len(ref_path.poses) > look_distance:
            # 좌표 변환 행렬 생성
            global_ref_start_point = (ref_path.poses[0].pose.position.x, ref_path.poses[0].pose.position.y)
            global_ref_start_next_point = (ref_path.poses[1].pose.position.x, ref_path.poses[1].pose.position.y)

            global_ref_end_point = (ref_path.poses[look_distance*2].pose.position.x, ref_path.poses[look_distance*2].pose.position.y)
            
            theta = atan2(global_ref_start_next_point[1] - global_ref_start_point[1], global_ref_start_next_point[0] - global_ref_start_point[0])
            translation = [global_ref_start_point[0], global_ref_start_point[1]]

            # 회전 변환
            trans_matrix = np.array([
                [cos(theta), -sin(theta), translation[0]], 
                [sin(theta), cos(theta), translation[1]], 
                [0, 0, 1]
            ])

            # 역 변환
            det_trans_matrix = np.array([
                [trans_matrix[0][0], trans_matrix[1][0], -(trans_matrix[0][0] * translation[0] + trans_matrix[1][0] * translation[1])], 
                [trans_matrix[0][1], trans_matrix[1][1], -(trans_matrix[0][1] * translation[0] + trans_matrix[1][1] * translation[1])],
                [0, 0, 1]
            ])

            world_end_point = np.array([[global_ref_end_point[0]], [global_ref_end_point[1]], [1]])
            local_end_point = det_trans_matrix.dot(world_end_point)
            world_ego_vehicle_position = np.array([[vehicle_pose_x], [vehicle_pose_y], [1]])
            local_ego_vehicle_position = det_trans_matrix.dot(world_ego_vehicle_position)   
            lane_off_set = [-10, -8 ,-6, -5, -4, -3, -2, 0, 2, 3, 4, 5, 8, 10, 15]
            local_lattice_points = []
            
            for i in range(len(lane_off_set)):
                local_lattice_points.append([local_end_point[0][0], local_end_point[1][0] + lane_off_set[i], 1])
            
            # Lattice 경로 생성
            for end_point in local_lattice_points:
                lattice_path = Path()
                lattice_path.header.frame_id = '/map'

                waypoints_x = []
                waypoints_y = []

                x_interval = 0.5        # 생성할 Path 의 Point 간격
                x_start = 0
                x_end = end_point[0]
                y_start = local_ego_vehicle_position[1][0]
                y_end = end_point[1]

                x_num = x_end / x_interval
                for i in range(x_start, int(x_num)):
                    waypoints_x.append(i * x_interval)

                d = y_start
                c = 0
                b = 3 * (y_end - y_start) / x_end**2
                a = -2 * (y_end - y_start) / x_end**3

                for x in waypoints_x:
                    result = a * x**3 + b * x**2 + c * x + d
                    waypoints_y.append(result)

                # Local -> Global 변환 후 lattice path에 추가
                for i in range(len(waypoints_y)):
                    local_result = np.array([[waypoints_x[i]], [waypoints_y[i]], [1]])
                    global_result = trans_matrix.dot(local_result)

                    read_pose = PoseStamped()
                    read_pose.pose.position.x = global_result[0][0]
                    read_pose.pose.position.y = global_result[1][0]
                    read_pose.pose.position.z = 0.0
                    read_pose.pose.orientation.x = 0
                    read_pose.pose.orientation.y = 0
                    read_pose.pose.orientation.z = 0
                    read_pose.pose.orientation.w = 1

                    lattice_path.poses.append(read_pose)

                out_path.append(lattice_path)

            # 경로 Publish
            for i in range(len(out_path)):
                globals()['lattice_pub_{}'.format(i + 1)] = rospy.Publisher('/lattice_path_{}'.format(i + 1), Path, queue_size=1)
                globals()['lattice_pub_{}'.format(i + 1)].publish(out_path[i])

        return out_path


if __name__ == '__main__':
    try:
        latticePlanner()
    except rospy.ROSInterruptException:
        pass