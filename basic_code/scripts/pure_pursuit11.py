#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, os
import rospkg
from math import cos, sin, pi, sqrt, pow, atan2
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from morai_msgs.msg import CtrlCmd
from std_msgs.msg import Float32  # 거리 토픽 구독을 위해 추가
import numpy as np
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class pure_pursuit:
    def __init__(self):
        rospy.init_node('pure_pursuit', anonymous=True)
        rospy.Subscriber("local_path", Path, self.path_callback)
        rospy.Subscriber("odom", Odometry, self.odom_callback)
        rospy.Subscriber("/distance_to_car", Float32, self.dist_forward_callback)  # 앞차와의 거리 토픽 구독

        self.ctrl_cmd_pub = rospy.Publisher('ctrl_cmd', CtrlCmd, queue_size=1)
        self.ctrl_cmd_msg = CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType = 2

        self.is_path = False
        self.is_odom = False

        self.forward_point = Point()
        self.current_postion = Point()
        self.is_look_forward_point = False
        self.vehicle_length = 4.47
        self.lfd = 10

        # ACC 관련 변수 초기화
        self.obstacle_distance = float('inf')  # 초기 장애물 거리
        self.previous_obstacle_distance = float('inf')
        self.safe_distance = 20.0  # 안전 거리 (m)
        self.min_safe_distance = 7.0  # 최소 안전 거리
        self.target_velocity = 15.0  # 초기 목표 속도

        rate = rospy.Rate(15)  # 15hz
        while not rospy.is_shutdown():
            if self.is_path and self.is_odom:
                self.pure_pursuit_control()
                self.acc_control()  # ACC 제어 추가
                self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)

            
            self.is_path = self.is_odom = False
            rate.sleep()

    def pure_pursuit_control(self):
        vehicle_position = self.current_postion
        self.is_look_forward_point = False

        translation = [vehicle_position.x, vehicle_position.y]

        t = np.array([
            [cos(self.vehicle_yaw), -sin(self.vehicle_yaw), translation[0]],
            [sin(self.vehicle_yaw), cos(self.vehicle_yaw), translation[1]],
            [0, 0, 1]
        ])

        det_t = np.array([
            [t[0][0], t[1][0], -(t[0][0] * translation[0] + t[1][0] * translation[1])],
            [t[0][1], t[1][1], -(t[0][1] * translation[0] + t[1][1] * translation[1])],
            [0, 0, 1]
        ])

        for num, i in enumerate(self.path.poses):
            path_point = i.pose.position
            global_path_point = [path_point.x, path_point.y, 1]
            local_path_point = det_t.dot(global_path_point)
            if local_path_point[0] > 0:
                dis = sqrt(pow(local_path_point[0], 2) + pow(local_path_point[1], 2))
                if dis >= self.lfd:
                    self.forward_point = path_point
                    self.is_look_forward_point = True
                    break

        theta = atan2(local_path_point[1], local_path_point[0])

        if self.is_look_forward_point:
            self.ctrl_cmd_msg.steering = atan2((2 * self.vehicle_length * sin(theta)), self.lfd)

            os.system('clear')
            print("-------------------------------------")
            print(" steering (deg) = ", self.ctrl_cmd_msg.steering * 180 / 3.14)
            print(" velocity (kph) = ", self.ctrl_cmd_msg.velocity)
            print("-------------------------------------")
        else:
            print("no found forward point")
            self.ctrl_cmd_msg.steering = 0.0

    def acc_control(self):
        base_velocity = 15.0  # 기본 속도 (kph)
        
        # 거리 변화율 계산
        distance_change = self.obstacle_distance - self.previous_obstacle_distance

        # 거리 변화율에 따른 속도 조절
        if self.obstacle_distance < 7:
            # 앞차와 너무 가까우면 정지
            self.target_velocity = 0.0
            rospy.logwarn("Too close! Stopping.")
        elif self.obstacle_distance < 15:
            if distance_change < 0:
                # 앞차와의 거리가 줄어들고 있다면 속도 감소
                reduction_factor = self.obstacle_distance / self.safe_distance
                self.target_velocity = base_velocity * reduction_factor
                rospy.loginfo(f"Reducing speed: {self.target_velocity:.2f} km/h")
            elif distance_change > 0:
                # 앞차와의 거리가 늘어나고 있다면 속도 증가
                increase_factor = 1 + (distance_change / self.safe_distance)
                self.target_velocity = base_velocity * increase_factor
                rospy.loginfo(f"Increasing speed: {self.target_velocity:.2f} km/h")
            else:
                # 거리 변화가 거의 없으면 현재 속도를 유지
                rospy.loginfo(f"Maintaining speed: {self.target_velocity:.2f} km/h")
            
        else:
            # 안전 거리 이상이면 기본 속도로 설정
            self.target_velocity = base_velocity
            rospy.loginfo(f"Max speed: {self.target_velocity:.2f} km/h")
        
        # 현재 속도를 제어 메시지에 반영
        self.ctrl_cmd_msg.velocity = self.target_velocity


    def path_callback(self, msg):
        self.is_path = True
        self.path = msg

    def odom_callback(self, msg):
        self.is_odom = True
        odom_quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                           msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        _, _, self.vehicle_yaw = euler_from_quaternion(odom_quaternion)
        self.current_postion.x = msg.pose.pose.position.x
        self.current_postion.y = msg.pose.pose.position.y

    def dist_forward_callback(self, msg):
    # 이전 거리를 현재 거리로 저장
        self.previous_obstacle_distance = self.obstacle_distance
        
        # 새로운 거리 토픽에서 장애물(앞차)까지의 거리 정보 수신
        self.obstacle_distance = msg.data
        rospy.loginfo(f"Distance to car: {self.obstacle_distance:.2f} meters")

if __name__ == '__main__':
    try:
        test_track = pure_pursuit()
    except rospy.ROSInterruptException:
        pass