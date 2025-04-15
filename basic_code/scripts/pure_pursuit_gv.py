#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import time
import rospy
from math import cos, sin, sqrt, pow, atan2
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus
import numpy as np
from tf.transformations import euler_from_quaternion

class pure_pursuit:
    def __init__(self):
        rospy.init_node('pure_pursuit', anonymous=True)

        # launch 파일로부터 local_path_name 인자를 받아옵니다.
        arg = rospy.myargv(argv=sys.argv)
        local_path_name = arg[1]

        # 로컬 경로 구독
        rospy.Subscriber(local_path_name, Path, self.path_callback)

        # 기타 필요한 데이터 구독
        rospy.Subscriber("/global_path", Path, self.global_path_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.status_callback)

        # 제어 명령 퍼블리셔
        self.ctrl_cmd_pub = rospy.Publisher("/ctrl_cmd", CtrlCmd, queue_size=1)

        self.ctrl_cmd_msg = CtrlCmd()
        
        # longlCmdType = 2: 목표 속도를 통한 제어
        self.ctrl_cmd_msg.longlCmdType = 2

        self.is_path = False
        self.is_odom = False
        self.is_status = False
        self.is_global_path = False

        self.is_look_forward_point = False

        self.forward_point = Point()
        self.current_position = Point()

        self.vehicle_length = 4.635
        self.lfd = 10
        self.min_lfd = 10
        self.max_lfd = 30
        self.lfd_gain = 0.78
        self.target_velocity = 15  # 목표 속도 (km/h)


        self.vel_planning = velocityPlanning(self.target_velocity / 3.6, 0.15)

        while True:
            if self.is_global_path:
                self.velocity_list = self.vel_planning.curvedBaseVelocity(self.global_path, 15)
                break
            else:
                rospy.loginfo('Waiting for global path data')

        rate = rospy.Rate(15)  # 15hz
        while not rospy.is_shutdown():
            if self.is_path and self.is_odom and self.is_status:
                self.current_waypoint = self.get_current_waypoint(self.status_msg, self.global_path)
                self.target_velocity = self.velocity_list[self.current_waypoint] * 3.6

                # 횡방향 제어: 조향각 계산
                steering = self.calc_pure_pursuit()
                if self.is_look_forward_point:
                    self.ctrl_cmd_msg.steering = steering
                else:
                    rospy.loginfo("No forward point found")
                    self.ctrl_cmd_msg.steering = 0.0

                # 종방향 제어: 목표 속도를 통한 제어
                self.ctrl_cmd_msg.velocity = self.target_velocity

                # 제어 명령 퍼블리시
                self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
                
            rate.sleep()

    def path_callback(self, msg):
        self.is_path = True
        self.path = msg  

    def odom_callback(self, msg):
        self.is_odom = True
        odom_quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        _, _, self.vehicle_yaw = euler_from_quaternion(odom_quaternion)
        self.current_position.x = msg.pose.pose.position.x
        self.current_position.y = msg.pose.pose.position.y

    def status_callback(self, msg):  # Vehicle Status Subscriber
        self.is_status = True
        self.status_msg = msg    
        
    def global_path_callback(self, msg):
        self.global_path = msg
        self.is_global_path = True
    
    def get_current_waypoint(self, ego_status, global_path):
        min_dist = float('inf')        
        current_waypoint = -1
        for i, pose in enumerate(global_path.poses):
            dx = ego_status.position.x - pose.pose.position.x
            dy = ego_status.position.y - pose.pose.position.y

            dist = sqrt(pow(dx, 2) + pow(dy, 2))
            if min_dist > dist:
                min_dist = dist
                current_waypoint = i
        return current_waypoint

    def calc_pure_pursuit(self):
        # 차량 속도에 비례한 전방주시거리 설정
        self.lfd = self.lfd_gain * min(self.max_lfd, max(self.min_lfd, self.status_msg.velocity.x))
        rospy.loginfo(self.lfd)
        
        vehicle_position = self.current_position
        self.is_look_forward_point = False

        # 좌표 변환 행렬 생성
        trans_matrix = np.array([[cos(self.vehicle_yaw), -sin(self.vehicle_yaw), 0],
                                 [sin(self.vehicle_yaw),  cos(self.vehicle_yaw), 0],
                                 [0, 0, 1]])

        det_trans_matrix = np.linalg.inv(trans_matrix)

        for idx, pose in enumerate(self.path.poses):
            path_point = pose.pose.position
            
            global_path_point = [path_point.x - vehicle_position.x, path_point.y - vehicle_position.y, 1]
            local_path_point = det_trans_matrix.dot(global_path_point)

            if local_path_point[0] > 0:
                dis = (local_path_point[0]**2 + local_path_point[1]**2)**0.5

                if dis >= self.lfd:
                    self.forward_point = local_path_point
                    self.is_look_forward_point = True
                    break
        
        try:
            theta = atan2(self.forward_point[1], self.forward_point[0])
        except:
            theta = 0
            
        steering = atan2(2 * self.vehicle_length * sin(theta), self.lfd)

        return steering

class velocityPlanning:
    def __init__ (self,car_max_speed, road_friciton):
        self.car_max_speed = car_max_speed
        self.road_friction = road_friciton

    def curvedBaseVelocity(self, gloabl_path, point_num):
        out_vel_plan = []

        for i in range(0,point_num):
            out_vel_plan.append(self.car_max_speed)

        for i in range(point_num, len(gloabl_path.poses) - point_num):
            x_list = []
            y_list = []
            for box in range(-point_num, point_num):
                x = gloabl_path.poses[i+box].pose.position.x
                y = gloabl_path.poses[i+box].pose.position.y
                x_list.append([-2*x, -2*y ,1])
                y_list.append((-x*x) - (y*y))

            #TODO: (6) 도로의 곡률 계산
            
            # 도로의 곡률 반경을 계산하기 위한 수식입니다.
            # Path 데이터의 좌표를 이용해서 곡선의 곡률을 구하기 위한 수식을 작성합니다.
            # 원의 좌표를 구하는 행렬 계산식, 최소 자승법을 이용하는 방식 등 곡률 반지름을 구하기 위한 식을 적용 합니다.
            # 적용한 수식을 통해 곡률 반지름 "r" 을 계산합니다.

            A = np.array(x_list)
            B = np.array(y_list)
            a, b, c = np.dot(np.linalg.pinv(A), B)
            
            r = (a**2 + b**2 - c)**0.5


            #TODO: (7) 곡률 기반 속도 계획
            
            # 계산 한 곡률 반경을 이용하여 최고 속도를 계산합니다.
            # 평평한 도로인 경우 최대 속도를 계산합니다. 
            # 곡률 반경 x 중력가속도 x 도로의 마찰 계수 계산 값의 제곱근이 됩니다.

            v_max = (r * 9.8 * 0.8) ** 0.5


            if v_max > self.car_max_speed:
                v_max = self.car_max_speed
            out_vel_plan.append(v_max)

        for i in range(len(gloabl_path.poses) - point_num, len(gloabl_path.poses)-10):
            out_vel_plan.append(30)

        for i in range(len(gloabl_path.poses) - 10, len(gloabl_path.poses)):
            out_vel_plan.append(0)

        return out_vel_plan

if __name__ == '__main__':
    try:
        pure_pursuit()
    except rospy.ROSInterruptException:
        pass
