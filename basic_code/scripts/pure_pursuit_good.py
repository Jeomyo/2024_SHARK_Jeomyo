#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# 차량 종/횡 방향 제어에 대한 코드
# Purpusuit 알고리즘의 Look Ahead Distance(전방 주시 거리) 값을 속도에 비례하여 가변 값으로 만들어 횡 방향 주행 성능을 올린다.
# 횡방향 제어 입력은 주행할 Local Path(지역경로)와 Odometry(차량의 상태 정보)를 받아 차량을 제어
# 종방향 제어 입력은 목표 속도를 지정 한뒤 목표 속도에 도달하기 위한 Throttle control 을 합니다.
# 종방향 제어 입력은 longlCmdType 1(Throttle control) 이용합니다.

import os, sys
import time
import rospy
import rospkg
from math import cos, sin, pi, sqrt, pow, atan2
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus, EventInfo
from morai_msgs.srv import MoraiEventCmdSrv
from std_msgs.msg import Bool
import numpy as np
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from enum import Enum


# 노드 실행 순서 
# 0. 필수 학습 지식
# 1. subscriber, publisher 선언
# 2. 속도 비례 Look Ahead Distance 값 설정
# 3. 좌표 변환 행렬 생성
# 4. Steering 각도 계산
# 5. PID 제어 생성
# 6. 도로의 곡률 계산
# 7. 곡률 기반 속도 계획
# 8. 제어입력 메세지 Publish

class Gear(Enum):
    P = 1
    R = 2
    N = 3
    D = 4


#TODO: (0) 필수 학습 지식
'''
# Look Ahead Distance 값을 현재 속도에 비례하여 설정해서 최대/최소 값을 정함
# 주행 속도에 비례한 값으로 변경 한 뒤 "self.lfd_gain" 을 변경한다.
'''

class pure_pursuit :
    def __init__(self):
        rospy.init_node('pure_pursuit', anonymous=True)

        # arg = rospy.myargv(argv=sys.argv)
        # local_path_name = arg[1]

        # arg 값으로 받은 변수(local path)로 사용한다 - L61 대체
        # rospy.Subscriber(local_path_name, Path, self.path_callback)

        
        rospy.Subscriber("/global_path", Path, self.global_path_callback)
        rospy.Subscriber("/lattice_path", Path, self.path_callback)     # L58로 대체
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/Competition_topic", EgoVehicleStatus, self.status_callback)
        rospy.Subscriber("/dynamic_status", Bool, self.dynamic_callback)
        
        rospy.Subscriber("is_dynamic", Bool, self.is_dynamic_callback)
        rospy.Subscriber("/speed_control", Bool, self.speed_callback)  # Bool 값 구독

        rospy.Subscriber('/CollisionData', CollisionData, self.collision_callback)
        self.is_collision_data = False
        self.is_collision = False
        detect_collision = False

        rospy.Subscriber("/traffic_light_state", Bool, self.is_red_callback)
        self.is_red = False

        rospy.wait_for_service('/Service_MoraiEventCmd', timeout= 5)
        self.event_cmd_srv = rospy.ServiceProxy('Service_MoraiEventCmd', MoraiEventCmdSrv)

        self.ctrl_cmd_pub = rospy.Publisher("/ctrl_cmd", CtrlCmd, queue_size=1)

        # User -> Simul; 모터 동작 메시지
        self.ctrl_cmd_msg = CtrlCmd()
        self.event_cmd = EventInfo()
        
        self.ctrl_cmd_msg.longlCmdType = 1

        self.is_path = False
        self.is_odom = False
        self.is_status = False
        self.is_global_path = False

        self.is_look_forward_point = False
        self.stop = False
        self.is_dynamic = False
        self.speed_control = False 

        self.forward_point = Point()
        self.current_postion = Point()

        self.vehicle_length = 4.635
        # 전방 주시 거리(lfd, look forward distance)
        self.lfd = 10
        self.min_lfd = 10
        self.max_lfd = 30
        self.lfd_gain = 0.78
        self.target_velocity = 40

        # 초기 car_max_speed 설정
        self.car_max_speed = 40  # 기본값 40으로 설정

        self.pid = pidControl()
        self.vel_planning = velocityPlanning(self.car_max_speed / 3.6, 0.15)

        while True:
            if self.is_global_path == True:
                self.velocity_list = self.vel_planning.curvedBaseVelocity(self.global_path, 40)
                break
            else:
                rospy.loginfo("not")

        rate = rospy.Rate(50) # 30hz
        while not rospy.is_shutdown():

            if self.is_path and self.is_odom and self.is_status and self.is_dynamic==False:
               
                self.current_waypoint = self.get_current_waypoint(self.current_postion, self.global_path)

                # speed_bool에 따른 target_velocity 설정
                if self.speed_control:
                    self.car_max_speed = 25
                else:
                    self.car_max_speed = 40

                self.target_velocity = self.velocity_list[self.current_waypoint] * 3.6
                        
                #  횡방향 제어 - steering = theta = calc_pure_pursuit
                steering = self.calc_pure_pursuit()
                if self.is_look_forward_point:
                        self.ctrl_cmd_msg.steering = steering
                else : 
                    rospy.loginfo("no found forward point")
                    self.ctrl_cmd_msg.steering = 0.0

                # 종방향 제어 - pid(타겟 속도, 현재 속도)
                output = self.pid.pid(self.target_velocity, self.status_msg.velocity.x * 3.6)

                if output > 0.0:
                    self.ctrl_cmd_msg.accel = output
                    self.ctrl_cmd_msg.brake = 0.0
                else:
                    self.ctrl_cmd_msg.accel = 0.0
                    self.ctrl_cmd_msg.brake = -output

                #TODO: (8) 제어입력 메세지 Publish
                
                # 제어입력 메세지 를 전송하는 publisher 를 만든다.
                self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)

                os.system('clear')
                print("-------------------------------------")
                print(" steering (deg) = ", self.ctrl_cmd_msg.steering * 180/pi)
                print(" velocity (kph) = ", output)
                print("-------------------------------------") 
                
            rate.sleep()


    def path_callback(self,msg):
        self.is_path=True
        self.path=msg  

    def odom_callback(self, msg):
        self.is_odom=True
        odom_quaternion=(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
        _,_,self.vehicle_yaw=euler_from_quaternion(odom_quaternion)
        self.current_postion.x=msg.pose.pose.position.x
        self.current_postion.y=msg.pose.pose.position.y

    def status_callback(self, msg): ## Vehicl Status Subscriber 
        self.is_status=True
        self.status_msg=msg

    def global_path_callback(self, msg):
        self.is_global_path = True
        self.global_path = msg

    def speed_callback(self, msg):
        self.speed_bool = msg.data

    def dynamic_callback(self, msg):
        self.dynamic = msg.data  

    def send_gear_cmd(self, gear_mode):  #
        gear_cmd = EventInfo()
        gear_cmd.option = 3
        gear_cmd.ctrl_mode = 3
        gear_cmd.gear = gear_mode
        gear_cmd_resp = self.event_cmd_srv(gear_cmd)
        rospy.loginfo(f"Gear changed to {gear_cmd}")  
    
    def get_current_waypoint(self, current_position, global_path):
        min_dist = float('inf')        
        currnet_waypoint = -1
        for i,pose in enumerate(global_path.poses):
            dx = current_position.x - pose.pose.position.x
            dy = current_position.y - pose.pose.position.y

            dist = sqrt(pow(dx,2) + pow(dy,2))
            if min_dist > dist:
                min_dist = dist
                currnet_waypoint = i
        return currnet_waypoint

    def calc_pure_pursuit(self, ):

        self.lfd = self.lfd_gain * min(self.max_lfd, max(self.min_lfd, self.status_msg.velocity.x))
        
        vehicle_position = self.current_postion
        self.is_look_forward_point = False

        # translation - 변환 좌표
        translation = [vehicle_position.x, vehicle_position.y]


        trans_matrix = np.array([[cos(self.vehicle_yaw), -sin(self.vehicle_yaw), 0],
                                 [sin(self.vehicle_yaw),  cos(self.vehicle_yaw), 0],
                                 [0                    , 0                     , 1]])

        det_trans_matrix = np.linalg.inv(trans_matrix)

        for idx, pose in enumerate(self.path.poses):
            path_point = pose.pose.position
            
            global_path_point = [path_point.x - translation[0], path_point.y - translation[1], 1]
            local_path_point = det_trans_matrix.dot(global_path_point)    

            # 후진하지 않기 위해서 - local_path_point 는 무조건 양수
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
    
    def is_dynamic_callback(self, msg) :
        self.is_dynamic = msg.data
        print("is_dynamic = ", self.is_dynamic)    
        print("is_dynamic = ", self.is_dynamic)    
        print("is_dynamic = ", self.is_dynamic)    
        print("is_dynamic = ", self.is_dynamic)    
        print("is_dynamic = ", self.is_dynamic) 
        self.ctrl_cmd_msg.brake=1.0
        self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)


class pidControl:
    def __init__(self):
        self.p_gain = 0.3
        self.i_gain = 0.00
        self.d_gain = 0.03
        self.prev_error = 0
        self.i_control = 0
        self.controlTime = 0.02

    def pid(self, target_vel, current_vel):
        #TODO: (5) PID 제어 생성 - 종방향 제어

        # 현재와 목표 속도 갭
        error = target_vel - current_vel

        p_control = self.p_gain * error         # p 제어
        self.i_control += error * self.controlTime    # 누적 속도(i) 차이 - 에러 값들의 총합
        d_control = self.d_gain * (error - self.prev_error) / self.controlTime  # 미분(d) 값

        # PID 제어 반영
        output = p_control + self.i_gain*self.i_control + d_control
        self.prev_error = error

        return output

class velocityPlanning:
    def __init__ (self, car_max_speed, road_friciton):
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

            #TODO: (6) 도로의 곡률 계산 - 최소 자승법 이용하여 곡률 반지름 "r" 계산
            #========================================================#
            A = np.array(x_list)
            B = np.array(y_list)
            a, b, c = np.dot(np.linalg.pinv(A), B)  

            r = (a**2 + b**2 - c)**0.5          
            # 최소 자승법 안정성 증가 - 아래 코드    


            #TODO: (7) 곡률 기반 속도 계획
            '''
            # 계산 한 곡률 반경을 이용하여 최고 속도를 계산합니다.
            # 평평한 도로인 경우 최대 속도를 계산합니다. 
            # 곡률 반경 x 중력가속도 x 도로의 마찰 계수 계산 값의 제곱근이 됩니다.
            v_max = 

            '''
            v_max = sqrt(r * 9.8 * self.road_friction)

            if v_max > self.car_max_speed:
                v_max = self.car_max_speed
            out_vel_plan.append(v_max)

        for i in range(len(gloabl_path.poses) - point_num, len(gloabl_path.poses) - 10):
            out_vel_plan.append(30)

        for i in range(len(gloabl_path.poses) - 10, len(gloabl_path.poses)):
            out_vel_plan.append(0)

        return out_vel_plan

if __name__ == '__main__':
    try:
        test_track=pure_pursuit()
    except rospy.ROSInterruptException:
        pass
