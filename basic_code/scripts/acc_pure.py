#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, os, time
import numpy as np
from math import cos, sin, pi, sqrt, pow, atan2
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry, Path
from morai_msgs.msg import CtrlCmd, CollisionData, EventInfo
from morai_msgs.srv import MoraiEventCmdSrv
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32, Float32MultiArray, Bool
import sensor_msgs.point_cloud2 as pc2
import time #for parking
from enum import Enum
from tf.transformations import euler_from_quaternion
   
class pure_pursuit:
    def __init__(self):
        rospy.init_node('pure_pursuit', anonymous=True)

        # Subscribers
        rospy.Subscriber("/local_path", Path, self.path_callback)
        rospy.Subscriber("/global_path", Path, self.global_path_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/distance_to_car", Float32, self.dist_forward_callback)
        rospy.Subscriber("dist_l", Float32, lambda msg: self.dist_callback(msg, "l"))
        rospy.Subscriber("dist_r", Float32, lambda msg: self.dist_callback(msg, "r"))
        rospy.Subscriber("dist_f", Float32, lambda msg: self.dist_callback(msg, "f"))
       
        rospy.Subscriber("dist_left", Float32, lambda msg: self.dist2_callback(msg, "left"))
        rospy.Subscriber("dist_right", Float32, lambda msg: self.dist2_callback(msg, "right"))
        rospy.Subscriber("dist_front", Float32, lambda msg: self.dist2_callback(msg, "front"))
       
        # Subscribers for shadow_drive
        # Subscribers for unexpected obstacle
        rospy.Subscriber("/is_unexpected_obstacle", Bool, self.is_unexpected_obstacle_callback)
       
        # Service
        rospy.wait_for_service('/Service_MoraiEventCmd', timeout= 5)
        self.event_cmd_srv = rospy.ServiceProxy('Service_MoraiEventCmd', MoraiEventCmdSrv)

        # Publisher for Control Command
        self.ctrl_cmd_pub = rospy.Publisher("/ctrl_cmd", CtrlCmd, queue_size=1)

        # Initialize Control Command Message
        self.ctrl_cmd_msg = CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType = 2  # Velocity control mode

        self.is_path = False
        self.is_odom = False
        self.is_global_path = False
       
        self.is_look_forward_point = False
        self.forward_point = Point()
        self.current_position = Point()

        # ACC parameters
        self.obstacle_distance = float('inf')  # 초기 장애물(앞차) 거리
        self.safe_distance = 15.0  # 안전 거리 (m)
        self.min_safe_distance = 7.0  # 최소 안전 거리
       
        self.vehicle_length = 4.47
        self.lfd = 10
        self.min_lfd = 10
        self.max_lfd = 30
        self.lfd_gain = 0.78
        self.target_velocity = 12  # 초기 목표 속도 (km/h)
       
        self.dist_r = None  # 오른쪽 장애물과의 거리
        self.dist_l = None   # 왼쪽 장애물과의 거리
        self.dist_f = None  # 정면 장애물과의 거리
        self.dist_right = None  # 오른쪽 장애물과의 거리
        self.dist_left = None   # 왼쪽 장애물과의 거리
        self.dist_front = None  # 정면 장애물과의 거리
        self.is_obstacle = False  # 장애물 감지 여부
        self.stopped = False  # 차량 정지 여부
       
        #for static obstacle
        self.is_static_obstacle = False
       
        # Velocity Planning
        self.vel_planning = velocityPlanning(self.target_velocity / 3.6, 0.15)  # 초기 속도 12 km/h를 m/s로 변환
       
        # Initialize for shadow drive
        self.is_shadow_drive = False
       
        # Initialize for unexpected obstacle
        self.unexpected_obstacle =False
        self.is_car_moving = True

        while True:
            if self.is_global_path:
                self.velocity_list = self.vel_planning.curvedBaseVelocity(self.global_path, 20)
                break
            else:
                rospy.loginfo('Waiting for global path data')

        rate = rospy.Rate(10)  # 10hz
       
        while not rospy.is_shutdown():
            if self.unexpected_obstacle ==False :
                self.check_gps_and_obstacle()
                if self.is_shadow_drive == False :
                    if self.is_static_obstacle == False :
                        self.current_waypoint = self.get_current_waypoint(self.current_position, self.global_path)
                        self.target_velocity = self.velocity_list[self.current_waypoint] * 3.6  # 목표 속도 설정    

                        # ACC 알고리즘: 앞차와의 거리 기반 속도 조절
                        self.target_velocity = self.acc_control()

                        # 횡방향 제어 (Steering)
                        steering = self.calc_pure_pursuit()

                        if self.is_look_forward_point:
                            self.ctrl_cmd_msg.steering = steering
                        else:
                            rospy.loginfo("no forward point found")
                            self.ctrl_cmd_msg.steering = 0.0

                        # 종방향 제어 (목표 속도 설정)
                        self.ctrl_cmd_msg.velocity = self.target_velocity
   
                        # 제어 입력 메시지 Publish
                        self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
   
                        os.system('clear')
                        print("-------------------------------------")
                        print(" steering (deg) = ", self.ctrl_cmd_msg.steering * 180/pi)
                        print(" velocity (kph) = ", self.ctrl_cmd_msg.velocity)
                        print("-------------------------------------")  
                    elif self.is_static_obstacle == True:
                        if self.is_look_forward_point:
                            if self.dist_front < 3.0:  # 미리 회피하기 위해 거리 조정 (3.0m로 설정)
                                if self.dist_left < self.dist_right:
                                    # 왼쪽으로 회피
                                    self.ctrl_cmd_msg.steering = -0.7  # 오른쪽으로 더 강하게 회피
                                else:
                                    # 오른쪽으로 회피
                                    self.ctrl_cmd_msg.steering = 0.7  # 왼쪽으로 더 강하게 회피
                                self.ctrl_cmd_msg.velocity = 8  # 속도를 줄여 회피를 더 안전하게 수행

                            elif self.dist_right < 4.0:  # 4.0m 내에 장애물이 있는 경우
                                # 오른쪽에 장애물이 있는 경우
                                self.ctrl_cmd_msg.steering = 0.7  # 왼쪽으로 더 강하게 회피
                                self.ctrl_cmd_msg.velocity = 8
                            elif self.dist_left < 4.0:  # 4.0m 내에 장애물이 있는 경우
                                # 왼쪽에 장애물이 있는 경우
                                self.ctrl_cmd_msg.steering = -0.7  # 오른쪽으로 더 강하게 회피
                                self.ctrl_cmd_msg.velocity = 8
                            else:
                                # 일반 주행
                                theta = self.find_theta()
                                self.ctrl_cmd_msg.steering = theta
                                self.ctrl_cmd_msg.velocity = 10
                elif self.is_shadow_drive == True :
                     self.check_obstacle()
                     self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
                                                 
            elif self.unexpected_obstacle == True :
                if self.is_car_moving == False :
                    self.ctrl_cmd_msg.steering = 0.0
                    self.ctrl_cmd_msg.velocity = 12.0
                    self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
                elif self.is_car_moving == True :
                    self.ctrl_cmd_msg.steering = 0.0
                    self.ctrl_cmd_msg.velocity = 0.0
                    self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
                    time.sleep(1)
                    self.is_car_moving =False
           
            rate.sleep()

    def path_callback(self, msg):
        self.is_path = True
        self.path = msg

    def odom_callback(self, msg):
        self.is_odom = True
        odom_quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        _, _, self.vehicle_yaw = euler_from_quaternion(odom_quaternion)
        self.current_position.x = msg.pose.pose.position.x
        self.current_position.y = msg.pose.pose.position.y
        if (-159 <= self.current_position.x <= -142 and 24 <= self.current_position.y <= 77):
            self.is_static_obstacle = True
        else :
            self.is_static_obstacle = False

    def global_path_callback(self, msg):
        self.global_path = msg
        self.is_global_path = True

    def dist_forward_callback(self, msg):
        self.obstacle_distance = msg.data
       
    def dist_callback(self, msg, direction):
        if direction == "l":
            self.dist_l = msg.data
        elif direction == "r":
            self.dist_r = msg.data
        elif direction == "f":
            self.dist_f = msg.data
       
    def dist2_callback(self, msg, direction):
        if direction == "left":
            self.dist_left = msg.data
        elif direction == "right":
            self.dist_right = msg.data
        elif direction == "front":
            self.dist_front = msg.data

        self.is_obstacle = (
            (self.dist_right and 1 < self.dist_right < 2.5) or
            (self.dist_left and 1 < self.dist_left < 2.5) or
            (self.dist_front and self.dist_front < 2.0)
        )
           
    def is_unexpected_obstacle_callback(self, msg) :
        if (-159 <= self.current_position.x <= -142 and 24 <= self.current_position.y <= 77):
            self.is_unexpected_obstacle = False
        else:
            self.is_unexpected_obstacle = msg.data
             
           
    # function for shadow_drive        
    def check_gps_and_obstacle(self):
        if self.is_shadow_drive == False:
            if self.current_position.x is not None and self.current_position.y is not None:
                if abs(self.current_position.x) < 0.01 and abs(self.current_position.y) < 0.01:
                    self.is_shadow_drive = True
        elif self.is_shadow_drive == True:
            if abs(self.current_position.x) > 0.01 or abs(self.current_position.y) > 0.01:
                self.is_shadow_drive = False
               
    def check_obstacle(self):
        if self.dist_f is not None and self.dist_f > 5: # 앞에 장애물이 없을때
            if self.dist_r is not None and self.dist_right < 1.6 and self.dist_r > 1: # 서서히 오른쪽 장애물에 접근 감지
                self.ctrl_cmd_msg.steering = 0.1
                self.ctrl_cmd_msg.velocity = 5
            elif self.dist_l is not None and self.dist_l < 1.6 and self.dist_l > 1: # 서서히 왼쪽 장애물에 접근 감지
                self.ctrl_cmd_msg.steering = -0.1
                self.ctrl_cmd_msg.velocity = 5
            elif self.dist_r is not None and self.dist_r < 1: # 바로 오른쪽에 장애물 존재
                self.ctrl_cmd_msg.steering = 1
                self.ctrl_cmd_msg.velocity = 5
            elif self.dist_l is not None and self.dist_l < 1: # 바로 왼쪽에 장애물 존재
                self.ctrl_cmd_msg.steering = -1
                self.ctrl_cmd_msg.velocity = 5      
            else:
                self.ctrl_cmd_msg.steering = 0
                self.ctrl_cmd_msg.velocity = 10      

        elif self.dist_f is not None and self.dist_f < 5: #앞에 장애물이 있을때
            if self.dist_r is not None and self.dist_r > 5: # 오른쪽에 길이 존재
                self.ctrl_cmd_msg.steering = -1                
                self.ctrl_cmd_msg.velocity = 3
            elif self.dist_l is not None and self.dist_l > 5: # 왼쪽에 길이 존재
                self.ctrl_cmd_msg.steering = 1                
                self.ctrl_cmd_msg.velocity = 3
            elif self.dist_r is not None and self.dist_r < 3 and self.dist_f < 4: # 큰 각도로 오른쪽 장애물에 접근
                self.ctrl_cmd_msg.steering = 1
                self.ctrl_cmd_msg.velocity = 5
            elif self.dist_l is not None and self.dist_l < 3 and self.dist_f < 4: # 큰 각도로 왼쪽 장애물에 접근
                self.ctrl_cmd_msg.steering = -1
                self.ctrl_cmd_msg.velocity = 5
            else:
                self.ctrl_cmd_msg.steering = 0
                self.ctrl_cmd_msg.velocity = 10    
               
               
    # function for acc control                        
    def acc_control(self):
        rospy.loginfo(f"ACC Control: Obstacle distance = {self.obstacle_distance:.2f}m")
       
        if self.obstacle_distance < 7 and self.obstacle_distance > 0:
            self.target_velocity = 0.0
            rospy.logwarn("Too close! Stopping.")
       
        elif self.obstacle_distance < 15 and self.obstacle_distance > 7:
            reduction_factor = self.obstacle_distance / self.safe_distance
            self.target_velocity *= reduction_factor
            self.target_velocity = max(self.target_velocity, 0.0)
            rospy.loginfo(f"Reducing speed: {self.target_velocity:.2f} km/h")
       
        elif self.obstacle_distance > 15 and self.obstacle_distance < 100:
            increase_factor = (self.obstacle_distance - self.safe_distance) / self.safe_distance
            self.target_velocity += increase_factor * 2.0
            self.target_velocity = min(self.target_velocity, 20.0)
            rospy.loginfo(f"Increasing speed: {self.target_velocity:.2f} km/h")

        elif self.obstacle_distance > 100:
            self.target_velocity = 12
            rospy.loginfo("Keep going 12km/h")

        return self.target_velocity

    def get_current_waypoint(self, current_position, global_path):
        min_dist = float('inf')
        current_waypoint = -1
        for i, pose in enumerate(global_path.poses):
            dx = current_position.x - pose.pose.position.x
            dy = current_position.y - pose.pose.position.y
            dist = sqrt(pow(dx, 2) + pow(dy, 2))
            if min_dist > dist:
                min_dist = dist
                current_waypoint = i
        return current_waypoint

    def calc_pure_pursuit(self):
        self.lfd = self.lfd_gain * min(self.max_lfd, max(self.min_lfd, self.target_velocity / 3.6))

        vehicle_position = self.current_position
        self.is_look_forward_point = False

        translation = [vehicle_position.x, vehicle_position.y]

        trans_matrix = np.array([[cos(self.vehicle_yaw), -sin(self.vehicle_yaw), 0],
                                 [sin(self.vehicle_yaw), cos(self.vehicle_yaw), 0],
                                 [0, 0, 1]])

        det_trans_matrix = np.linalg.inv(trans_matrix)

        for idx, pose in enumerate(self.path.poses):
            path_point = pose.pose.position
            global_path_point = [path_point.x - translation[0], path_point.y - translation[1], 1]
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
       
    #function for obstacle_mission
    def find_theta(self):
        vehicle_position=self.current_postion
        translation=[vehicle_position.x, vehicle_position.y]

        t=np.array([
                        [cos(self.vehicle_yaw), -sin(self.vehicle_yaw),translation[0]],
                        [sin(self.vehicle_yaw),cos(self.vehicle_yaw),translation[1]],
                        [0                    ,0                    ,1            ]])

        det_t=np.array([
                       [t[0][0],t[1][0],-(t[0][0]*translation[0]+t[1][0]*translation[1])],
                       [t[0][1],t[1][1],-(t[0][1]*translation[0]+t[1][1]*translation[1])],
                       [0      ,0      ,1                                               ]])

        for num,i in enumerate(self.path.poses) :
            path_point=i.pose.position
            global_path_point=[path_point.x,path_point.y,1]
            local_path_point=det_t.dot(global_path_point)          
            if local_path_point[0]>0 :
                dis=sqrt(pow(local_path_point[0],2)+pow(local_path_point[1],2))
                if dis>= self.lfd :
                    self.forward_point=path_point
                    self.is_look_forward_point=True
                    break
               
        theta=atan2(local_path_point[1],local_path_point[0])
       
        return theta

class velocityPlanning:
    def __init__(self, car_max_speed, road_friction):
        self.car_max_speed = car_max_speed
        self.road_friction = road_friction

    def curvedBaseVelocity(self, global_path, point_num):
        out_vel_plan = []

        for i in range(0, point_num):
            out_vel_plan.append(self.car_max_speed)

        for i in range(point_num, len(global_path.poses) - point_num):
            x_list = []
            y_list = []
            for box in range(-point_num, point_num):
                x = global_path.poses[i+box].pose.position.x
                y = global_path.poses[i+box].pose.position.y
                x_list.append([-2*x, -2*y ,1])
                y_list.append((-x*x) - (y*y))

            A = np.array(x_list)
            B = np.array(y_list)
            a, b, c = np.dot(np.linalg.pinv(A), B)
            r = (a**2 + b**2 - c)**0.5

            v_max = (r * 9.8 * 0.8) ** 0.5

            if v_max > self.car_max_speed:
                v_max = self.car_max_speed
            out_vel_plan.append(v_max)

        for i in range(len(global_path.poses) - point_num, len(global_path.poses) - 10):
            out_vel_plan.append(12)  # 최대 속도 12 km/h

        for i in range(len(global_path.poses) - 10, len(global_path.poses)):
            out_vel_plan.append(0)

        return out_vel_plan

if __name__ == '__main__':
    try:
        test_track = pure_pursuit()
    except rospy.ROSInterruptException:
        pass