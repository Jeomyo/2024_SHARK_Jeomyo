#!/usr/bin/env python3

import rospy
import numpy as np

from morai_msgs.msg import CtrlCmd
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry

class shadow_drive():
    def __init__(self):
        rospy.init_node('shadow_drive', anonymous=True)
        self.dist_f_sub = rospy.Subscriber("dist_f", Float32, self.dist_f_callback)
        self.dist_r_sub = rospy.Subscriber("dist_r", Float32, self.dist_r_callback)
        self.dist_l_sub = rospy.Subscriber("dist_l", Float32, self.dist_l_callback)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/distance_to_car", Float32, self.dist_forward_callback)

        self.cmd_pub = rospy.Publisher("ctrl_cmd", CtrlCmd, queue_size=1)

        self.dist_f = None
        self.dist_l = None
        self.dist_r = None
        self.obstacle_distance = None

        self.current_x = None
        self.current_y = None

        self.rate = rospy.Rate(30)
        self.cmd = CtrlCmd()
        self.cmd.longlCmdType = 2
        self.cmd.velocity = 10
        self.cmd.steering = 0

        self.acc_active = True 
        self.target_velocity = 10.0  # 초기 ACC 속도 설정
        self.safe_distance = 20.0  # 안전 거리 초기화

        rospy.spin()

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.check_gps_and_obstacle()

    def dist_f_callback(self, msg):
        self.dist_f = msg.data
        self.check_gps_and_obstacle()

    def dist_r_callback(self, msg):
        self.dist_r = msg.data
        self.check_gps_and_obstacle()

    def dist_l_callback(self, msg):
        self.dist_l = msg.data
        self.check_gps_and_obstacle()

    def dist_forward_callback(self, msg):
        self.obstacle_distance = msg.data
        if self.acc_active:
            self.acc_control()  # ACC 제어는 acc_active가 True일 때만 작동

    def check_gps_and_obstacle(self):
        if self.current_x is not None and self.current_y is not None:
            if abs(self.current_x) < 0.01 and abs(self.current_y) < 0.01:
                self.check_obstacle()
                self.acc_active = False  # 장애물 회피 중에는 ACC 비활성화

    def check_obstacle(self):
        obstacle_avoided = False  # 장애물 회피 완료 플래그

        if self.dist_f is not None and self.dist_f > 5:  # 앞에 장애물이 없을 때
            if self.dist_r is not None and 1 < self.dist_r < 1.6:  # 오른쪽 장애물 접근 감지
                self.cmd.steering = 0.1
                self.cmd.velocity = 5
            elif self.dist_l is not None and 1 < self.dist_l < 1.6:  # 왼쪽 장애물 접근 감지
                self.cmd.steering = -0.1
                self.cmd.velocity = 5
            elif self.dist_r is not None and self.dist_r < 1:  # 바로 오른쪽에 장애물 존재
                self.cmd.steering = 1
                self.cmd.velocity = 5
            elif self.dist_l is not None and self.dist_l < 1:  # 바로 왼쪽에 장애물 존재
                self.cmd.steering = -1
                self.cmd.velocity = 5       

                obstacle_avoided = True  # 장애물 회피 완료

        elif self.dist_f is not None and self.dist_f < 5:  # 앞에 장애물이 있을 때
            if self.dist_r is not None and self.dist_r > 5:  # 오른쪽에 길이 존재
                self.cmd.steering = -1                
                self.cmd.velocity = 3
            elif self.dist_l is not None and self.dist_l > 5:  # 왼쪽에 길이 존재
                self.cmd.steering = 1                
                self.cmd.velocity = 3
            elif self.dist_r is not None and self.dist_r < 3 and self.dist_f < 4:  # 큰 각도로 오른쪽 장애물에 접근
                self.cmd.steering = 1
                self.cmd.velocity = 5
            elif self.dist_l is not None and self.dist_l < 3 and self.dist_f < 4:  # 큰 각도로 왼쪽 장애물에 접근
                self.cmd.steering = -1
                self.cmd.velocity = 5          
                obstacle_avoided = True  # 장애물 회피 완료

        self.cmd_pub.publish(self.cmd)

        if obstacle_avoided:
            self.acc_active = True  # 장애물 회피가 완료되면 ACC 활성화

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
            self.target_velocity += increase_factor * 1.5
            self.target_velocity = min(self.target_velocity, 15.0)
            rospy.loginfo(f"Increasing speed: {self.target_velocity:.2f} km/h")

        elif self.obstacle_distance > 100:
            self.target_velocity = 10
            rospy.loginfo("Keep going 10km/h")

        self.cmd.velocity = self.target_velocity
        self.cmd_pub.publish(self.cmd)

if __name__ == '__main__':
    shadow_drive()
