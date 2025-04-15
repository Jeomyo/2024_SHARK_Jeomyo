#!/usr/bin/env python3
# -*- coding: utf-8 -*- 

import rospy
import numpy as np
import math
from sensor_msgs.msg import PointCloud
from morai_msgs.msg import EgoVehicleStatus
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, Point32
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Bool

class Cluster_viz:

    def __init__(self):

        rospy.Subscriber("/clusters", PoseArray, self.callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/Competition_topic", EgoVehicleStatus, self.status_callback)

        self.object_pointcloud_pub = rospy.Publisher('object_pointcloud_data', PointCloud, queue_size=1)


        self.is_odom = False
        self.cluster_status = False
        self.dangerous_status = False
        self.prev_dangerous_data = None
        self.cluster_data = None  # 초기화

        rate = rospy.Rate(50)  # 30hz
        while not rospy.is_shutdown():
            if self.is_odom and self.cluster_status:
                
                # (1) 좌표 변환 행렬 생성
                trans_matrix = self.trans_matrix(self.vehicle_yaw)

                obj_data_cluster = self.tf_global(trans_matrix, self.cluster_data)

                self.object_pointcloud_pub.publish(obj_data_cluster)

            rate.sleep()

    def trans_matrix(self, vehicle_yaw):
        # (1) 좌표 변환 행렬 생성
        trans_matrix = np.array([[math.cos(vehicle_yaw), -math.sin(vehicle_yaw), 0],
                                 [math.sin(vehicle_yaw), math.cos(vehicle_yaw), 0],
                                 [0, 0, 1]], dtype=np.float32)
        return trans_matrix
    
    def tf_global(self, trans_matrix, cluster_data):
        obj_data = PointCloud()
        obj_data.header.frame_id = 'map'
        obj_data.header.stamp = rospy.Time.now()  # 타임스탬프 추가
        obj_data.points = []  # 포인트 리스트 초기화

        vehicle_pos_x = self.vehicle_pos_x  # 루프 밖으로 이동
        vehicle_pos_y = self.vehicle_pos_y  # 루프 밖으로 이동

        for num, i in enumerate(cluster_data.poses):

            # (2) 전역 좌표로 변환
            local_result = [i.position.x, i.position.y, 1]
            temp = trans_matrix.dot(local_result)
            global_result = [temp[0] + vehicle_pos_x, temp[1] + vehicle_pos_y]

            # (3) 전역 좌표를 PointCloud에 추가
            tmp_point = Point32()
            tmp_point.x = global_result[0]
            tmp_point.y = global_result[1]
            tmp_point.z = 1.0  # 필요에 따라 수정 가능
            obj_data.points.append(tmp_point)

        return obj_data

    def callback(self, msg):    
        self.cluster_data = msg
        self.cluster_status = True

    def status_callback(self, msg):  # 속도만 처리
        self.is_status = True
        self.vehicle_velocity = msg.velocity.x * 3.6  # 속도를 km/h로 변환

    def odom_callback(self, msg):
        self.is_odom = True
        odom_quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        _, _, self.vehicle_yaw = euler_from_quaternion(odom_quaternion)
        self.vehicle_pos_x = msg.pose.pose.position.x
        self.vehicle_pos_y = msg.pose.pose.position.y

if __name__ == '__main__':

    rospy.init_node('velodyne_clustering', anonymous=True)

    Cluster_visualization = Cluster_viz()  # 오타 수정

    rospy.spin()
