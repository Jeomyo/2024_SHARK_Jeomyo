#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import rospy
import numpy as np
from math import atan2
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseArray, Pose
from sklearn.cluster import DBSCAN

class SCANCluster:
    def __init__(self):

        self.scan_sub = rospy.Subscriber("/velodyne_points", PointCloud2, self.callback)
        self.cluster_pub = rospy.Publisher("/clusters", PoseArray, queue_size=1)

        self.pc_np = None

        self.cluster_msg = PoseArray()

        self.dbscan = DBSCAN(eps=0.3, min_samples=3)

    def callback(self, msg):
        self.pc_np = self.pointcloud2_to_xyz(msg)

        pc = self.cluster(self.pc_np)
        self.cluster_pub.publish(pc)

    def pointcloud2_to_xyz(self, cloud_msg):
        point_list = []

        # PointCloud2 메시지를 numpy 배열로 변환
        for point in pc2.read_points(cloud_msg, skip_nans=True):
            if point[0] > 0 and point[1] > -5 and point[1] < 5 and point[2] > -0.77 and point[2] < 2.7:
                point_list.append((point[0], point[1], point[2]))

        point_np = np.array(point_list, np.float32)  # 리스트를 numpy 배열로 변환
        return point_np
    
    def cluster(self, xy_np):
        xy = xy_np[:, :2]  # x, y 좌표 추출

        db = self.dbscan.fit_predict(xy)  # DBSCAN 클러스터링 수행

        n_cluster = np.max(db) + 1  # 클러스터 개수

        self.cluster_msg = PoseArray()  # PoseArray 초기화
        self.cluster_msg.header.frame_id = "/map"
        self.cluster_msg.header.stamp = rospy.Time.now()

        # 각 클러스터의 중심 계산 및 PoseArray에 추가
        for cluster in range(n_cluster):
            c_tmp = np.mean(xy[db == cluster, :], axis=0)  # 클러스터 중심 계산

            # Pose 객체 생성 및 중심 좌표 할당
            pose = Pose()
            pose.position.x = c_tmp[0]
            pose.position.y = c_tmp[1]

            self.cluster_msg.poses.append(pose)  # PoseArray에 Pose 추가

        return self.cluster_msg


if __name__ == '__main__':
    rospy.init_node('velodyne_clustering', anonymous=True)
    scan_cluster = SCANCluster()
    rospy.spin()
