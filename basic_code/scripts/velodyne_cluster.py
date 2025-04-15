#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseArray, Pose
from visualization_msgs.msg import Marker, MarkerArray
from sklearn.cluster import DBSCAN
from std_msgs.msg import Float32MultiArray

class SCANCluster:
    def __init__(self):
        self.scan_sub = rospy.Subscriber("/velodyne_points", PointCloud2, self.callback)
        self.cluster_pub = rospy.Publisher("/clusters", PoseArray, queue_size=1)
        self.bbox_size_pub = rospy.Publisher("/cluster_sizes", Float32MultiArray, queue_size=1)
        self.marker_pub = rospy.Publisher("/cluster_markers", MarkerArray, queue_size=1)
        self.closest_distance_pub = rospy.Publisher("/cluster_distances", Float32MultiArray, queue_size=1)  # ✅ 추가된 퍼블리셔

        self.dbscan = DBSCAN(eps=0.2, min_samples=3)

    def callback(self, msg):
        self.pc_np = self.pointcloud2_to_xyz(msg)

        # 클러스터링 실행 (중앙 좌표 및 바운딩 박스 크기 반환)
        cluster_corners_and_center, bbox_sizes, marker_array, closest_distances = self.cluster(self.pc_np[:, :2])
        
        self.cluster_pub.publish(cluster_corners_and_center)  
        self.marker_pub.publish(marker_array)  

        # 바운딩 박스 크기 퍼블리시
        bbox_sizes_msg = Float32MultiArray()
        bbox_sizes_msg.data = bbox_sizes
        self.bbox_size_pub.publish(bbox_sizes_msg)

        # 최근접 거리 퍼블리시
        distance_msg = Float32MultiArray()
        distance_msg.data = closest_distances
        self.closest_distance_pub.publish(distance_msg)

    def pointcloud2_to_xyz(self, cloud_msg):
        point_list = []
        for point in pc2.read_points(cloud_msg, skip_nans=True):
            x, y, z = point[:3]

            # ✅ 방위각(Azimuth) 계산
            azimuth = np.arctan2(y, x) * 180 / np.pi  

            # ✅ 가이드봇이 있는 좁은 후방 (-30° ~ 30°) 제거
            if azimuth < -60 or azimuth > 60:
                point_list.append((x, y, z))  # ✅ 필터링된 포인트 추가

        return np.array(point_list, np.float32)

    def cluster(self, xy):
        db = self.dbscan.fit_predict(xy)
        unique_clusters = np.unique(db)

        cluster_msg = PoseArray()
        cluster_msg.header.frame_id = "map"
        cluster_msg.header.stamp = rospy.Time.now()

        marker_array = MarkerArray()
        bbox_sizes = []
        closest_distances = []

        for i, cluster in enumerate(unique_clusters):
            if cluster == -1:
                continue

            cluster_points = xy[db == cluster]
            x_min, y_min = np.min(cluster_points, axis=0)
            x_max, y_max = np.max(cluster_points, axis=0)

            center_pose = Pose()
            center_pose.position.x = np.mean(cluster_points[:, 0])
            center_pose.position.y = np.mean(cluster_points[:, 1])
            center_pose.position.z = 0
            cluster_msg.poses.append(center_pose)

            bbox_sizes.extend([x_max - x_min, y_max - y_min])

            # 최근접 점 찾기
            dists = np.linalg.norm(cluster_points, axis=1)
            min_dist = np.min(dists)
            closest_distances.append(min_dist)

            # rospy.loginfo(f"물체 {i+1} 거리: {min_dist:.2f}m")  # 터미널 출력
            # 거리가 0.2m 이하인 경우 경고 출력
            if min_dist <= 0.2:
                rospy.logwarn(f"⚠️ 경고: 물체 {i+1}이(가) 너무 가까움! 거리: {min_dist:.2f}m")
            elif min_dist >= 0.2 and min_dist <= 0.4:
                print(f" 물체 {i+1}이(가) 가까움! 거리: {min_dist:.2f}m")

            # Bounding Box Marker 생성
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "cluster"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = (x_min + x_max) / 2
            marker.pose.position.y = (y_min + y_max) / 2
            marker.pose.position.z = 0.1  
            marker.scale.x = x_max - x_min
            marker.scale.y = y_max - y_min
            marker.scale.z = 0.1  
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.5  

            marker_array.markers.append(marker)  

        return cluster_msg, bbox_sizes, marker_array, closest_distances  # 최근접 거리 리스트 반환

if __name__ == '__main__':
    rospy.init_node('velodyne_clustering', anonymous=True)
    scan_cluster = SCANCluster()
    rospy.spin()
