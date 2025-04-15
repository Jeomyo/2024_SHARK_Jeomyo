#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
import math
from sensor_msgs.msg import PointCloud2, PointCloud, CompressedImage
from geometry_msgs.msg import Point32
from std_msgs.msg import Float32  # 바운딩 박스 데이터 송신
import sensor_msgs.point_cloud2 as pc2
from scipy.spatial import KDTree
from numpy.linalg import inv

parameters_cam = {
    "WIDTH": 640,  # image width
    "HEIGHT": 480,  # image height
    "FOV": 90,  # Field of view
    "X": 3.18,  # meter
    "Y": 0.08,
    "Z": 0.6,
    "YAW": 0,  # radian
    "PITCH": 0,
    "ROLL": 0,
}

parameters_lidar = {
    "X": 3.8,  # meter
    "Y": 0.01,
    "Z": 0.43,
    "YAW": 0.0,  # radian
    "PITCH": 0.0,
    "ROLL": 0,
}

def getRotMat(RPY):
    cosR = math.cos(RPY[0])  # roll, x축
    cosP = math.cos(RPY[1])  # pitch, y축
    cosY = math.cos(RPY[2])  # yaw z축
    sinR = math.sin(RPY[0])
    sinP = math.sin(RPY[1])
    sinY = math.sin(RPY[2])

    rotRoll = np.array([1, 0, 0, 0, cosR, -sinR, 0, sinR, cosR]).reshape(3, 3)
    rotPitch = np.array([cosP, 0, sinP, 0, 1, 0, -sinP, 0, cosP]).reshape(3, 3)
    rotYaw = np.array([cosY, -sinY, 0, sinY, cosY, 0, 0, 0, 1]).reshape(3, 3)

    rotMat = rotYaw.dot(rotPitch.dot(rotRoll))
    return rotMat

def getSensorToVehicleMat(sensorRPY, sensorPosition):
    sensorRotationMat = getRotMat(sensorRPY)
    sensorTranslationMat = np.array([sensorPosition])

    Tr_sensor_to_vehicle = np.concatenate((sensorRotationMat, sensorTranslationMat.T), axis=1)
    Tr_sensor_to_vehicle = np.insert(Tr_sensor_to_vehicle, 3, values=[0, 0, 0, 1], axis=0)

    return Tr_sensor_to_vehicle

def getLiDARTOCameraTransformMat(camRPY, camPosition, lidarRPY, lidarPosition):
    Tr_lidar_to_vehicle = getSensorToVehicleMat(lidarRPY, lidarPosition)
    Tr_cam_to_vehicle = getSensorToVehicleMat(camRPY, camPosition)
    Tr_vehicle_to_cam = inv(Tr_cam_to_vehicle)
    Tr_lidar_to_cam = Tr_vehicle_to_cam.dot(Tr_lidar_to_vehicle).round(6)

    print(Tr_lidar_to_cam)
    return Tr_lidar_to_cam

def getTransformMat(params_cam, params_lidar):
    lidarPositionOffset = np.array([0, 0, -0.25])  # VLP16 사용해야 함
    camPositionOffset = np.array([0.1085, 0, 0])  # Camera Offset

    camPosition = np.array([params_cam.get(i) for i in (["X", "Y", "Z"])]) + camPositionOffset
    camRPY = np.array([params_cam.get(i) for i in (["ROLL", "PITCH", "YAW"])]) + np.array([-90 * math.pi / 180, 0, -90 * math.pi / 180])
    lidarPosition = np.array([params_lidar.get(i) for i in (["X", "Y", "Z"])]) + lidarPositionOffset
    lidarRPY = np.array([params_lidar.get(i) for i in (["ROLL", "PITCH", "YAW"])])
    Tr_lidar_to_cam = getLiDARTOCameraTransformMat(camRPY, camPosition, lidarRPY, lidarPosition)
    return Tr_lidar_to_cam

def getCameraMat(params_cam):
    focalLength = params_cam["WIDTH"] / (2 * np.tan(np.deg2rad(params_cam["FOV"] / 2)))
    principalX = params_cam["WIDTH"] / 2
    principalY = params_cam["HEIGHT"] / 2
    CameraMat = np.array([focalLength, 0., principalX, 0, focalLength, principalY, 0, 0, 1]).reshape(3, 3)

    print(CameraMat)
    return CameraMat

class LiDARToCameraTransform:
    def __init__(self, params_cam, params_lidar):
        self.scan_sub = rospy.Subscriber("/velodyne_points", PointCloud2, self.scan_callback)  # LiDAR 포인트 클라우드 구독
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.img_callback)  # 카메라 이미지 구독
        self.bbox_sub = rospy.Subscriber("/car_bounding_boxes", Float32MultiArray, self.bbox_callback)  # YOLO 바운딩 박스 구독
        # 차량과의 거리 퍼블리셔 추가
        self.dist_pub = rospy.Publisher('/distance_to_car', Float32, queue_size=1)
        self.pc_pub = rospy.Publisher('/calib_points', PointCloud, queue_size=1)
        self.pc_np = None
        self.img = None
        self.width = params_cam["WIDTH"]
        self.height = params_cam["HEIGHT"]
        self.TransformMat = getTransformMat(params_cam, params_lidar)
        self.CameraMat = getCameraMat(params_cam)
        self.bboxes = []

    def img_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        self.img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    def scan_callback(self, msg):
        point_list = []
        for point in pc2.read_points(msg, skip_nans=True):
            point_list.append((point[0], point[1], point[2], 1))
        self.pc_np = np.array(point_list, np.float32)

    def bbox_callback(self, msg):
        # 바운딩 박스 데이터 수신
        self.bboxes = [msg.data]  # 최신 바운딩 박스로 덮어쓰기
        self.process_lidar_data()  # 바운딩 박스 수신 후 LiDAR 데이터 처리

    def transformLiDARToCamera(self, pc_lidar):
        pc_wrt_cam = self.TransformMat.dot(pc_lidar)
        pc_wrt_cam = np.delete(pc_wrt_cam, 3, axis=0)
        return pc_wrt_cam

    def transformCameraToImage(self, pc_camera):
        # 투영된 포인트가 이미지 프레임 바깥에 있는 경우를 제거
        pc_proj_to_img = self.CameraMat.dot(pc_camera)
        
        # 카메라 좌표계에서 Z축이 0 이하인 포인트는 제외 (카메라 뒤쪽에 있는 포인트)
        valid_indices = np.where(pc_proj_to_img[2, :] > 0)
        pc_proj_to_img = pc_proj_to_img[:, valid_indices[0]]
        pc_camera = pc_camera[:, valid_indices[0]]
        
        # 이미지 좌표로 변환
        pc_proj_to_img /= pc_proj_to_img[2, :]

        # 이미지 범위를 벗어난 포인트 제외
        valid_indices = (pc_proj_to_img[0, :] >= 0) & (pc_proj_to_img[0, :] < self.width) & \
                        (pc_proj_to_img[1, :] >= 0) & (pc_proj_to_img[1, :] < self.height)
        pc_proj_to_img = pc_proj_to_img[:, valid_indices]
        pc_camera = pc_camera[:, valid_indices]
        
        return pc_proj_to_img, pc_camera  # 투영된 이미지 좌표와 대응하는 3D 좌표 반환

    def calculate_distance_to_car(self, bbox, lidar_points_image, lidar_points_3d):
        # bbox는 [x1, y1, w, h] 형식으로 전달됩니다.
        x1, y1, w, h = bbox
        x2, y2 = x1 + w, y1 + h

        # 바운딩 박스 영역 안에 있는 LiDAR 포인트 필터링
        mask = (lidar_points_image[0, :] >= x1) & (lidar_points_image[0, :] <= x2) & \
               (lidar_points_image[1, :] >= y1) & (lidar_points_image[1, :] <= y2)

        filtered_points_3d = lidar_points_3d[:, mask]

        if filtered_points_3d.size > 0:
            # KD-Tree를 사용해 가장 가까운 포인트 찾기
            tree = KDTree(filtered_points_3d.T)
            min_distance = tree.query([0, 0, 0], k=1)[0]  # 차량에서 가장 가까운 포인트의 거리 (z축 값 기준)
            return min_distance, filtered_points_3d
        else:
            return None, None

    def create_pointcloud(self, points):
        pointcloud_msg = PointCloud()
        pointcloud_msg.header.stamp = rospy.Time.now()
        pointcloud_msg.header.frame_id = 'base_link'

        for point in points.T:
            pointcloud_msg.points.append(Point32(point[0], point[1], point[2]))

        return pointcloud_msg

    def process_lidar_data(self):
        if self.pc_np is not None and self.img is not None and self.bboxes:
            xyz_p = self.pc_np[:, 0:3]
            xyz_p = np.insert(xyz_p, 3, 1, axis=1).T
            xyz_p = np.delete(xyz_p, np.where(xyz_p[0, :] < 0), axis=1)

            # LiDAR 데이터를 카메라 좌표계로 변환 및 이미지 평면으로 투영
            xyz_c, xyz_p_filtered = self.transformLiDARToCamera(xyz_p), xyz_p
            xy_i, xyz_c_filtered = self.transformCameraToImage(xyz_c)

            # 바운딩 박스별로 거리 계산
            found_distance = False
            for bbox in self.bboxes:

                dist_forward, filtered_point_3d = self.calculate_distance_to_car(bbox, xy_i, xyz_c_filtered)
                if dist_forward is not None:
                    rospy.loginfo(f"Detected car at distance: {dist_forward} meters")
                    self.dist_pub.publish(Float32(dist_forward))  # 거리 퍼블리시

                    # PointCloud 메시지로 퍼블리시
                    pointcloud_msg = self.create_pointcloud(filtered_point_3d)
                    self.pc_pub.publish(pointcloud_msg)
                    found_distance = True

            if not found_distance:
                # 유효한 바운딩 박스가 없을 경우, inf 값을 퍼블리시
                rospy.logwarn("No detect.")
                self.dist_pub.publish(Float32(float('inf')))

if __name__ == '__main__':
    rospy.init_node('ex_calib', anonymous=True)
    Transformer = LiDARToCameraTransform(parameters_cam, parameters_lidar)
    rospy.spin()
