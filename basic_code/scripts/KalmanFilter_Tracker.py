#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray, Float32
from filterpy.kalman import KalmanFilter
from scipy.spatial import distance

class KalmanFilterTracker:
    def __init__(self):
        # 칼만 필터 초기화
        self.kf = KalmanFilter(dim_x=6, dim_z=3)
        self.kf.F = np.array([[1, 0, 0, 1, 0, 0],
                              [0, 1, 0, 0, 1, 0],
                              [0, 0, 1, 0, 0, 1],
                              [0, 0, 0, 1, 0, 0],
                              [0, 0, 0, 0, 1, 0],
                              [0, 0, 0, 0, 0, 1]])
        self.kf.H = np.array([[1, 0, 0, 0, 0, 0],
                              [0, 1, 0, 0, 0, 0],
                              [0, 0, 1, 0, 0, 0]])
        self.kf.P *= 1000.0  # 초기 상태 불확실성
        self.kf.R = np.eye(3) * 0.1  # 측정 불확실성
        self.kf.Q = np.eye(6) * 0.01  # 프로세스 잡음

        # 이전 위치 저장 변수
        self.prev_position = None

        # 동적/정적 상태를 퍼블리시하는 토픽
        self.dyn_pub = rospy.Publisher('/vehicle_dynamic_status', Float32, queue_size=10)

        # 3D 좌표 구독 (캘리브레이션 후 3D 좌표를 수신)
        rospy.Subscriber('/vehicle_3d_coordinates', Float32MultiArray, self.lidar_callback)

    def lidar_callback(self, msg):
        # 수신된 3D 좌표
        position = np.array(msg.data).reshape(-1, 3)

        if len(position) == 0:
            return  # 유효한 좌표가 없으면 처리하지 않음

        # 현재 프레임에서 3D 좌표의 평균 좌표 계산
        current_position = np.mean(position, axis=0)

        if self.prev_position is not None:
            # 칼만 필터 예측 및 업데이트
            self.kf.predict()
            self.kf.update(current_position)

            # 예측된 위치와 실제 위치의 차이 계산
            predicted_position = self.kf.x[:3]
            dist_moved = distance.euclidean(predicted_position, self.prev_position)

            # 동적/정적 상태 판단 (임계값 설정)
            threshold = 0.1  # 움직임 판단 기준 (미터 단위)
            if dist_moved > threshold:
                rospy.loginfo(f"Vehicle is dynamic. Moved: {dist_moved} meters.")
                self.dyn_pub.publish(1.0)  # 동적 상태
            else:
                rospy.loginfo(f"Vehicle is static. Moved: {dist_moved} meters.")
                self.dyn_pub.publish(0.0)  # 정적 상태

        # 현재 위치를 이전 위치로 저장
        self.prev_position = current_position


if __name__ == '__main__':
    rospy.init_node('kalman_filter_tracker', anonymous=True)
    tracker = KalmanFilterTracker()
    rospy.spin()
