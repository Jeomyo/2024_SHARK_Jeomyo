#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf
from math import pi
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker  # Marker 메시지 추가

# tf 는 물체의 위치와 자세 데이터를 좌표계로 나타내는 예제입니다.

# 노드 실행 순서 
# 1. Callback 함수 생성
# 2. 브로드캐스터 생성 및 Ego 상태 tf 브로드캐스팅
# 3. RViz 시각화를 위한 Marker 퍼블리셔 생성

class Ego_listener():
    def __init__(self):
        rospy.init_node('status_listener', anonymous=True)
        
        rospy.Subscriber("odom", Odometry, self.odom_callback)

        # Marker 퍼블리셔 추가
        self.marker_pub = rospy.Publisher('/ego_marker', Marker, queue_size=10)

        rospy.spin()

    #TODO: (1) Callback 함수 생성
    def odom_callback(self, msg):
        self.is_odom = True

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        self.orientation_x = msg.pose.pose.orientation.x
        self.orientation_y = msg.pose.pose.orientation.y
        self.orientation_z = msg.pose.pose.orientation.z
        self.orientation_w = msg.pose.pose.orientation.w

        # TODO: (2) 브로드캐스터 생성 및 Ego 상태 tf 브로드캐스팅
        br = tf.TransformBroadcaster()
        br.sendTransform((self.x, self.y, 1),
                        (self.orientation_x, self.orientation_y, self.orientation_z, self.orientation_w),
                        rospy.Time.now(),
                        "Ego",
                        "map")
        
        # TODO: (3) Marker 생성 및 퍼블리시
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "ego_vehicle"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = self.x
        marker.pose.position.y = self.y
        marker.pose.position.z = 0.5  # 마커의 z축 높이
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 1.0  # 마커의 크기 설정 (x축)
        marker.scale.y = 1.0  # 마커의 크기 설정 (y축)
        marker.scale.z = 1.0  # 마커의 크기 설정 (z축)

        marker.color.a = 1.0  # 투명도 (1.0: 불투명, 0.0: 투명)
        marker.color.r = 0.0  # 빨간색
        marker.color.g = 1.0  # 녹색 (차량을 녹색으로 표현)
        marker.color.b = 0.0  # 파란색

        # Marker 퍼블리시
        self.marker_pub.publish(marker)

if __name__ == '__main__':
    try:
        tl = Ego_listener()
    except rospy.ROSInternalException:
        pass
