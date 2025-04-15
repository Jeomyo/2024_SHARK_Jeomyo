#!/usr/bin/env python3
# -*- coding: utf-8 -*-
 
import rospy
import cv2
import numpy as np
import os, rospkg
from std_msgs.msg import String  # 퍼블리시할 데이터 타입
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError

class IMGParser:
    def __init__(self):
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)
        self.lc_pub = rospy.Publisher("/lane_position", String, queue_size=1)

    def callback(self, msg):
        try:
            # 카메라 데이터 받아서 데이터 변환 - Byte 단위 이미지 -> np.array
            np_arr = np.frombuffer(msg.data, dtype='uint8')
            # 1차원 배열 형태 np_arr를 3차원 배열의 bgr 형태로 변환
            img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        except CvBridgeError as e:
            print(e)

        # cv2.cvtColor() - bgr 이미지 -> HSV 이미지 
        img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)

        lower_ylane = np.array([20, 100, 100])  # H, S, V 값으로 노란색 시작점
        upper_ylane = np.array([30, 255, 255])  # H, S, V 값으로 노란색 끝점

        img_ylane = cv2.inRange(img_hsv, lower_ylane, upper_ylane)

        # 사다리꼴 좌표
        top_left = (280, 200)
        top_right = (360, 200)
        bottom_left = (0, 375)
        bottom_right = (640, 375)

        # 사다리꼴 좌표 배열로 만들기
        roi_vertices = np.array([[bottom_left, top_left, top_right, bottom_right]], dtype=np.int32)

        # ROI 마스크 생성
        mask = np.zeros_like(img_ylane)
        cv2.fillPoly(mask, roi_vertices, (255, 255, 255))

        # 마스크를 적용해 ROI 영역만 남김
        img_lane_roi = cv2.bitwise_and(img_ylane, mask)

        # 이미지를 좌측, 우측으로 나누기 위한 경계 설정
        left_region = img_lane_roi[:, :320]
        right_region = img_lane_roi[:, 320:]

        # 각 영역에서 노란색 차선 픽셀 수 계산
        left_lane_pixels = cv2.countNonZero(left_region)
        right_lane_pixels = cv2.countNonZero(right_region)

        #노란색 차선이 가장 많이 검출된 영역 확인
        if left_lane_pixels > 100000:
            lane_position = "Left"
        elif left_lane_pixels < right_lane_pixels:
            lane_position = "Right"
        else:
            lane_position = "Center"

        lane_msg = String()
        lane_msg.data = lane_position
        self.lc_pub.publish(lane_msg)  # 퍼블리시

        # 차선 위치 결과 출력
        print(f"left: {left_lane_pixels}, right: {right_lane_pixels} .")

        # 결과 이미지를 시각적으로 출력
        cv2.imshow('Lane ROI', img_lane_roi)  # ROI 영역이 적용된 이미지를 띄움
        cv2.waitKey(1)  # 키 입력을 기다리지 않지만 창을 계속 띄움

if __name__ == '__main__':
    rospy.init_node('image_parser', anonymous=True)
    image_parser = IMGParser()
    rospy.spin()
