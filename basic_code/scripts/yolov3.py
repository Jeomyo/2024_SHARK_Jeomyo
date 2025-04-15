#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32MultiArray
from sort import Sort  # You need to install SORT and import it

class IMGParser:
    def __init__(self):
        rospy.init_node('image_parser', anonymous=True)
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)
        self.bbox_pub = rospy.Publisher("/object_bounding_boxes", Float32MultiArray, queue_size=1)

        # YOLO 설정
        self.net = cv2.dnn.readNet("/home/jeomyo/catkin_ws/src/basic_code/scripts/yolov3.weights",
                                   "/home/jeomyo/catkin_ws/src/basic_code/scripts/yolov3.cfg")
        self.layer_names = self.net.getLayerNames()
        unconnected_out_layers = self.net.getUnconnectedOutLayers()
        self.output_layers = [self.layer_names[i - 1] for i in unconnected_out_layers.flatten()]

        self.classes = []
        with open("/home/jeomyo/catkin_ws/src/basic_code/scripts/coco.names", "r") as f:
            self.classes = [line.strip() for line in f.readlines()]

        self.latest_image = None
        self.processing = False  # 처리 중인지 확인하는 플래그

        # Initialize SORT tracker
        self.tracker = Sort(max_age=50, min_hits=3, iou_threshold=0.6)

    def callback(self, msg):
        if not self.processing:  # 처리 중이 아닐 때만 새로운 이미지를 받음
            np_arr = np.frombuffer(msg.data, np.uint8)
            self.latest_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    def process_image(self):
        if self.latest_image is not None:
            self.processing = True  # 처리 중 플래그 설정
            img = self.latest_image.copy()
            self.latest_image = None  # 이미지를 사용한 후 초기화
            self.detect_objects(img)
            self.processing = False  # 처리 완료 후 플래그 해제

    def detect_objects(self, img):
        height, width, _ = img.shape
        blob = cv2.dnn.blobFromImage(img, 0.00392, (480, 480), (0, 0, 0), True, crop=False)
        self.net.setInput(blob)
        outputs = self.net.forward(self.output_layers)

        boxes = []
        confidences = []
        class_ids = []
        object_labels = []  # 객체 클래스 이름 저장용

        for output in outputs:
            for detection in output:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > 0.3:  # 신뢰도 기준
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)

                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)

                    boxes.append([x, y, w, h, confidence])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)
                    object_labels.append(self.classes[class_id])  # 클래스 이름 저장

        indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
        object_boxes = []
        filtered_labels = []  # 객체 라벨 저장 리스트

        for i in range(len(boxes)):
            if i in indexes:
                label = str(self.classes[class_ids[i]])
                if label == "car" or label == "person":  # 'car' 또는 'person' 클래스만 추적
                    x, y, w, h, conf = boxes[i]
                    object_boxes.append([x, y, x + w, y + h, conf])  # x1, y1, x2, y2 형식으로
                    filtered_labels.append(label)  # 클래스 라벨을 추적에 맞게 저장

        # SORT로 트래킹을 업데이트
        tracked_objects = self.tracker.update(np.array(object_boxes))

    
        for idx, track in enumerate(tracked_objects):
            x1, y1, x2, y2, track_id = track
            label = filtered_labels[idx]  # 객체 라벨과 트랙 ID를 매칭
            bbox_data = Float32MultiArray(data=[x1, y1, x2 - x1, y2 - y1, track_id])
            self.bbox_pub.publish(bbox_data)  # 트래킹 ID와 함께 박스 송신

            # 객체 클래스 이름과 트랙 ID 출력
            print(f"Detected: {label}, Track ID: {int(track_id)}")

        if len(tracked_objects) == 0:
            print("No objects detected")

        if filtered_labels:
            print("Detected classes:", filtered_labels)

        

if __name__ == '__main__':
    try:
        img_parser = IMGParser()
        rate = rospy.Rate(50)  # 1Hz로 루프 실행
        while not rospy.is_shutdown():
            img_parser.process_image()  # 이미지 처리 함수 호출
            rate.sleep()  # 주기 제어
    except rospy.ROSInterruptException:
        pass
