#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
import math

class RPLidarProcessor:
    def __init__(self):
        rospy.init_node('rplidar_angle_distance_publisher', anonymous=True)
        
        self.sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.pub = rospy.Publisher('/rplidar_angle_distance', Float32MultiArray, queue_size=10)
        
        rospy.loginfo("📡 RPLIDAR Processor Node Initialized")

    def scan_callback(self, scan_msg):
        angle_min = scan_msg.angle_min
        angle_increment = scan_msg.angle_increment
        ranges = scan_msg.ranges

        angle_distance_data = []

        for i, r in enumerate(ranges):
            if math.isinf(r) or math.isnan(r):
                continue

            angle_rad = angle_min + i * angle_increment
            angle_deg = math.degrees(angle_rad)

            # ✅ 방위각 -60° ~ +60° 사이만 처리
            if -60 <= angle_deg <= 60:
                # ✅ 거리 기준 알림
                if r <= 0.3:
                    print(f"🚨 [위험] {round(r*100)}cm 거리에서 물체 감지! (각도: {round(angle_deg)}°)")
                elif r <= 0.5:
                    print(f"⚠️ [경고] {round(r*100)}cm 거리에서 물체 감지 (각도: {round(angle_deg)}°)")

                # 각도와 거리 저장 (필요 시 사용)
                angle_distance_data.append(angle_deg)
                angle_distance_data.append(r)

        # 퍼블리시
        msg = Float32MultiArray()
        msg.data = angle_distance_data
        self.pub.publish(msg)

if __name__ == '__main__':
    try:
        node = RPLidarProcessor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
