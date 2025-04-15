#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import rospkg
import json
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


class ReadPathPub:

    def __init__(self):
        rospy.init_node('read_path_pub', anonymous=True)
        self.global_path_pub = rospy.Publisher('/global_path', Path, queue_size=1)

        self.global_path_msg = Path()
        self.global_path_msg.header.frame_id = '/map'

        # ROS package path를 통해 JSON 파일 경로 설정
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('basic_code')
        full_path = pkg_path + '/path' + '/link_set.json'

        # JSON 파일 읽기
        with open(full_path, 'r') as f:
            link_set = json.load(f)

        # link_set 데이터를 읽어 global_path_msg에 경로 추가
        for link in link_set:
            for point in link['points']:
                read_pose = PoseStamped()
                read_pose.pose.position.x = point[0]
                read_pose.pose.position.y = point[1]
                read_pose.pose.position.z = point[2]
                read_pose.pose.orientation.w = 1
                self.global_path_msg.poses.append(read_pose)

        # 주기적으로 경로를 퍼블리시
        rate = rospy.Rate(20)  # 20Hz
        while not rospy.is_shutdown():
            self.global_path_msg.header.stamp = rospy.Time.now()
            self.global_path_pub.publish(self.global_path_msg)
            rate.sleep()


if __name__ == '__main__':
    try:
        test_track = ReadPathPub()
    except rospy.ROSInterruptException:
        pass
