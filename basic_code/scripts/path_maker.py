#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import os
import math
import rospkg
from std_msgs.msg import Float32MultiArray

class PathMaker:
    def __init__(self):
        rospy.init_node('path_maker', anonymous=True)
        rospy.Subscriber('/utm_coordinates', Float32MultiArray, self.status_callback)
        
        self.prev_x = 0
        self.prev_y = 0
        self.is_status = False
        
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('basic_code')
        full_path = pkg_path + '/scripts' + '/HL.txt'
        self.f = open(full_path, 'w')
    
        while not rospy.is_shutdown():
            if self.is_status:
                self.path_make()

        self.f.close()

    def path_make(self):
        x = self.status_msg.data[0]
        y = self.status_msg.data[1]

        distance = math.sqrt((x - self.prev_x)**2 + (y - self.prev_y)**2)
        if distance > 0.3:
            data = '{0}\t{1}\n'.format(x, y)
            self.f.write(data)
            # 터미널에도 출력
            print(f'{x}\t{y}')
            self.prev_x = x
            self.prev_y = y

    def status_callback(self, msg):
        self.is_status = True
        self.status_msg = msg

if __name__ == '__main__':
    try:
        path_maker = PathMaker()
    except rospy.ROSInterruptException:
        pass
