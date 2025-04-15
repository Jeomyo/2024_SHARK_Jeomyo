#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, os

from pyproj import Proj
from std_msgs.msg import Float32MultiArray
from morai_msgs.msg import GPSMessage



class GPS_to_UTM:
    def __init__(self):
        rospy.init_node('GPS_to_UTM', anonymous=True)
        self.gps_sub = rospy.Subscriber("/gps", GPSMessage, self.gps_callback)
        self.utm_pub = rospy.Publisher("/utm_coordinates", Float32MultiArray, queue_size=10)
        self.proj_UTM = Proj(proj='utm', zone=52, ellps = 'WGS84', preserve_units=False)

        self.utm_msg = Float32MultiArray()
        self.is_gps_data = False

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():

            self.is_gps_data = False
            rate.sleep()


    def gps_callback(self, gps_msg):
        self.is_gps_data = True
        latitude = gps_msg.latitude
        longitude = gps_msg.longitude
        altitude = gps_msg.altitude
        utm_xy = self.proj_UTM(longitude, latitude)
        utm_x = utm_xy[0]
        utm_y = utm_xy[1]
        map_x = utm_x - gps_msg.eastOffset
        map_y = utm_y - gps_msg.northOffset
        
        os.system('clear')
        print(f''' 
        ----------------[ GPS data ]----------------
            latitude    : {latitude}
            longitude   : {longitude}
            altitude    : {altitude}

                             |
                             | apply Projection (utm 52 zone)
                             V

        ------------------[ utm ]-------------------
              utm_x     : {utm_x}
              utm_y     : {utm_y}

                             |
                             | apply offset (east and north)
                             V
              
        ------------------[ map ]-------------------
        simulator map_x : {map_x}
        simulator map_y : {map_y}
        ''')



        utm_msg = Float32MultiArray()
        utm_msg.data = [map_x, map_y]
        self.utm_pub.publish(utm_msg)



if __name__ == '__main__':
    try:
        GPS_to_UTM = GPS_to_UTM()
    except rospy.ROSInterruptException:
        pass