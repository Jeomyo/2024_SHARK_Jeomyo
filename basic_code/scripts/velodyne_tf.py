#!/usr/bin/env python3
import rospy
import tf
from sensor_msgs.msg import LaserScan, PointCloud2
from laser_geometry import LaserProjection

class LidarToPointCloud:
    def __init__(self):
        rospy.init_node('lidar_to_pointcloud', anonymous=True)

        self.laser_projector = LaserProjection()
        self.listener = tf.TransformListener()

        # Subscribe to RPLIDAR scan data
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.pc_pub = rospy.Publisher('/velodyne_points', PointCloud2, queue_size=10)

    def scan_callback(self, scan_msg):
        try:
            cloud_msg = self.laser_projector.projectLaser(scan_msg)
            cloud_msg.header.frame_id = "velodyne"
            self.pc_pub.publish(cloud_msg)
        except Exception as e:
            rospy.logwarn(f"LaserScan to PointCloud2 conversion failed: {e}")

if __name__ == '__main__':
    try:
        LidarToPointCloud()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
