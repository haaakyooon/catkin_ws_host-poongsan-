#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2

def odom_callback(msg):
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "odom"
    msg.child_frame_id = "base_link"
    odom_pub.publish(msg)

def cloud_callback(msg):
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "odom"
    cloud_pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('odom_cloud_stamper')
    
    # Odometry stamper
    odom_pub = rospy.Publisher('/odometry/imu_stamped', Odometry, queue_size=10)
    rospy.Subscriber('/odometry/imu', Odometry, odom_callback)
    
    # PointCloud stamper
    cloud_pub = rospy.Publisher('/lio_sam/mapping/cloud_registered_stamped', PointCloud2, queue_size=10)
    rospy.Subscriber('/lio_sam/mapping/cloud_registered', PointCloud2, cloud_callback)
    
    rospy.loginfo("Odom and Cloud stamper node started")
    rospy.spin()