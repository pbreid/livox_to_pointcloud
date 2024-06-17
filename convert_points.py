#!/usr/bin/env python

import sys
import os
import argparse
import rospy
import struct
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from std_msgs.msg import Header

# Ensure the path to the generated messages is in the Python path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'devel', 'lib', 'python3', 'dist-packages'))

from custom_msgs.msg import CustomMsg, CustomPoint  # Update the import statement

def custom_to_pointcloud2(custom_msg):
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = custom_msg.header.frame_id

    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        PointField('reflectivity', 12, PointField.UINT8, 1),
        PointField('tag', 13, PointField.UINT8, 1),
        PointField('line', 14, PointField.UINT8, 1),
    ]

    points = []
    for point in custom_msg.points:
        points.append([point.x, point.y, point.z, point.reflectivity, point.tag, point.line])

    pointcloud2_msg = point_cloud2.create_cloud(header, fields, points)
    pointcloud2_msg.is_dense = True  # Set is_dense to True if all points are valid
    return pointcloud2_msg

def callback(custom_msg):
    pointcloud2_msg = custom_to_pointcloud2(custom_msg)
    pointcloud2_pub.publish(pointcloud2_msg)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Convert custom point cloud message to PointCloud2.')
    parser.add_argument('--input_topic', type=str, required=True, help='Input topic for custom point cloud message')
    parser.add_argument('--output_topic', type=str, required=True, help='Output topic for PointCloud2 message')
    parser.add_argument('--node_name', type=str, required=True, help='Unique name for the ROS node')
    args = parser.parse_args()

    rospy.init_node(args.node_name)

    pointcloud2_pub = rospy.Publisher(args.output_topic, PointCloud2, queue_size=10)
    rospy.Subscriber(args.input_topic, CustomMsg, callback)

    rospy.spin()

