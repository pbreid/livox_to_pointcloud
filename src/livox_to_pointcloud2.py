#!/usr/bin/env python3

import rospy
import rosbag
import signal
import sys
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
from custom_msgs.msg import CustomMsg  # Replace with the correct import for CustomMsg
from tqdm import tqdm

def custom_to_pointcloud2(custom_msg, frame_id, timestamp):
    header = Header()
    header.stamp = timestamp
    header.frame_id = frame_id

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

def signal_handler(sig, frame):
    rospy.loginfo('Interrupted! Exiting...')
    sys.exit(0)

def process_bag(input_bag, output_bag, topics):
    rospy.loginfo("Counting messages in the bag file...")
    
    # Get the total number of messages quickly using get_message_count
    with rosbag.Bag(input_bag, 'r') as bag:
        total_messages = bag.get_message_count()

    rospy.loginfo(f"Total messages to process: {total_messages}")

    with rosbag.Bag(output_bag, 'w') as outbag, tqdm(total=total_messages, desc="Processing messages", unit="msg") as pbar:
        for topic, msg, t in rosbag.Bag(input_bag).read_messages():
            # Only write the original message if it is not in the output topics list
            if topic not in topics.values():
                outbag.write(topic, msg, t)

            # Check if the message is from one of the specified input topics
            if topic in topics:
                pointcloud2_msg = custom_to_pointcloud2(msg, msg.header.frame_id, msg.header.stamp)
                output_topic = topics[topic]
                outbag.write(output_topic, pointcloud2_msg, t)

            pbar.update(1)

if __name__ == '__main__':
    rospy.init_node('process_bag_node')

    signal.signal(signal.SIGINT, signal_handler)

    input_bag = rospy.get_param('~input_bag')
    output_bag = rospy.get_param('~output_bag')
    
    # Define input topics and their corresponding output topics
    topics = {
        rospy.get_param('~input_topic1'): rospy.get_param('~output_topic1'),
        rospy.get_param('~input_topic2'): rospy.get_param('~output_topic2'),
    }

    rospy.loginfo("Starting bag processing...")
    process_bag(input_bag, output_bag, topics)
    rospy.loginfo("Finished bag processing.")
