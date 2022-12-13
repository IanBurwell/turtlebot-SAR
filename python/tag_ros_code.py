#!/usr/bin/env python3
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import TransformStamped
import rospy
import tf2_ros
import tag_compiler
import tf2_geometry_msgs
import aglomerate_apriltag_pose
import heapq as hq
import numpy as np


world_frame = 'map'
max_angle = np.deg2rad(30)
min_dis = 0.5
max_dis = 2

tag_dict = None
tag_queue = None
tag_dict = {}
tag_queue = []


def tag_pose_broadcaster(msg, tag_ID):
    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = world_frame
    t.child_frame_id = tag_ID
    t.transform.translation = msg.position
    t.transform.rotation = msg.orientation

    br.sendTransform(t)


def tag_detect_handle(msg):
    global tag_dict, tag_queue

    for tag_msg in msg.detections:
        tag_ID = f"tag_{tag_msg.id[0]}"

        T_w_c = tfBuffer.lookup_transform(world_frame, msg.header.frame_id, rospy.Time())
        T_w_tag = tf2_geometry_msgs.do_transform_pose(tag_msg.pose.pose, T_w_c)

        update_tag = tag_compiler.tag_detected(tag_ID, tag_msg.pose.pose.pose, T_w_tag, max_dis, max_angle, 
                                               max_dis, max_angle, tag_dict, tag_queue)

        if update_tag != False:
            tag_pose_broadcaster(update_tag[0], tag_ID)
        else:
            return False


if __name__ == '__main__':
    global tfBuffer
    rospy.init_node('tag_compiler')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rospy.Subscriber('/tag_detections', AprilTagDetectionArray, tag_detect_handle)

    rospy.loginfo("tag_compiler node started")

    rospy.spin()


