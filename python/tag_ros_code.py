#!/usr/bin/env python  
import rospy

import math
import tf2_ros
import geometry_msgs.msg
import tag_compiler
import aglomerate_apriltag_pose
import AprilTagDetection.msg
import heapq as hq


global tag_dict
global tag_queue
def tag_listener():

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)


def tag_pose_broadcaster(msg, tag_ID):

    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.Pose()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = world_frame
    t.child_frame_id = tag_ID
    t.transform = msg

    br.sendTransform(t)

def tag_detect_handle(msg):

    tag_ID = msg.id

    pose = msg.pose.pose
    covariance = msg.pose.covariance

    T_w_c = tf2_ros.lookup_transform(world_frame,camera_frame)
    T_w_tag = tf2_ros.do_transform_pose(pose, T_w_c)

    update_tag = tag_compiler.tag_detected(pose,T_w_tag,max_dis,max_angle,max_dis,max_angle,tag_dict,tag_queue)
    if update_tag != False:
        tag_pose_broadcaster(update_tag, tag_ID)
    else:
        return False

if __name__ == '__main__':
    global max_angle, min_dis,max_dis, world_frame,camera_frame
    world_frame = 'world'
    camera_frame = 'camera'
    tag_dict = {}
    tag_queue = hq.heapify([])
    rospy.init_node('tag compiler')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rospy.Subscriber('/tag_detections', AprilTagDetection.msg, tag_detect_handle)

    rospy.spin()



