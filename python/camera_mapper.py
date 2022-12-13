#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
import numpy as np
import tf
import cv2

pub = None
ogrid_base_msg = OccupancyGrid()
ogrid_base_data = []
ogrid_mask = None
ogrid_origin = None
ogrid_resolution = None
tf_listener = None

camera_fov = 62-10
camera_range = 2
camera_line_width = 2

def update_mask(laser_ranges):
    global ogrid_mask, ogrid_origin, ogrid_resolution

    # return if no map has been received yet
    if ogrid_mask is None or ogrid_origin is None or ogrid_resolution is None:
        return

    # lookup robot pose
    try:
        trans, rot = listener.lookupTransform('/map', '/base_link', rospy.Time(0))       
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logerr('Could not lookup robot transform')
        return    

    # calculate robot position in grid
    trans[0] = int((trans[0] - ogrid_origin[0])/ogrid_resolution)
    trans[1] = int((trans[1] - ogrid_origin[1])/ogrid_resolution)
    robot_angle = np.rad2deg(tf.transformations.euler_from_quaternion(rot)[2])

    # draw camera view into the mask
    # cv2.ellipse(ogrid_mask, (trans[0], trans[1]), (range,range), robot_angle, -camera_fov/2, camera_fov/2, int(True), -1)
    for r, theta in zip(laser_ranges, range(-camera_fov//2, camera_fov//2)):
        r = int(r/ogrid_resolution)
        cv2.line(ogrid_mask, (trans[0], trans[1]), 
                             (trans[0]+int(r*np.cos(np.deg2rad(theta+robot_angle))), trans[1]+int(r*np.sin(np.deg2rad(theta+robot_angle)))),
                             int(True), camera_line_width)


def occupancy_grid_callback(msg):
    global ogrid_mask, ogrid_origin, ogrid_resolution, ogrid_base_msg, ogrid_base_data

    # handle first map
    if ogrid_mask is None:
        ogrid_mask = np.full((msg.info.height, msg.info.width), False, dtype=np.int8)
        ogrid_origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        return

    # resize grid if necessary
    if msg.info.height != ogrid_mask.shape[0] or msg.info.width != ogrid_mask.shape[1]:
        # check in which direction the grid has to be resized
        if msg.info.origin.position.x < ogrid_origin[0]:
            # left
            ogrid_mask = np.concatenate((np.full((ogrid_mask.shape[0], msg.info.width - ogrid_mask.shape[1]), False, dtype=np.int8), ogrid_mask), axis=1)
        else:
            # right
            ogrid_mask = np.concatenate((ogrid_mask, np.full((ogrid_mask.shape[0], msg.info.width - ogrid_mask.shape[1]), False, dtype=np.int8)), axis=1)
        
        if msg.info.origin.position.y < ogrid_origin[1]:
            # bottom
            ogrid_mask = np.concatenate((np.full((msg.info.height - ogrid_mask.shape[0], ogrid_mask.shape[1]), False, dtype=np.int8), ogrid_mask), axis=0)
        else:
            # top
            ogrid_mask = np.concatenate((ogrid_mask, np.full((msg.info.height - ogrid_mask.shape[0], ogrid_mask.shape[1]), False, dtype=np.int8)), axis=0)

    # update ogrid trackers
    ogrid_base_msg.info = msg.info
    ogrid_base_data = msg.data
    ogrid_origin = (msg.info.origin.position.x, msg.info.origin.position.y)   
    ogrid_resolution = msg.info.resolution


def laser_scan_callback(msg):
    global camera_fov, camera_range
    # trim laser data
    ranges_rolled = np.roll(np.array(msg.ranges, dtype=np.float32), 180)

    # trim laser data
    laser_ranges = ranges_rolled[180-camera_fov//2:180+camera_fov//2]
    laser_ranges = np.where(laser_ranges > camera_range, camera_range, laser_ranges)

    update_mask(laser_ranges)


def publish_map():
    if ogrid_mask is None or len(ogrid_base_data) == 0:
        return

    # use mask to form new occupancy grid
    ogrid = np.where(ogrid_mask.flatten(), np.array(ogrid_base_data, dtype=np.int8), -1)

    # publish map
    ogrid_base_msg.header.stamp = rospy.Time.now()
    ogrid_base_msg.header.seq += 1
    ogrid_base_msg.header.frame_id = 'map'
    ogrid_base_msg.data = ogrid.tolist()
    pub.publish(ogrid_base_msg)



def setup():
    global pub, listener
    rospy.init_node('camera_mapper')

    # Create a subscriber for incoming occupancy grid messages
    rospy.Subscriber('map', OccupancyGrid, occupancy_grid_callback)
    # Create a subscriber for incoming laser scan messages
    rospy.Subscriber('scan', LaserScan, laser_scan_callback)

    
    # Create a publisher for the occupancy grid
    pub = rospy.Publisher('map_camera', OccupancyGrid, queue_size=10)

    # Create a tf listener
    listener = tf.TransformListener()


def loop():
    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        r.sleep()

        publish_map()
    # rospy.spin()


if __name__ == '__main__':
    done = False
    setup()
    while not done:
        try:    
            loop()
            done = True
        except rospy.ROSInterruptException:
            done = True
        except KeyboardInterrupt:
            done = True
        except:
            pass
        