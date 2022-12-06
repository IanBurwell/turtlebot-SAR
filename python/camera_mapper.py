#!/usr/env/python3

import rospy
from nav_msgs.msg import OccupancyGrid
from rospy.numpy_msg import numpy_msg
import numpy as np
import tf
import cv2

pub = None
ogrid_mask = None
ogrid_origin = None
tf_listener = None

def occupancy_grid_callback(msg):
    global pub, ogrid_mask, ogrid_origin, tf_listener

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

    # update origin
    ogrid_origin = (msg.info.origin.position.x, msg.info.origin.position.y)   

    # lookup robot pose
    try:
        trans, rot = listener.lookupTransform('/map', '/base_link', rospy.Time(0))       
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logerr('Could not lookup transform')
        return    

    # calculate robot position in grid
    trans[0] = int((trans[0] - ogrid_origin[0])/msg.info.resolution)
    trans[1] = int((trans[1] - ogrid_origin[1])/msg.info.resolution)

    # draw camera view into the mask
    cam_fov = 62.2
    cam_range = int(2/msg.info.resolution)
    robot_angle = np.rad2deg(tf.transformations.euler_from_quaternion(rot)[2])
    cv2.ellipse(ogrid_mask, (trans[0], trans[1]), (cam_range,cam_range), robot_angle, -cam_fov/2, cam_fov/2, int(True), -1)

    # use mask to form new occupancy grid
    ogrid = np.where(ogrid_mask.flatten(), np.array(msg.data, dtype=np.int8), -1)

    # publish map
    msg.data = ogrid.tolist()
    pub.publish(msg)


def main():
    global pub, listener
    rospy.init_node('camera_mapper')

    # Create a subscriber for incoming occupancy grid messages
    rospy.Subscriber('map', OccupancyGrid, occupancy_grid_callback)
    
    # Create a publisher for the occupancy grid
    pub = rospy.Publisher('map_camera', OccupancyGrid, queue_size=10)

    # Create a tf listener
    listener = tf.TransformListener()

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass